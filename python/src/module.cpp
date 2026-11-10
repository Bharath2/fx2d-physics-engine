// The fx2d Python module: scenes, bodies, joints, contacts and ray casts, plus step_many for
// stepping many independent scenes at once. Vectors cross the boundary as tuples; poses and
// velocities of a whole scene also come out as NumPy arrays.

#include "Fx2D/Scene.h"
#ifndef FX2D_NO_YAML
#include "Fx2D/YamlUtils.h"
#endif

#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>
#include <nanobind/stl/array.h>
#include <nanobind/stl/optional.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <exception>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace nb = nanobind;
using namespace nb::literals;

namespace {

using Vec2 = std::array<float, 2>;
using Vec3 = std::array<float, 3>;

FxVec2f to_fx(const Vec2& v) {
    return {v[0], v[1]};
}
FxVec3f to_fx(const Vec3& v) {
    return {v[0], v[1], v[2]};
}
Vec2 to_py(const FxVec2f& v) {
    return {v.x(), v.y()};
}
Vec3 to_py(const FxVec3f& v) {
    return {v.x(), v.y(), v.z()};
}

// What a contact or ray hit reports, copied out by value: the engine's own records borrow
// entity pointers that are only valid until the next step, which Python cannot be held to.
struct PyContact {
    std::string body1, body2;
    Vec2 normal{};
    std::vector<Vec2> points;
    float depth = 0.0f;
    float normal_impulse = 0.0f;
};

struct PyRayHit {
    std::string body;
    Vec2 point{};
    Vec2 normal{};
    float distance = 0.0f;
};

struct Material {
    std::optional<float> elasticity, static_friction, dynamic_friction;
};

std::shared_ptr<FxEntity> add_body(FxScene& scene, const std::string& name, FxShape shape,
                                   const Vec2& position, float angle, float mass, bool is_static,
                                   const Material& material) {
    if (scene.entity_exists(name))
        throw std::invalid_argument("a body named '" + name + "' already exists");
    auto e = std::make_shared<FxEntity>(name);
    e->set_visual_geometry(FxVisualShape(shape));
    e->set_collision_geometry(std::move(shape));
    e->set_init_pose(FxVec3f{position[0], position[1], angle});
    if (material.elasticity) e->elasticity = *material.elasticity;
    if (material.static_friction) e->static_friction = *material.static_friction;
    if (material.dynamic_friction) e->dynamic_friction = *material.dynamic_friction;
    if (is_static) {
        // Immovable: no mass, no inertia, no gravity, immune to contact impulses.
        e->set_mass(0.0f);
        e->set_inertia(0.0f);
        e->gravity_scale = 0.0f;
        e->enable_external_forces(false);
    } else {
        e->set_mass(mass);
        e->set_inertia();
    }
    if (!scene.add_entity(e)) throw std::runtime_error("could not add body '" + name + "'");
    e->reset();
    return e;
}

std::shared_ptr<FxEntity> body_named(const FxScene& scene, const std::string& name) {
    auto e = scene.get_entity(name);
    if (!e) throw nb::key_error(("no body named '" + name + "'").c_str());
    return e;
}

template<typename Get>
nb::ndarray<nb::numpy, float, nb::shape<-1, 3>> body_column(FxScene& scene, Get get) {
    const size_t n = scene.entity_count();
    auto* data = new float[n * 3];
    size_t i = 0;
    scene.for_each_entity(std::execution::seq, [&](const FxEntity* e) {
        const FxVec3f v = get(*e);
        data[3 * i] = v.x();
        data[3 * i + 1] = v.y();
        data[3 * i + 2] = v.z();
        ++i;
    });
    nb::capsule owner(data, [](void* p) noexcept { delete[] static_cast<float*>(p); });
    return nb::ndarray<nb::numpy, float, nb::shape<-1, 3>>(data, {n, 3}, owner);
}

// Workers kept from call to call: starting threads cost more than stepping a small scene, and
// vector environments call step_many every step. The calling thread works alongside them, and
// each scene goes to one thread, start to finish, so it gets the bits a single thread gives.
class StepPool {
  public:
    // `pool` threads in all, the caller included, of which `workers` take part in this call.
    void run(const std::vector<FxScene*>& scenes, double dt, int steps, size_t pool,
             size_t workers) {
        std::lock_guard<std::mutex> one_call(m_call); // calls from several Python threads queue
        resize(pool - 1);
        const size_t helpers = std::min(workers - 1, m_threads.size());
        {
            std::lock_guard<std::mutex> lock(m_mutex);
            m_scenes = &scenes;
            m_dt = dt;
            m_steps = steps;
            m_next = 0;
            m_error = nullptr;
            m_tickets = helpers;
            ++m_job;
        }
        for (size_t i = 0; i < helpers; ++i)
            m_wake.notify_one();
        drain();
        // Every scene is taken: helpers still asleep are not waited for, only those working.
        std::unique_lock<std::mutex> lock(m_mutex);
        m_tickets = 0;
        m_done.wait(lock, [this] { return m_busy == 0; });
        if (m_error) std::rethrow_exception(m_error);
    }

  private:
    void resize(size_t helpers) {
        if (m_threads.size() == helpers) return;
        {
            std::lock_guard<std::mutex> lock(m_mutex);
            m_stop = true;
        }
        m_wake.notify_all();
        for (auto& t : m_threads)
            t.join();
        m_threads.clear();
        m_stop = false;
        for (size_t i = 0; i < helpers; ++i)
            m_threads.emplace_back([this, job = m_job] { work(job); });
    }

    void work(uint64_t seen) {
        for (;;) {
            {
                std::unique_lock<std::mutex> lock(m_mutex);
                m_wake.wait(lock, [&] { return m_stop || m_job != seen; });
                if (m_stop) return;
                seen = m_job;
                if (m_tickets == 0) continue; // woke after the job was handed out
                --m_tickets;
                ++m_busy;
            }
            drain();
            std::lock_guard<std::mutex> lock(m_mutex);
            if (--m_busy == 0) m_done.notify_one();
        }
    }

    void drain() {
        for (size_t i = m_next.fetch_add(1); i < m_scenes->size(); i = m_next.fetch_add(1)) {
            try {
                for (int k = 0; k < m_steps; ++k)
                    (*m_scenes)[i]->step(m_dt);
            } catch (...) {
                std::lock_guard<std::mutex> lock(m_mutex);
                if (!m_error) m_error = std::current_exception();
            }
        }
    }

    std::mutex m_call, m_mutex;
    std::condition_variable m_wake, m_done;
    std::vector<std::thread> m_threads;
    const std::vector<FxScene*>* m_scenes = nullptr;
    double m_dt = 0.0;
    int m_steps = 0;
    std::atomic<size_t> m_next{0};
    size_t m_tickets = 0; // helpers that may still join the current job
    size_t m_busy = 0; // helpers that joined it and are not done
    uint64_t m_job = 0;
    bool m_stop = false;
    std::exception_ptr m_error;
};

void step_many(const std::vector<FxScene*>& scenes, double dt, int steps, int threads) {
    if (std::find(scenes.begin(), scenes.end(), nullptr) != scenes.end())
        throw std::invalid_argument("step_many: None is not a scene");
    std::vector<FxScene*> sorted(scenes);
    std::sort(sorted.begin(), sorted.end());
    if (std::adjacent_find(sorted.begin(), sorted.end()) != sorted.end())
        throw std::invalid_argument("step_many: a scene is listed twice");
    // By default a thread per four scenes, up to every core: waking a thread costs about as
    // much as stepping a small scene, so a few scenes go faster on fewer threads.
    const size_t pool = threads > 0 ? static_cast<size_t>(threads) :
                                      std::max(1u, std::thread::hardware_concurrency());
    const size_t wanted = threads > 0 ? pool : std::max<size_t>(1, scenes.size() / 4);
    const size_t workers = std::min({wanted, pool, scenes.size()});
    nb::gil_scoped_release release;
    if (workers <= 1) {
        for (FxScene* s : scenes)
            for (int k = 0; k < steps; ++k)
                s->step(dt);
        return;
    }
    // Never destroyed: joining threads while a process exits can hang on some platforms.
    static StepPool* step_pool = new StepPool();
    step_pool->run(scenes, dt, steps, pool, workers);
}

} // namespace

NB_MODULE(_fx2d, m) {
    m.doc() = "Fx2D: a deterministic 2D rigid-body physics engine";

    nb::enum_<ControlMode>(m, "ControlMode")
        .value("POSITION", ControlMode::POSITION)
        .value("VELOCITY", ControlMode::VELOCITY)
        .value("EFFORT", ControlMode::EFFORT);

    nb::class_<PyContact>(m, "Contact")
        .def_ro("body1", &PyContact::body1)
        .def_ro("body2", &PyContact::body2)
        .def_ro("normal", &PyContact::normal, "unit normal, pointing from body1 to body2")
        .def_ro("points", &PyContact::points)
        .def_ro("depth", &PyContact::depth)
        .def_ro("normal_impulse", &PyContact::normal_impulse,
                "summed over the points, as applied in the step's last substep")
        .def("__repr__", [](const PyContact& c) {
            return "<fx2d.Contact '" + c.body1 + "' '" + c.body2 + "', " +
                   std::to_string(c.points.size()) + " points>";
        });

    nb::class_<PyRayHit>(m, "RayHit")
        .def_ro("body", &PyRayHit::body)
        .def_ro("point", &PyRayHit::point)
        .def_ro("normal", &PyRayHit::normal)
        .def_ro("distance", &PyRayHit::distance)
        .def("__repr__", [](const PyRayHit& h) {
            return "<fx2d.RayHit '" + h.body + "' at distance " + std::to_string(h.distance) + ">";
        });

    nb::class_<FxEntity>(m, "Body")
        .def_prop_ro("name", &FxEntity::get_name)
        .def_prop_ro(
            "pose", [](const FxEntity& e) { return to_py(e.pose); }, "(x, y, angle)")
        .def_prop_ro("position", [](const FxEntity& e) { return to_py(e.pose.xy()); })
        .def_prop_ro("angle", [](const FxEntity& e) { return e.pose.theta(); })
        .def_prop_rw(
            "velocity", [](const FxEntity& e) { return to_py(e.velocity); },
            [](FxEntity& e, const Vec3& v) {
                e.velocity = to_fx(v);
                e.wake(); // a sleeper handed a velocity would otherwise ignore it
            },
            "(vx, vy, angular)")
        .def(
            "set_state",
            [](FxEntity& e, const Vec3& pose, const Vec3& velocity) {
                e.set_state(to_fx(pose), to_fx(velocity));
            },
            "pose"_a, "velocity"_a = Vec3{0.0f, 0.0f, 0.0f},
            "Place the body at a pose and velocity, awake, as if it started there.")
        .def_prop_ro("mass", &FxEntity::mass)
        .def_prop_ro("inertia", &FxEntity::inertia)
        .def_rw("elasticity", &FxEntity::elasticity)
        .def_rw("static_friction", &FxEntity::static_friction)
        .def_rw("dynamic_friction", &FxEntity::dynamic_friction)
        .def_rw("gravity_scale", &FxEntity::gravity_scale)
        .def_rw("damping", &FxEntity::vel_damping,
                "drag on speed and spin: about e^-1 of each is left after 1 / damping seconds")
        .def_rw("enabled", &FxEntity::enabled)
        .def_rw("is_sensor", &FxEntity::is_sensor)
        .def_rw("ccd", &FxEntity::enable_ccd)
        .def_prop_ro("sleeping", &FxEntity::is_sleeping)
        .def("wake", &FxEntity::wake)
        .def(
            "apply_force",
            [](FxEntity& e, const Vec2& force, std::optional<Vec2> point) {
                if (point) e.apply_force(to_fx(force), to_fx(*point));
                else e.apply_force(to_fx(force));
            },
            "force"_a, "point"_a = nb::none(), "Force for the next step, at a world point.")
        .def("apply_torque", &FxEntity::apply_torque, "torque"_a)
        .def(
            "apply_impulse",
            [](FxEntity& e, const Vec2& impulse, std::optional<Vec2> point) {
                if (point) e.apply_impulse(to_fx(impulse), to_fx(*point));
                else e.apply_impulse(to_fx(impulse));
            },
            "impulse"_a, "point"_a = nb::none())
        .def("__repr__", [](const FxEntity& e) { return "<fx2d.Body '" + e.get_name() + "'>"; });

    nb::class_<FxJoint>(m, "Joint")
        .def_prop_ro("name", &FxJoint::get_name)
        .def_prop_rw("control_mode", &FxJoint::get_control_mode, &FxJoint::set_control_mode)
        .def_prop_rw(
            "pid", [](const FxJoint& j) { return to_py(j.get_pid()); },
            [](FxJoint& j, const Vec3& pid) { j.set_pid(to_fx(pid)); }, "(p, i, d) gains")
        .def_prop_rw("max_effort", &FxJoint::get_max_effort, &FxJoint::set_max_effort);

    nb::class_<FxRevoluteJoint, FxJoint>(m, "RevoluteJoint")
        .def_prop_ro("angle", &FxRevoluteJoint::get_theta, "body2's angle relative to body1")
        .def_prop_ro("angular_velocity", &FxRevoluteJoint::get_omega)
        .def("set_angle", &FxRevoluteJoint::set_theta, "angle"_a, "instant"_a = true)
        .def("set_angular_velocity", &FxRevoluteJoint::set_omega, "omega"_a, "instant"_a = true)
        .def("set_torque", &FxRevoluteJoint::set_torque, "torque"_a);

    nb::class_<FxPrismaticJoint, FxJoint>(m, "PrismaticJoint")
        .def_prop_ro("position", &FxPrismaticJoint::get_position)
        .def_prop_ro("velocity", &FxPrismaticJoint::get_velocity)
        .def("set_position", &FxPrismaticJoint::set_position, "position"_a, "instant"_a = true)
        .def("set_velocity", &FxPrismaticJoint::set_velocity, "velocity"_a, "instant"_a = true)
        .def("set_force", &FxPrismaticJoint::set_force, "force"_a);

    nb::class_<FxScene>(m, "Scene")
        .def(
            "__init__",
            [](FxScene* s, const std::array<unsigned int, 2>& size, const Vec2& gravity) {
                if (size[0] == 0 || size[1] == 0)
                    throw std::invalid_argument("scene size must be positive");
                new (s) FxScene(FxVec2ui{size[0], size[1]});
                s->set_gravity(to_fx(gravity));
            },
            "size"_a = std::array<unsigned int, 2>{100, 100}, "gravity"_a = Vec2{0.0f, -9.81f},
            "A world spanning [0, width] x [0, height] in whole units; bodies bounce off its "
            "edges.")
#ifndef FX2D_NO_YAML
        .def_static(
            "from_yaml",
            [](const std::string& text_or_path) { return FxYAML::buildScene(text_or_path); },
            "text_or_path"_a, "Build a scene from a YAML file, or from YAML text.")
#endif
        .def(
            "add_box",
            [](FxScene& s, const std::string& name, const Vec2& position, const Vec2& size,
               float angle, float mass, bool is_static, std::optional<float> elasticity,
               std::optional<float> static_friction, std::optional<float> dynamic_friction) {
                return add_body(s, name, FxShape(FxVec2f{size[0], size[1]}), position, angle, mass,
                                is_static, {elasticity, static_friction, dynamic_friction});
            },
            "name"_a, "position"_a, "size"_a, "angle"_a = 0.0f, "mass"_a = 1.0f, "static"_a = false,
            "elasticity"_a = nb::none(), "static_friction"_a = nb::none(),
            "dynamic_friction"_a = nb::none())
        .def(
            "add_circle",
            [](FxScene& s, const std::string& name, const Vec2& position, float radius, float angle,
               float mass, bool is_static, std::optional<float> elasticity,
               std::optional<float> static_friction, std::optional<float> dynamic_friction) {
                return add_body(s, name, FxShape(radius), position, angle, mass, is_static,
                                {elasticity, static_friction, dynamic_friction});
            },
            "name"_a, "position"_a, "radius"_a, "angle"_a = 0.0f, "mass"_a = 1.0f,
            "static"_a = false, "elasticity"_a = nb::none(), "static_friction"_a = nb::none(),
            "dynamic_friction"_a = nb::none())
        .def(
            "add_revolute_joint",
            [](FxScene& s, const std::string& name, const std::string& body1,
               const std::string& body2, const Vec2& anchor, float angle_min, float angle_max) {
                auto j = std::make_shared<FxRevoluteJoint>(name, body_named(s, body1),
                                                           body_named(s, body2), to_fx(anchor),
                                                           angle_min, angle_max);
                if (!s.add_joint(j)) throw std::runtime_error("could not add joint '" + name + "'");
                return j;
            },
            "name"_a, "body1"_a, "body2"_a, "anchor"_a = Vec2{0.0f, 0.0f},
            "angle_min"_a = -3.14159f, "angle_max"_a = 3.14159f,
            "A hinge at `anchor`, in body1's frame. Joints start as position motors holding\n"
            "their start (P = 1); set control_mode to EFFORT for a free joint.")
        .def(
            "add_prismatic_joint",
            [](FxScene& s, const std::string& name, const std::string& body1,
               const std::string& body2, const Vec2& axis, float position_min, float position_max) {
                auto j = std::make_shared<FxPrismaticJoint>(name, body_named(s, body1),
                                                            body_named(s, body2), to_fx(axis),
                                                            position_min, position_max);
                if (!s.add_joint(j)) throw std::runtime_error("could not add joint '" + name + "'");
                return j;
            },
            "name"_a, "body1"_a, "body2"_a, "axis"_a = Vec2{1.0f, 0.0f},
            "position_min"_a = -1000.0f, "position_max"_a = 1000.0f,
            "A slider along `axis`, in body1's frame. Joints start as position motors holding\n"
            "their start (P = 1); set control_mode to EFFORT for a free joint.")
        .def("body", &body_named, "name"_a)
        .def("__getitem__", &body_named, "name"_a)
        .def("__contains__", &FxScene::entity_exists, "name"_a)
        .def(
            "joint",
            [](const FxScene& s, const std::string& name) {
                auto j = s.get_joint(name);
                if (!j) throw nb::key_error(("no joint named '" + name + "'").c_str());
                return j;
            },
            "name"_a)
        .def_prop_ro("bodies",
                     [](FxScene& s) {
                         std::vector<std::shared_ptr<FxEntity>> out;
                         s.for_each_entity(std::execution::seq, [&](const FxEntity* e) {
                             out.push_back(s.get_entity(e->get_name()));
                         });
                         return out;
                     })
        .def("disable_collision", &FxScene::disable_collision, "body1"_a, "body2"_a)
        .def("enable_collision", &FxScene::enable_collision, "body1"_a, "body2"_a)
        .def(
            "step",
            [](FxScene& s, double dt, int steps) {
                nb::gil_scoped_release release;
                for (int k = 0; k < steps; ++k)
                    s.step(dt);
            },
            "dt"_a = 1.0 / 60.0, "steps"_a = 1,
            "Advance `steps` steps of `dt` seconds each. A step is at least 0.001 s, and one "
            "longer\n"
            "than 0.06 s is shortened to 0.06 s; for longer spans, take more steps.")
        .def("reset", &FxScene::reset, "Back to the scene as it was when it first stepped.")
        .def_prop_ro("time", &FxScene::time_elapsed)
        .def(
            "set_gravity", [](FxScene& s, const Vec2& g) { s.set_gravity(to_fx(g)); }, "gravity"_a)
        .def("set_substeps", &FxScene::set_substeps, "substeps"_a)
        .def("set_velocity_passes", &FxScene::set_velocity_passes, "passes"_a)
        .def(
            "poses",
            [](FxScene& s) { return body_column(s, [](const FxEntity& e) { return e.pose; }); },
            "Every body's (x, y, angle), in the order of `bodies`.")
        .def(
            "velocities",
            [](FxScene& s) { return body_column(s, [](const FxEntity& e) { return e.velocity; }); },
            "Every body's (vx, vy, angular), in the order of `bodies`.")
        .def(
            "contacts",
            [](const FxScene& s) {
                std::vector<PyContact> out;
                for (const FxContact& c : s.contacts()) {
                    PyContact pc;
                    pc.body1 = c.entity1 ? c.entity1->get_name() : std::string();
                    pc.body2 = c.entity2 ? c.entity2->get_name() : std::string();
                    pc.normal = to_py(c.normal);
                    for (size_t i = 0; i < c.count; ++i) {
                        if (!FxSolver::contact_point_touches(c, i)) continue;
                        pc.points.push_back(to_py(c.position[i]));
                        pc.normal_impulse += c.jn_accumulated[i];
                    }
                    pc.depth = c.penetration_depth;
                    out.push_back(std::move(pc));
                }
                return out;
            },
            "The pairs touching after the last step.")
        .def(
            "raycast",
            [](const FxScene& s, const Vec2& origin, const Vec2& direction,
               float max_distance) -> std::optional<PyRayHit> {
                FxRayHit hit;
                if (!s.raycast(to_fx(origin), to_fx(direction), max_distance, hit))
                    return std::nullopt;
                return PyRayHit{hit.entity->get_name(), to_py(hit.point), to_py(hit.normal),
                                hit.distance};
            },
            "origin"_a, "direction"_a, "max_distance"_a, "The nearest hit, or None.");

    m.def("step_many", &step_many, "scenes"_a, "dt"_a = 1.0 / 60.0, "steps"_a = 1, "threads"_a = 0,
          "Step independent scenes in parallel, each on one thread, with the GIL released. "
          "threads=0 uses a thread per four scenes, up to every core. Each scene's results are "
          "identical to stepping it alone.");
}
