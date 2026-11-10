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
#include <nanobind/stl/variant.h>
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
#include <variant>
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

using BodyRef = std::variant<std::string, std::shared_ptr<FxEntity>>;
using JointRef = std::variant<std::string, std::shared_ptr<FxJoint>>;

FxVec2fArray to_fx(const std::vector<Vec2>& points) {
    FxVec2fArray out(points.size());
    for (size_t i = 0; i < points.size(); ++i)
        out[i] = to_fx(points[i]);
    return out;
}

nb::ndarray<nb::numpy, float, nb::shape<-1, 2>> to_numpy(const FxVec2fArray& points) {
    const size_t n = points.size();
    auto* data = new float[n * 2];
    for (size_t i = 0; i < n; ++i) {
        data[2 * i] = points[i].x();
        data[2 * i + 1] = points[i].y();
    }
    nb::capsule owner(data, [](void* p) noexcept { delete[] static_cast<float*>(p); });
    return nb::ndarray<nb::numpy, float, nb::shape<-1, 2>>(data, {n, 2}, owner);
}

const char* shape_kind(const FxShape& s) {
    if (s.is_circle()) return "circle";
    if (s.is_chain()) return "chain";
    if (s.is_edge()) return "edge";
    if (s.is_capsule()) return "capsule";
    return "polygon";
}

std::shared_ptr<FxEntity> body_named(const FxScene& scene, const std::string& name) {
    auto e = scene.get_entity(name);
    if (!e) throw nb::key_error(("no body named '" + name + "'").c_str());
    return e;
}

// A body argument is its name or the Body itself, and either must belong to this scene.
std::shared_ptr<FxEntity> resolve(const FxScene& scene, const BodyRef& ref) {
    if (const auto* name = std::get_if<std::string>(&ref)) return body_named(scene, *name);
    const auto& body = std::get<std::shared_ptr<FxEntity>>(ref);
    if (!body || scene.get_entity(body->get_name()) != body)
        throw std::invalid_argument("that body is not in this scene");
    return body;
}

std::shared_ptr<FxJoint> resolve(const FxScene& scene, const JointRef& ref) {
    const std::string name = std::holds_alternative<std::string>(ref) ?
                                 std::get<std::string>(ref) :
                                 std::get<std::shared_ptr<FxJoint>>(ref)->get_name();
    auto joint = scene.get_joint(name);
    if (!joint) throw nb::key_error(("no joint named '" + name + "'").c_str());
    return joint;
}

// The engine's contact records borrow raw pointers valid until the next step, which Python
// cannot be held to, so what they report is copied out with the bodies as shared handles.
std::shared_ptr<FxEntity> handle(const FxScene& scene, const FxEntity* e) {
    return e ? scene.get_entity(e->get_name()) : nullptr;
}

struct PyContact {
    std::shared_ptr<FxEntity> body1, body2;
    Vec2 normal{};
    std::vector<Vec2> points;
    float depth = 0.0f;
    float normal_impulse = 0.0f;
    bool sensor = false;
};

struct PyContactEvent {
    std::shared_ptr<FxEntity> body1, body2;
    bool sensor = false;
};

struct PyRayHit {
    std::shared_ptr<FxEntity> body;
    Vec2 point{};
    Vec2 normal{};
    float distance = 0.0f;
};

PyRayHit to_py(const FxRayHit& hit) {
    return {hit.entity, to_py(hit.point), to_py(hit.normal), hit.distance};
}

std::vector<PyContactEvent> to_py(const FxScene& scene, const std::vector<FxContactEvent>& in) {
    std::vector<PyContactEvent> out;
    out.reserve(in.size());
    for (const auto& ev : in)
        out.push_back({handle(scene, ev.entity1), handle(scene, ev.entity2), ev.is_trigger});
    return out;
}

struct Material {
    std::optional<float> elasticity, static_friction, dynamic_friction;
};

// Static bodies never move; kinematic ones move only by the velocity they are given. Neither
// is pushed by contacts, forces or gravity. Edges and chains have no area, so they are static.
std::shared_ptr<FxEntity> add_body(FxScene& scene, const std::string& name, FxShape shape,
                                   const Vec2& position, float angle, float mass, bool is_static,
                                   bool kinematic, const Material& material) {
    if (scene.entity_exists(name))
        throw std::invalid_argument("a body named '" + name + "' already exists");
    if (is_static && kinematic) throw std::invalid_argument("a body is static or kinematic");
    if ((shape.is_edge() || shape.is_chain()) && !is_static)
        throw std::invalid_argument("edges and chains have no area: add them with static=True");
    auto e = std::make_shared<FxEntity>(name);
    e->set_visual_geometry(FxVisualShape(shape));
    e->set_collision_geometry(std::move(shape));
    e->set_init_pose(FxVec3f{position[0], position[1], angle});
    if (material.elasticity) e->elasticity = *material.elasticity;
    if (material.static_friction) e->static_friction = *material.static_friction;
    if (material.dynamic_friction) e->dynamic_friction = *material.dynamic_friction;
    if (is_static) {
        e->set_mass(0.0f);
        e->set_inertia(0.0f);
    } else {
        e->set_mass(mass);
        e->set_inertia();
    }
    if (is_static || kinematic) {
        e->gravity_scale = 0.0f;
        e->enable_external_forces(false); // last: it zeroes what set_mass and set_inertia set
    }
    if (!scene.add_entity(e)) throw std::runtime_error("could not add body '" + name + "'");
    e->reset();
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

    nb::class_<FxShape>(m, "Shape", "A body's geometry, in the body's own frame.")
        .def_static(
            "box", [](const Vec2& size, float rounding) { return FxShape(to_fx(size), rounding); },
            "size"_a, "rounding"_a = 0.0f,
            "A width x height box, its corners rounded by `rounding`.")
        .def_static(
            "circle", [](float radius) { return FxShape(radius); }, "radius"_a)
        .def_static(
            "polygon",
            [](const std::vector<Vec2>& vertices, float rounding) {
                return FxShape(to_fx(vertices), rounding);
            },
            "vertices"_a, "rounding"_a = 0.0f,
            "A convex polygon. Its vertices are re-centred on their mean, which is where the "
            "body's position puts it.")
        .def_static(
            "capsule", [](float length, float radius) { return FxShape(length, radius); },
            "length"_a, "radius"_a,
            "A segment `length` long along the body's x axis, rounded by "
            "`radius`.")
        .def_static(
            "edge",
            [](const Vec2& start, const Vec2& end) { return FxShape(to_fx(start), to_fx(end)); },
            "start"_a, "end"_a, "A segment with no thickness, for static bodies.")
        .def_static(
            "chain",
            [](const std::vector<Vec2>& points) {
                return FxShape(to_fx(points), FxShape::ChainTag{});
            },
            "points"_a,
            "An open polyline of 3 or more points, for static terrain. It collides on one side "
            "only, to the left of the direction its points run.")
        .def_prop_ro("kind", &shape_kind, "circle, polygon, capsule, edge or chain")
        .def_prop_ro("rounding", &FxShape::skin_radius,
                     "the radius of a circle or capsule, or a polygon's rounding")
        .def_prop_ro(
            "vertices", [](const FxShape& s) { return to_numpy(s.__vertices()); },
            nb::rv_policy::move, "in the body's frame; none for a circle")
        .def("__repr__",
             [](const FxShape& s) { return std::string("<fx2d.Shape ") + shape_kind(s) + ">"; });

    nb::class_<FxEntity>(m, "Body",
                         "A rigid body. The same object comes back for the same name, and it "
                         "stays valid, if no longer simulated, after leaving its scene.")
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
        .def_prop_ro(
            "shape",
            [](const FxEntity& e) -> std::optional<FxShape> {
                if (!e.collision_geometry()) return std::nullopt;
                return FxShape(*e.collision_geometry());
            },
            "a copy of the body's collision shape, in its own frame")
        .def(
            "world_vertices",
            [](const FxEntity& e) {
                const auto& shape = e.collision_geometry();
                return to_numpy(shape ? shape->vertices() : FxVec2fArray(0));
            },
            "The shape's vertices where the body is now, as an (n, 2) array.")
        .def_rw("elasticity", &FxEntity::elasticity)
        .def_rw("static_friction", &FxEntity::static_friction)
        .def_rw("dynamic_friction", &FxEntity::dynamic_friction)
        .def_rw("gravity_scale", &FxEntity::gravity_scale)
        .def_rw("damping", &FxEntity::vel_damping,
                "drag on speed and spin: about e^-1 of each is left after 1 / damping seconds")
        .def_rw("enabled", &FxEntity::enabled)
        .def_rw("is_sensor", &FxEntity::is_sensor,
                "reports contacts but pushes nothing and is pushed by nothing")
        .def_rw("ccd", &FxEntity::enable_ccd)
        .def_rw("collision_group", &FxEntity::collision_group,
                "bodies sharing a negative group never collide with each other; 0 for none")
        .def_prop_ro("sleeping", &FxEntity::is_sleeping)
        .def("wake", &FxEntity::wake)
        .def(
            "to_world",
            [](const FxEntity& e, const Vec2& p) { return to_py(e.to_world_frame(to_fx(p))); },
            "point"_a, "A point in the body's frame, in the world's.")
        .def(
            "to_local",
            [](const FxEntity& e, const Vec2& p) { return to_py(e.to_entity_frame(to_fx(p))); },
            "point"_a, "A point in the world's frame, in the body's.")
        .def(
            "velocity_at",
            [](const FxEntity& e, const Vec2& p) {
                return to_py(e.velocity_at_world_point(to_fx(p)));
            },
            "point"_a, "The velocity of the body's material at a world point.")
        .def(
            "apply_force",
            [](FxEntity& e, const Vec2& force, std::optional<Vec2> point) {
                if (point) e.apply_force(to_fx(force), to_fx(*point));
                else e.apply_force(to_fx(force));
            },
            "force"_a, "point"_a = nb::none(),
            "A force for the whole of the next step, at the centre or at a world point.")
        .def("apply_torque", &FxEntity::apply_torque, "torque"_a,
             "A torque for the whole of the next step.")
        .def(
            "apply_impulse",
            [](FxEntity& e, const Vec2& impulse, std::optional<Vec2> point) {
                if (point) e.apply_impulse(to_fx(impulse), to_fx(*point));
                else e.apply_impulse(to_fx(impulse));
            },
            "impulse"_a, "point"_a = nb::none())
        .def("__repr__", [](const FxEntity& e) { return "<fx2d.Body '" + e.get_name() + "'>"; });

    nb::class_<PyContact>(m, "Contact")
        .def_ro("body1", &PyContact::body1)
        .def_ro("body2", &PyContact::body2)
        .def_ro("normal", &PyContact::normal, "unit normal, pointing from body1 to body2")
        .def_ro("points", &PyContact::points)
        .def_ro("depth", &PyContact::depth)
        .def_ro("normal_impulse", &PyContact::normal_impulse,
                "summed over the points, as applied in the step's last substep")
        .def_ro("sensor", &PyContact::sensor, "either body is a sensor, so nothing was pushed")
        .def("__repr__", [](const PyContact& c) {
            return "<fx2d.Contact '" + (c.body1 ? c.body1->get_name() : "") + "' '" +
                   (c.body2 ? c.body2->get_name() : "") + "', " + std::to_string(c.points.size()) +
                   " points>";
        });

    nb::class_<PyContactEvent>(m, "ContactEvent", "A pair that began or stopped touching.")
        .def_ro("body1", &PyContactEvent::body1)
        .def_ro("body2", &PyContactEvent::body2)
        .def_ro("sensor", &PyContactEvent::sensor);

    nb::class_<PyRayHit>(m, "RayHit")
        .def_ro("body", &PyRayHit::body)
        .def_ro("point", &PyRayHit::point)
        .def_ro("normal", &PyRayHit::normal)
        .def_ro("distance", &PyRayHit::distance)
        .def("__repr__", [](const PyRayHit& h) {
            return "<fx2d.RayHit '" + (h.body ? h.body->get_name() : "") + "' at distance " +
                   std::to_string(h.distance) + ">";
        });

    nb::class_<FxEntityGroup>(m, "Group",
                              "A named set of bodies, removed and enabled together, and by "
                              "default exempt from colliding with each other.")
        .def_prop_ro("name", &FxEntityGroup::get_name)
        .def_prop_ro("bodies", &FxEntityGroup::members)
        .def("__len__", &FxEntityGroup::size)
        .def("set_enabled", &FxEntityGroup::set_enabled, "enabled"_a);

    nb::class_<FxJoint>(m, "Joint")
        .def_prop_ro("name", &FxJoint::get_name)
        .def_prop_ro("body1", &FxJoint::get_entity1)
        .def_prop_ro("body2", &FxJoint::get_entity2)
        .def_rw("enabled", &FxJoint::enabled, "a disabled joint's motor stops driving it")
        .def_prop_rw("control_mode", &FxJoint::get_control_mode, &FxJoint::set_control_mode)
        .def_prop_rw(
            "pid", [](const FxJoint& j) { return to_py(j.get_pid()); },
            [](FxJoint& j, const Vec3& pid) { j.set_pid(to_fx(pid)); }, "(p, i, d) gains")
        .def_prop_rw("max_effort", &FxJoint::get_max_effort, &FxJoint::set_max_effort)
        .def("__repr__", [](const FxJoint& j) { return "<fx2d.Joint '" + j.get_name() + "'>"; });

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

    nb::class_<FxDistanceJoint, FxJoint>(m, "DistanceJoint",
                                         "Holds two anchors within [min_length, max_length] "
                                         "apart; rotation stays free.")
        .def_prop_ro("length", &FxDistanceJoint::get_length, "the anchors' separation now")
        .def_prop_ro("rate", &FxDistanceJoint::get_rate, "how fast the separation is changing")
        .def_prop_ro("min_length", &FxDistanceJoint::get_min_length)
        .def_prop_ro("max_length", &FxDistanceJoint::get_max_length)
        .def("set_limits", &FxDistanceJoint::set_limits, "min_length"_a, "max_length"_a,
             "New limits. A large change is solved within one substep, so reel in gradually.")
        .def("set_compliance", &FxDistanceJoint::set_compliance, "compliance"_a,
             "Metres of stretch per newton of load; 0 for a rigid link.")
        .def("set_length", &FxDistanceJoint::set_length, "length"_a, "instant"_a = true)
        .def("set_rate", &FxDistanceJoint::set_rate, "rate"_a, "instant"_a = true)
        .def("set_force", &FxDistanceJoint::set_force, "force"_a,
             "Positive pushes the anchors apart.");

    using Body = std::shared_ptr<FxEntity>;
    nb::class_<FxScene>(m, "Scene",
                        "A world of bodies and joints, held by name. Wherever a body or joint "
                        "is asked for, its name and its object both do.")
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
            "add_body",
            [](FxScene& s, const std::string& name, const FxShape& shape, const Vec2& position,
               float angle, float mass, bool is_static, bool kinematic,
               std::optional<float> elasticity, std::optional<float> static_friction,
               std::optional<float> dynamic_friction) {
                return add_body(s, name, shape, position, angle, mass, is_static, kinematic,
                                {elasticity, static_friction, dynamic_friction});
            },
            "name"_a, "shape"_a, "position"_a = Vec2{0.0f, 0.0f}, "angle"_a = 0.0f, "mass"_a = 1.0f,
            "static"_a = false, "kinematic"_a = false, "elasticity"_a = nb::none(),
            "static_friction"_a = nb::none(), "dynamic_friction"_a = nb::none(),
            "Add a body of any shape. A static body never moves; a kinematic one moves only by "
            "the velocity it is given. Neither is pushed by contacts, forces or gravity.")
        .def(
            "add_box",
            [](FxScene& s, const std::string& name, const Vec2& position, const Vec2& size,
               float angle, float mass, bool is_static, bool kinematic, float rounding,
               std::optional<float> elasticity, std::optional<float> static_friction,
               std::optional<float> dynamic_friction) {
                return add_body(s, name, FxShape(to_fx(size), rounding), position, angle, mass,
                                is_static, kinematic,
                                {elasticity, static_friction, dynamic_friction});
            },
            "name"_a, "position"_a, "size"_a, "angle"_a = 0.0f, "mass"_a = 1.0f, "static"_a = false,
            "kinematic"_a = false, "rounding"_a = 0.0f, "elasticity"_a = nb::none(),
            "static_friction"_a = nb::none(), "dynamic_friction"_a = nb::none())
        .def(
            "add_circle",
            [](FxScene& s, const std::string& name, const Vec2& position, float radius, float angle,
               float mass, bool is_static, bool kinematic, std::optional<float> elasticity,
               std::optional<float> static_friction, std::optional<float> dynamic_friction) {
                return add_body(s, name, FxShape(radius), position, angle, mass, is_static,
                                kinematic, {elasticity, static_friction, dynamic_friction});
            },
            "name"_a, "position"_a, "radius"_a, "angle"_a = 0.0f, "mass"_a = 1.0f,
            "static"_a = false, "kinematic"_a = false, "elasticity"_a = nb::none(),
            "static_friction"_a = nb::none(), "dynamic_friction"_a = nb::none())
        .def(
            "add_revolute_joint",
            [](FxScene& s, const std::string& name, const BodyRef& body1, const BodyRef& body2,
               const Vec2& anchor, float angle_min, float angle_max) {
                auto j =
                    std::make_shared<FxRevoluteJoint>(name, resolve(s, body1), resolve(s, body2),
                                                      to_fx(anchor), angle_min, angle_max);
                if (!s.add_joint(j)) throw std::runtime_error("could not add joint '" + name + "'");
                return j;
            },
            "name"_a, "body1"_a, "body2"_a, "anchor"_a = Vec2{0.0f, 0.0f},
            "angle_min"_a = -3.14159f, "angle_max"_a = 3.14159f,
            "A hinge at `anchor`, in body1's frame. Joints start as position motors holding\n"
            "their start (P = 1); set control_mode to EFFORT for a free joint.")
        .def(
            "add_prismatic_joint",
            [](FxScene& s, const std::string& name, const BodyRef& body1, const BodyRef& body2,
               const Vec2& axis, float position_min, float position_max) {
                auto j =
                    std::make_shared<FxPrismaticJoint>(name, resolve(s, body1), resolve(s, body2),
                                                       to_fx(axis), position_min, position_max);
                if (!s.add_joint(j)) throw std::runtime_error("could not add joint '" + name + "'");
                return j;
            },
            "name"_a, "body1"_a, "body2"_a, "axis"_a = Vec2{1.0f, 0.0f},
            "position_min"_a = -1000.0f, "position_max"_a = 1000.0f,
            "A slider along `axis`, in body1's frame. Joints start as position motors holding\n"
            "their start (P = 1); set control_mode to EFFORT for a free joint.")
        .def(
            "add_distance_joint",
            [](FxScene& s, const std::string& name, const BodyRef& body1, const BodyRef& body2,
               const Vec2& anchor1, const Vec2& anchor2, std::optional<float> min_length,
               std::optional<float> max_length, double compliance) {
                auto j =
                    std::make_shared<FxDistanceJoint>(name, resolve(s, body1), resolve(s, body2),
                                                      to_fx(anchor1), to_fx(anchor2),
                                                      min_length.value_or(-1.0f),
                                                      max_length.value_or(-1.0f));
                if (compliance > 0.0) j->set_compliance(compliance);
                if (!s.add_joint(j)) throw std::runtime_error("could not add joint '" + name + "'");
                return j;
            },
            "name"_a, "body1"_a, "body2"_a, "anchor1"_a = Vec2{0.0f, 0.0f},
            "anchor2"_a = Vec2{0.0f, 0.0f}, "min_length"_a = nb::none(),
            "max_length"_a = nb::none(), "compliance"_a = 0.0,
            "A link between anchors in each body's frame. Limits left out take the distance the\n"
            "anchors are apart now; equal limits make a rigid link.")
        .def(
            "add_rope_joint",
            [](FxScene& s, const std::string& name, const BodyRef& body1, const BodyRef& body2,
               const Vec2& anchor1, const Vec2& anchor2, std::optional<float> length) {
                auto j =
                    std::make_shared<FxDistanceJoint>(name, resolve(s, body1), resolve(s, body2),
                                                      to_fx(anchor1), to_fx(anchor2), 0.0f,
                                                      length.value_or(-1.0f));
                if (!s.add_joint(j)) throw std::runtime_error("could not add joint '" + name + "'");
                return j;
            },
            "name"_a, "body1"_a, "body2"_a, "anchor1"_a = Vec2{0.0f, 0.0f},
            "anchor2"_a = Vec2{0.0f, 0.0f}, "length"_a = nb::none(),
            "A rope: it stops the anchors parting further than `length`, by default the distance\n"
            "they are apart now, and goes slack when they come closer.")
        .def("body", &body_named, "name"_a)
        .def("__getitem__", &body_named, "name"_a)
        .def("__contains__", &FxScene::entity_exists, "name"_a)
        .def("__len__", &FxScene::entity_count)
        .def(
            "remove",
            [](FxScene& s, const BodyRef& body) { s.delete_entity(resolve(s, body)->get_name()); },
            "body"_a, "Take a body out of the scene, and the joints that hold it.")
        .def("__delitem__",
             [](FxScene& s, const std::string& name) {
                 if (!s.delete_entity(name))
                     throw nb::key_error(("no body named '" + name + "'").c_str());
             })
        .def(
            "joint", [](const FxScene& s, const JointRef& joint) { return resolve(s, joint); },
            "name"_a)
        .def(
            "remove_joint",
            [](FxScene& s, const JointRef& joint) {
                s.delete_joint(resolve(s, joint)->get_name());
            },
            "joint"_a)
        .def_prop_ro("bodies",
                     [](FxScene& s) {
                         std::vector<Body> out;
                         s.for_each_entity(std::execution::seq, [&](const FxEntity* e) {
                             out.push_back(s.get_entity(e->get_name()));
                         });
                         return out;
                     })
        .def(
            "add_group",
            [](FxScene& s, const std::string& name, const std::vector<BodyRef>& bodies,
               bool self_collide) {
                std::vector<Body> members;
                for (const auto& b : bodies)
                    members.push_back(resolve(s, b));
                auto group = s.create_group(name, self_collide);
                if (!group) throw std::invalid_argument("a group named '" + name + "' exists");
                for (const auto& b : members)
                    s.add_to_group(group, b);
                return group;
            },
            "name"_a, "bodies"_a = std::vector<BodyRef>{}, "self_collide"_a = false,
            "Group bodies under a name. Unless `self_collide`, they never collide with each "
            "other.")
        .def(
            "add_to_group",
            [](FxScene& s, const std::string& group, const BodyRef& body) {
                auto g = s.get_group(group);
                if (!g) throw nb::key_error(("no group named '" + group + "'").c_str());
                s.add_to_group(g, resolve(s, body));
            },
            "group"_a, "body"_a)
        .def(
            "group",
            [](const FxScene& s, const std::string& name) {
                auto g = s.get_group(name);
                if (!g) throw nb::key_error(("no group named '" + name + "'").c_str());
                return g;
            },
            "name"_a)
        .def(
            "remove_group",
            [](FxScene& s, const std::string& name) {
                if (!s.delete_group(name))
                    throw nb::key_error(("no group named '" + name + "'").c_str());
            },
            "name"_a, "Remove a group and every body in it.")
        .def(
            "disable_collision",
            [](FxScene& s, const BodyRef& a, const BodyRef& b) {
                s.disable_collision(resolve(s, a)->get_name(), resolve(s, b)->get_name());
            },
            "body1"_a, "body2"_a)
        .def(
            "enable_collision",
            [](FxScene& s, const BodyRef& a, const BodyRef& b) {
                s.enable_collision(resolve(s, a)->get_name(), resolve(s, b)->get_name());
            },
            "body1"_a, "body2"_a)
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
        .def("reset", &FxScene::reset,
             "Back to the scene as it was when it first stepped, or at capture_initial_state.")
        .def("capture_initial_state", &FxScene::capture_initial_state,
             "Make the bodies, joints and groups there are now what reset() restores. Taken at "
             "the first step if never called, so bodies added after it are removed by reset().")
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
                    pc.body1 = handle(s, c.entity1);
                    pc.body2 = handle(s, c.entity2);
                    pc.normal = to_py(c.normal);
                    for (size_t i = 0; i < c.count; ++i) {
                        if (!FxSolver::contact_point_touches(c, i)) continue;
                        pc.points.push_back(to_py(c.position[i]));
                        pc.normal_impulse += c.jn_accumulated[i];
                    }
                    pc.depth = c.penetration_depth;
                    pc.sensor =
                        (c.entity1 && c.entity1->is_sensor) || (c.entity2 && c.entity2->is_sensor);
                    out.push_back(std::move(pc));
                }
                return out;
            },
            "The pairs touching after the last step.")
        .def(
            "contacts_began", [](const FxScene& s) { return to_py(s, s.begin_contact_events()); },
            "Pairs that began touching during the last step.")
        .def(
            "contacts_ended", [](const FxScene& s) { return to_py(s, s.end_contact_events()); },
            "Pairs that stopped touching during the last step.")
        .def(
            "raycast",
            [](const FxScene& s, const Vec2& origin, const Vec2& direction,
               float max_distance) -> std::optional<PyRayHit> {
                FxRayHit hit;
                if (!s.raycast(to_fx(origin), to_fx(direction), max_distance, hit))
                    return std::nullopt;
                return to_py(hit);
            },
            "origin"_a, "direction"_a, "max_distance"_a, "The nearest hit, or None.")
        .def(
            "raycast_all",
            [](const FxScene& s, const Vec2& origin, const Vec2& direction, float max_distance) {
                std::vector<FxRayHit> hits;
                s.raycast_all(to_fx(origin), to_fx(direction), max_distance, hits);
                std::vector<PyRayHit> out;
                for (const auto& h : hits)
                    out.push_back(to_py(h));
                return out;
            },
            "origin"_a, "direction"_a, "max_distance"_a, "Every hit along the ray, nearest first.")
        .def(
            "overlap",
            [](const FxScene& s, const FxShape& shape, const Vec2& position, float angle) {
                std::vector<Body> out;
                s.overlap_shape(shape, FxVec3f{position[0], position[1], angle}, out);
                return out;
            },
            "shape"_a, "position"_a, "angle"_a = 0.0f, "The bodies a shape placed here overlaps.")
        .def(
            "overlap_circle",
            [](const FxScene& s, const Vec2& centre, float radius) {
                std::vector<Body> out;
                s.overlap_circle(to_fx(centre), radius, out);
                return out;
            },
            "centre"_a, "radius"_a)
        .def(
            "overlap_box",
            [](const FxScene& s, const Vec2& centre, const Vec2& size) {
                std::vector<Body> out;
                s.overlap_box(to_fx(centre), to_fx(size), out);
                return out;
            },
            "centre"_a, "size"_a, "The bodies overlapping an upright width x height box.")
        .def(
            "body_at",
            [](const FxScene& s, const Vec2& point) { return s.entity_at_point(to_fx(point)); },
            "point"_a, "The first body covering a point, or None.");

    m.def("step_many", &step_many, "scenes"_a, "dt"_a = 1.0 / 60.0, "steps"_a = 1, "threads"_a = 0,
          "Step independent scenes in parallel, each on one thread, with the GIL released. "
          "threads=0 uses a thread per four scenes, up to every core. Each scene's results are "
          "identical to stepping it alone.");
}
