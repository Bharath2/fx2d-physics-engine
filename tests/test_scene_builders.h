#pragma once

#include "Fx2D/Scene.h"

#include <memory>
#include <string>

// Shared entity construction for tests. Suites keep their own thin wrappers for the physics
// defaults they want; what lives here is the part that must not vary.

// Physics knobs a test may care about. Defaults are inert: full mass, no bounce, no friction.
struct FxBody {
    float mass = 1.0f;
    float elasticity = 0.0f;
    float static_friction = 0.0f;
    float dynamic_friction = 0.0f;
    bool ccd = false;
    bool sensor = false;
};

namespace fx_test_detail {

// Pose is set before the geometry so each shape is placed in the world as it is attached, and
// reset() afterwards leaves the cached bounding box valid before the first step.
inline void finish(FxScene& scene, const std::shared_ptr<FxEntity>& e, const FxVec2f& at,
                   const FxBody& body) {
    e->set_init_pose(FxVec3f{at.x(), at.y(), 0.0f});
    e->set_mass(body.mass);
    e->set_inertia();
    e->elasticity = body.elasticity;
    e->static_friction = body.static_friction;
    e->dynamic_friction = body.dynamic_friction;
    e->enable_ccd = body.ccd;
    e->is_sensor = body.sensor;
    scene.add_entity(e);
    e->reset();
}

} // namespace fx_test_detail

// Both geometries are always set. set_inertia() derives inertia from the *visual* shape, so a
// collision-only body silently takes the inertia of the default 0.5-radius circle, which makes
// every rotational result meaningless.
inline std::shared_ptr<FxEntity> add_box(FxScene& scene, const std::string& name, const FxVec2f& at,
                                         const FxVec2f& size, const FxBody& body = {}) {
    auto e = std::make_shared<FxEntity>(name);
    e->set_visual_geometry(FxVisualShape(size));
    e->set_collision_geometry(FxCollisionShape(size));
    fx_test_detail::finish(scene, e, at, body);
    return e;
}

inline std::shared_ptr<FxEntity> add_circle(FxScene& scene, const std::string& name,
                                            const FxVec2f& at, float radius,
                                            const FxBody& body = {}) {
    auto e = std::make_shared<FxEntity>(name);
    e->set_visual_geometry(FxVisualShape(radius));
    e->set_collision_geometry(FxCollisionShape(radius));
    fx_test_detail::finish(scene, e, at, body);
    return e;
}

// Pins a body in place: no mass, no inertia, no gravity, immune to contact impulses.
// enable_external_forces must come last, since it zeroes the inverse mass and inertia that
// set_mass and set_inertia would otherwise restore.
inline std::shared_ptr<FxEntity> make_static(const std::shared_ptr<FxEntity>& e) {
    e->set_mass(0.0f);
    e->set_inertia(0.0f);
    e->gravity_scale = 0.0f;
    e->enable_external_forces(false);
    return e;
}

inline FxScene make_scene(const FxVec2f& size, float gravity_y = -10.0f) {
    FxScene scene(
        FxVec2ui{static_cast<unsigned int>(size.x()), static_cast<unsigned int>(size.y())});
    scene.set_gravity(FxVec2f{0.0f, gravity_y});
    return scene;
}
