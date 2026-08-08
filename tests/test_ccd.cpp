#include "Fx2D/Entity.h"
#include "Fx2D/Math.h"
#include "Fx2D/Solver.h"

#include <cmath>
#include <iostream>
#include <memory>
#include <stdexcept>

namespace {

void require(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}

// Build a circle entity at (x, y) with radius r and velocity (vx, vy).
// A zero-timestep step() call is used to push the pose into the collision
// shape so that bounding_box() and collision_check() return valid data.
std::shared_ptr<FxEntity> make_circle(const std::string& name, float x, float y, float r,
                                      float vx = 0.0f, float vy = 0.0f) {
    auto e = std::make_shared<FxEntity>(name);
    e->set_mass(1.0f);
    e->set_inertia(1.0f);
    e->set_init_pose(FxVec3f{x, y, 0.0f});
    e->set_collision_geometry(FxCollisionShape(r));
    e->step(FxVec2f{0.0f, 0.0f}, 0.0); // zero-dt step to sync AABB
    e->velocity = FxVec3f{vx, vy, 0.0f};
    return e;
}

// Fast body would tunnel in one substep; CCD speculative contact must fire.
void test_fast_body_speculative_contact() {
    const float r = 0.5f;
    const float sep = 0.5f; // gap between surfaces
    const float speed = 1000.0f;
    const float dt = 1.0f / 600.0f;

    // e1 static at origin, e2 to the right approaching at -speed
    auto e1 = make_circle("static_wall", 0.0f, 0.0f, r);
    auto e2 = make_circle("fast_body", 2.0f * r + sep, 0.0f, r, -speed, 0.0f);
    e2->enable_ccd = true;

    // Static check must return invalid (bodies are separated)
    FxContact static_c = FxSolver::collision_check(e1, e2);
    require(!static_c.is_valid(), "static check should be invalid (bodies separated)");

    // Speculative check must return valid
    FxContact spec_c = FxSolver::speculative_contact_check(e1, e2, dt);
    require(spec_c.is_valid(), "speculative check must detect impending collision");
    require(spec_c.penetration_depth < 0.0f, "speculative depth must be negative (pre-contact)");
    require(spec_c.count >= 1, "speculative contact must have at least one contact point");
}

// Without enable_ccd, static check alone stays invalid (would tunnel).
void test_ccd_flag_off_no_regression() {
    const float r = 0.5f;
    const float sep = 0.5f;
    const float speed = 1000.0f;

    auto e1 = make_circle("wall2", 0.0f, 0.0f, r);
    auto e2 = make_circle("bullet2", 2.0f * r + sep, 0.0f, r, -speed, 0.0f);
    // enable_ccd intentionally left false

    FxContact static_c = FxSolver::collision_check(e1, e2);
    require(!static_c.is_valid(),
            "CCD off: static check must be invalid (bodies separated — would tunnel)");
}

// Separating bodies must not get a speculative contact.
void test_separating_bodies_no_speculative_contact() {
    const float r = 0.5f;
    const float sep = 0.5f;
    const float dt = 1.0f / 60.0f;

    auto e1 = make_circle("sep1", 0.0f, 0.0f, r, -100.0f, 0.0f); // moving left
    auto e2 = make_circle("sep2", 2.0f * r + sep, 0.0f, r, 100.0f, 0.0f); // moving right
    e1->enable_ccd = true;
    e2->enable_ccd = true;

    FxContact spec_c = FxSolver::speculative_contact_check(e1, e2, dt);
    require(!spec_c.is_valid(), "separating bodies must not generate a speculative contact");
}

} // namespace

void run_ccd_tests() {
    test_fast_body_speculative_contact();
    test_ccd_flag_off_no_regression();
    test_separating_bodies_no_speculative_contact();
    std::cout << "CCD tests passed." << std::endl;
}
