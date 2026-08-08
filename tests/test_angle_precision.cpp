#include "Fx2D/Physics.h"

#include <cmath>
#include <iostream>
#include <memory>
#include <stdexcept>

namespace {

void require(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}

bool approx(float a, float b, float eps = 1e-5f) {
    return std::fabs(a - b) <= eps;
}

// FxAngleWrap used to shift by +pi before the fmod, which rounds away any angle
// smaller than ~1.2e-7 rad (half an ulp of float near pi) and returns exactly 0.
// Substep rotations fall in that range at small timesteps, so this silently froze
// all angular motion. In-range angles must now pass through bit-exact.
void test_angle_wrap_preserves_tiny_angles() {
    const float tiny[] = {1e-8f, 5e-8f, 1e-7f, 5e-7f, 1e-6f, 1e-4f};
    for (float a : tiny) {
        require(FxAngleWrap(a) == a, "FxAngleWrap must preserve tiny positive angles exactly");
        require(FxAngleWrap(-a) == -a, "FxAngleWrap must preserve tiny negative angles exactly");
    }
    require(FxAngleWrap(0.0f) == 0.0f, "FxAngleWrap(0) must be 0");
}

void test_angle_wrap_preserves_in_range_angles() {
    const float vals[] = {-3.0f, -1.5f, -0.25f, 0.25f, 1.5f, 3.0f};
    for (float a : vals) {
        require(FxAngleWrap(a) == a, "FxAngleWrap must be the identity inside [-pi, pi)");
    }
    require(FxAngleWrap(-FxPif) == -FxPif, "FxAngleWrap(-pi) must stay -pi");
}

void test_angle_wrap_still_wraps_out_of_range() {
    require(approx(FxAngleWrap(FxPif), -FxPif), "FxAngleWrap(pi) must wrap to -pi");
    require(approx(FxAngleWrap(2.0f * FxPif + 0.5f), 0.5f), "2pi + 0.5 must wrap to 0.5");
    require(approx(FxAngleWrap(-2.0f * FxPif - 0.5f), -0.5f), "-2pi - 0.5 must wrap to -0.5");
    require(approx(FxAngleWrap(FxPif + 0.25f), -FxPif + 0.25f), "pi + 0.25 must wrap below -pi");
    for (float a : {-20.0f, -7.0f, 7.0f, 20.0f}) {
        float w = FxAngleWrap(a);
        require(w >= -FxPif && w < FxPif, "wrapped angle must land inside [-pi, pi)");
    }
}

// Physics-level guard: a constant torque must produce rotation at the smallest
// timestep FxScene accepts. This returned exactly zero before the wrap fix.
void test_torque_rotates_body_at_min_timestep() {
    FxScene scene({12, 8});
    scene.set_gravity({0.0f, 0.0f});

    auto body = std::make_shared<FxEntity>("spinner");
    body->set_mass(1.0f);
    body->set_inertia(0.5f);
    body->set_init_pose(FxVec3f{6.0f, 4.0f, 0.0f});
    body->gravity_scale = 0.0f;
    scene.add_entity(body);

    const double dt = 1e-3; // FxScene's minimum accepted step
    for (int i = 0; i < 100; ++i) {
        body->apply_torque(6.0f);
        scene.step(dt);
    }

    require(std::fabs(body->pose.theta()) > 1e-4f,
            "a constant torque must rotate the body at dt = 1e-3");
    require(std::fabs(body->velocity.theta()) > 1e-3f,
            "angular velocity must survive the substep integration at dt = 1e-3");
}

} // namespace

void run_angle_precision_tests() {
    test_angle_wrap_preserves_tiny_angles();
    test_angle_wrap_preserves_in_range_angles();
    test_angle_wrap_still_wraps_out_of_range();
    test_torque_rotates_body_at_min_timestep();
    std::cout << "Angle precision tests passed." << std::endl;
}
