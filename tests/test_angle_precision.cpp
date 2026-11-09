#include "Fx2D/Scene.h"

#include "test_harness.h"

#include <bit>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>

namespace {

bool approx(float a, float b, float eps = 1e-5f) {
    return std::fabs(a - b) <= eps;
}

// In-range angles must pass through bit-exact (old wrap floored tiny substep rotations).
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

// An offset-anchor motor must keep authority at small timesteps. A correction below one ulp
// of the float pose once rounded away entirely: 1.97e-6 rad over a second, a dead motor.
float motor_rotation_over_a_second(float origin_x, double dt, int steps) {
    FxScene scene({400.0f, 40.0f});
    scene.set_gravity(FxVec2f{0.0f, 0.0f});

    auto base = std::make_shared<FxEntity>("base_link");
    base->set_visual_geometry(FxVisualShape(FxVec2f{0.4f, 0.4f}));
    base->set_init_pose(FxVec3f{origin_x, 4.0f, 0.0f});
    base->set_mass(0.0f);
    base->set_inertia(0.0f);
    base->enable_external_forces(false);
    base->gravity_scale = 0.0f;
    scene.add_entity(base);

    auto arm = std::make_shared<FxEntity>("arm");
    arm->set_visual_geometry(FxVisualShape(FxVec2f{2.0f, 0.25f}));
    arm->set_init_pose(FxVec3f{origin_x + 1.0f, 4.0f, 0.0f}); // centre 1.0 from the anchor
    arm->set_mass(1.0f);
    arm->set_inertia(0.5f);
    arm->gravity_scale = 0.0f;
    scene.add_entity(arm);

    auto joint = std::make_shared<FxRevoluteJoint>("motor", base, arm, FxVec2f{0.0f, 0.0f});
    joint->set_control_mode(ControlMode::VELOCITY);
    joint->set_pid(FxVec3f{6.0f, 0.0f, 0.0f});
    joint->set_omega(3.0f, false);
    scene.add_joint(joint);

    base->reset();
    arm->reset();

    const float theta0 = arm->pose.theta();
    for (int i = 0; i < steps; ++i)
        scene.step(dt);
    return std::fabs(arm->pose.theta() - theta0);
}

void test_offset_motor_keeps_authority_at_small_timesteps() {
    for (float origin_x : {0.5f, 7.0f, 300.0f}) {
        const float coarse = motor_rotation_over_a_second(origin_x, 1e-2, 100);
        const float fine = motor_rotation_over_a_second(origin_x, 1e-3, 1000);

        require(coarse > 1.0f, "the motor must turn the arm at dt=1e-2 from x=" +
                                   std::to_string(origin_x) + ", got " + std::to_string(coarse));
        require(fine > 1.0f, "the motor must keep its authority at dt=1e-3 from x=" +
                                 std::to_string(origin_x) + ", got " + std::to_string(fine) +
                                 " rad (it managed 1.97e-6 before the mixed-precision fix)");
        // The two timesteps must agree: shrinking dt must not change what the motor achieves.
        require(std::fabs(fine - coarse) < 0.25f * coarse,
                "coarse and fine timesteps must agree at x=" + std::to_string(origin_x) + ": " +
                    std::to_string(coarse) + " vs " + std::to_string(fine));
    }
}

// Distance from the origin must not decide how well a motor works.
void test_motor_authority_does_not_decay_with_distance() {
    const float near_origin = motor_rotation_over_a_second(0.5f, 1e-3, 1000);
    const float far_away = motor_rotation_over_a_second(300.0f, 1e-3, 1000);

    require(far_away > 0.7f * near_origin,
            "a motor 300 units from the origin must perform like one at the origin: " +
                std::to_string(near_origin) + " near vs " + std::to_string(far_away) + " far");
}

std::uint32_t ulp_distance(float a, float b) {
    auto ordered = [](float f) {
        const auto u = std::bit_cast<std::int32_t>(f);
        return u < 0 ? std::numeric_limits<std::int32_t>::min() - u : u;
    };
    const std::int64_t d = static_cast<std::int64_t>(ordered(a)) - ordered(b);
    return static_cast<std::uint32_t>(d < 0 ? -d : d);
}

// FxSinCos stands in for the C library's sin and cos, whose last bit differs between platforms.
void test_sincos_matches_the_c_library() {
    for (int i = -20000; i <= 20000; ++i) {
        const float a = static_cast<float>(i) * 1e-3f;
        float s, c;
        FxSinCos(a, s, c);
        const auto ref_s = static_cast<float>(std::sin(static_cast<double>(a)));
        const auto ref_c = static_cast<float>(std::cos(static_cast<double>(a)));
        require(ulp_distance(s, ref_s) <= 1 && ulp_distance(c, ref_c) <= 1,
                "FxSinCos(" + std::to_string(a) + ") must be within 1 ulp of sin and cos");
    }
}

// The output bits themselves are pinned, so a platform that rounds differently fails here first.
void test_sincos_bits_are_pinned() {
    struct Pin {
        float angle;
        std::uint32_t sin_bits, cos_bits;
    };
    const Pin pins[] = {
        {1e-7f, 0x33d6bf95u, 0x3f800000u},     {0.1f, 0x3dcc7577u, 0x3f7eb898u},
        {0.785398f, 0x3f3504f1u, 0x3f3504f5u}, {1.0f, 0x3f576aa4u, 0x3f0a5140u},
        {-2.5f, 0xbf193578u, 0xbf4d17bfu},     {3.14159f, 0x362a2217u, 0xbf800000u},
        {-3.14159f, 0xb62a2217u, 0xbf800000u}, {6.0f, 0xbe8f0f8cu, 0x3f75cdb8u},
        {100.0f, 0xbf01a12eu, 0x3f5cc0eeu},    {-1234.5f, 0xbe14e299u, 0xbf7d4796u},
    };
    for (const Pin& pin : pins) {
        float s, c;
        FxSinCos(pin.angle, s, c);
        require(std::bit_cast<std::uint32_t>(s) == pin.sin_bits &&
                    std::bit_cast<std::uint32_t>(c) == pin.cos_bits,
                "FxSinCos(" + std::to_string(pin.angle) + ") must give the pinned bits");
    }
    float s, c;
    FxSinCos(-0.0f, s, c);
    require(s == 0.0f && std::signbit(s) && c == 1.0f, "FxSinCos(-0) must give (-0, 1)");
    FxSinCos(std::numeric_limits<float>::quiet_NaN(), s, c);
    require(std::isnan(s) && std::isnan(c), "FxSinCos(NaN) must give NaN");
}

} // namespace

void run_angle_precision_tests() {
    test_sincos_matches_the_c_library();
    test_sincos_bits_are_pinned();
    test_offset_motor_keeps_authority_at_small_timesteps();
    test_motor_authority_does_not_decay_with_distance();
    test_angle_wrap_preserves_tiny_angles();
    test_angle_wrap_preserves_in_range_angles();
    test_angle_wrap_still_wraps_out_of_range();
    test_torque_rotates_body_at_min_timestep();
    std::cout << "Angle precision tests passed." << std::endl;
}
