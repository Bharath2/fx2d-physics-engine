// Joint Control Demo (visual) — Fx2D raylib example
//
// Same scene and motors as examples/joint_control_demo/main.cpp, but rendered
// in a window. A step callback cycles both joints through the three control
// modes on a timer, so the arm and the slider visibly change behaviour every
// few seconds and then repeat:
//
//   POSITION  — arm swings to alternating target angles; slider steps between
//               offsets along the rail
//   VELOCITY  — arm spins at a constant rate; slider tracks a constant speed
//               and reverses when it nears a travel limit
//   EFFORT    — arm is driven by raw torque (it keeps accelerating, since
//               nothing closes the loop); slider is pushed by a raw force
//
// Build and run from the repo root:
//   cmake -S . -B build -DFX2D_BUILD_EXAMPLES=ON && cmake --build build
//   ./build/example_joint_control

#include "Fx2D/Core.h"

#include <cmath>
#include <iostream>
#include <memory>

namespace {

constexpr double PHASE_SECONDS = 5.0;

enum class Phase { Position, Velocity, Effort };

const char* phase_name(Phase p) {
    switch (p) {
    case Phase::Position:
        return "POSITION  (arm -> target angle, slider -> target offset)";
    case Phase::Velocity:
        return "VELOCITY  (arm spins at 3.14 rad/s, slider at 1.0 m/s)";
    case Phase::Effort:
        return "EFFORT    (raw torque 15 N*m, raw force -6 N)";
    }
    return "";
}

} // namespace

int main() {
    FxScene scene = FxYAML::buildScene("examples/joint_control_demo/Scene.yml");

    auto motor = std::dynamic_pointer_cast<FxRevoluteJoint>(scene.get_joint("arm_motor"));
    auto slider = std::dynamic_pointer_cast<FxPrismaticJoint>(scene.get_joint("slider_motor"));
    if (!motor || !slider) {
        std::cerr << "ERROR: joints 'arm_motor' / 'slider_motor' missing from the scene\n";
        return 1;
    }

    int current_phase = -1; // forces setup on the first callback
    bool position_toggle = false;

    scene.set_step_callback([&](FxScene& s, double) {
        const int phase_index = static_cast<int>(s.time_elapsed() / PHASE_SECONDS) % 3;
        const auto phase = static_cast<Phase>(phase_index);

        // Within the position phase, alternate targets halfway through so the
        // arm and slider visibly travel rather than sitting on one setpoint.
        if (phase == Phase::Position) {
            const bool second_half =
                std::fmod(s.time_elapsed(), PHASE_SECONDS) > PHASE_SECONDS * 0.5;
            if (second_half != position_toggle) {
                position_toggle = second_half;
                motor->set_theta(position_toggle ? -1.2f : 1.2f, /*instant=*/false);
                slider->set_position(position_toggle ? -3.0f : 3.0f, /*instant=*/false);
            }
        }

        // Velocity mode has no limit handling of its own, so bounce the slider
        // off its travel limits to keep it on screen.
        if (phase == Phase::Velocity) {
            const float pos = slider->get_position();
            if (pos > 3.5f) slider->set_velocity(-1.0f, false);
            if (pos < -3.5f) slider->set_velocity(1.0f, false);
        }

        if (phase_index == current_phase) return;
        current_phase = phase_index;
        std::cout << "[t=" << s.time_elapsed() << "s] " << phase_name(phase) << "\n";

        switch (phase) {
        case Phase::Position:
            // Position loops want strong damping: D near 2*sqrt(P*inertia).
            motor->set_pid({6.0f, 0.05f, 3.0f});
            slider->set_pid({8.0f, 0.2f, 5.0f});
            motor->set_control_mode(ControlMode::POSITION);
            slider->set_control_mode(ControlMode::POSITION);
            position_toggle = true; // flips on the next callback, issuing targets
            break;

        case Phase::Velocity:
            // A velocity loop needs its own gains: the error is already a
            // rate, so the position-mode D term would stall the motor.
            motor->set_pid({6.0f, 0.2f, 0.0f});
            slider->set_pid({8.0f, 0.2f, 0.0f});
            motor->set_control_mode(ControlMode::VELOCITY);
            slider->set_control_mode(ControlMode::VELOCITY);
            motor->set_omega(3.14f, false);
            slider->set_velocity(1.0f, false);
            break;

        case Phase::Effort:
            motor->set_control_mode(ControlMode::EFFORT);
            slider->set_control_mode(ControlMode::EFFORT);
            motor->set_torque(15.0f);
            slider->set_force(-6.0f);
            break;
        }
    });

    std::cout << "Joint control demo — each control mode runs for " << PHASE_SECONDS
              << " s, then the cycle repeats.\n"
              << "Close the window to exit.\n";

    FxRylbRenderer renderer(scene, 60);
    renderer.run();
    return 0;
}
