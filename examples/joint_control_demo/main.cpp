// Joint Control Demo — Fx2D headless example
//
// Demonstrates a revolute motor joint cycling through all three control modes:
//   POSITION  — arm rotates to a target angle (radians)
//   VELOCITY  — arm spins at a target angular velocity (rad/s)
//   EFFORT    — arm is driven by a direct torque (N·m)
//
// Run procedure:
//   1. Copy this file to src/main.cpp (replacing the existing placeholder)
//   2. Build with CMake per the README (from the repo root)
//   3. Run ./build/Fx2D from the repo root — Scene.yml is loaded by path
//      relative to the working directory, so run from the repo root, not
//      from the build/ directory.
//
// Visual version: replace the step loops below with
//   FxRylbRenderer(scene, 60).run()
// — the joint API calls above the loop are identical.

#include "Fx2D/Core.h"
#include <iostream>
#include <memory>

int main() {
    FxScene scene = FxYAML::buildScene("examples/joint_control_demo/Scene.yml");

    std::shared_ptr<FxJoint> j = scene.get_joint("arm_motor");
    if (!j) {
        std::cerr << "ERROR: joint 'arm_motor' not found in scene\n";
        return 1;
    }
    auto motor = std::dynamic_pointer_cast<FxRevoluteJoint>(j);
    if (!motor) {
        std::cerr << "ERROR: 'arm_motor' is not a revolute joint\n";
        return 1;
    }

    const double dt           = 0.001; // 1 ms fixed step (minimum allowed by FxScene)
    const int    steps        = 500;   // 0.5 s per phase
    const int    print_every  = 100;

    // ---------------------------------------------------------------
    // POSITION phase — arm tracks theta = 1.0 rad
    // ---------------------------------------------------------------
    std::cout << "=== POSITION phase  (target theta = 1.0 rad) ===\n";
    motor->set_control_mode(ControlMode::POSITION);
    motor->set_theta(1.0f, /*instant=*/false);
    for (int i = 0; i < steps; ++i) {
        scene.step(dt);
        if (i % print_every == 0) {
            std::cout << "  step " << i
                      << "  theta=" << motor->get_theta()
                      << "  omega=" << motor->get_omega() << "\n";
        }
    }

    // ---------------------------------------------------------------
    // VELOCITY phase — arm tracks omega = 3.14 rad/s
    // ---------------------------------------------------------------
    std::cout << "=== VELOCITY phase  (target omega = 3.14 rad/s) ===\n";
    motor->set_control_mode(ControlMode::VELOCITY);
    motor->set_omega(3.14f, /*instant=*/false);
    for (int i = 0; i < steps; ++i) {
        scene.step(dt);
        if (i % print_every == 0) {
            std::cout << "  step " << i
                      << "  theta=" << motor->get_theta()
                      << "  omega=" << motor->get_omega() << "\n";
        }
    }

    // ---------------------------------------------------------------
    // EFFORT phase — arm driven by direct torque 15.0 N·m
    // ---------------------------------------------------------------
    std::cout << "=== EFFORT phase    (torque = 15.0 N*m) ===\n";
    motor->set_control_mode(ControlMode::EFFORT);
    motor->set_torque(15.0f);
    for (int i = 0; i < steps; ++i) {
        scene.step(dt);
        if (i % print_every == 0) {
            std::cout << "  step " << i
                      << "  theta=" << motor->get_theta()
                      << "  omega=" << motor->get_omega() << "\n";
        }
    }

    return 0;
}
