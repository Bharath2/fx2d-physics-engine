// Joint Control Demo — Fx2D headless example
//
// Demonstrates a revolute motor and a prismatic motor cycling through all three
// control modes:
//   POSITION  — arm rotates to a target angle (rad); slider moves to a target offset (m)
//   VELOCITY  — arm spins at a target angular velocity (rad/s); slider tracks a speed (m/s)
//   EFFORT    — arm is driven by a direct torque (N·m); slider by a direct force (N)
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

    auto motor  = std::dynamic_pointer_cast<FxRevoluteJoint>(scene.get_joint("arm_motor"));
    auto slider = std::dynamic_pointer_cast<FxPrismaticJoint>(scene.get_joint("slider_motor"));
    if (!motor) {
        std::cerr << "ERROR: revolute joint 'arm_motor' not found in scene\n";
        return 1;
    }
    if (!slider) {
        std::cerr << "ERROR: prismatic joint 'slider_motor' not found in scene\n";
        return 1;
    }

    // 10 ms fixed step. FxScene accepts steps down to 1 ms, but the joint
    // constraints stiffen as the substep shrinks (XPBD compliance scales with
    // 1/dt^2) and at 1 ms they cancel the revolute motor's torque outright, so
    // the arm never moves. See docs/joint_control.md.
    const double dt           = 0.01;
    const int    steps        = 200;   // 2 s per phase
    const int    print_every  = 40;

    auto report = [&](int step) {
        std::cout << "  step " << step
                  << "  theta=" << motor->get_theta()
                  << "  omega=" << motor->get_omega()
                  << "  pos="   << slider->get_position()
                  << "  vel="   << slider->get_velocity() << "\n";
    };

    // ---------------------------------------------------------------
    // POSITION phase — arm tracks theta = 1.0 rad, slider tracks 1.0 m
    // ---------------------------------------------------------------
    // Position loops want strong damping: D near 2*sqrt(P*inertia) / 2*sqrt(P*mass)
    // keeps the arm and slider from oscillating around the target.
    std::cout << "=== POSITION phase  (target theta = 1.0 rad, position = 1.0 m) ===\n";
    motor->set_pid({6.0f, 0.05f, 3.0f});
    slider->set_pid({8.0f, 0.2f, 5.0f});
    motor->set_control_mode(ControlMode::POSITION);
    slider->set_control_mode(ControlMode::POSITION);
    motor->set_theta(1.0f, /*instant=*/false);
    slider->set_position(1.0f, /*instant=*/false);
    for (int i = 0; i < steps; ++i) {
        scene.step(dt);
        if (i % print_every == 0) report(i);
    }

    // ---------------------------------------------------------------
    // VELOCITY phase — arm tracks omega = 3.14 rad/s, slider tracks 0.5 m/s
    // ---------------------------------------------------------------
    // A velocity loop needs its own gains: the error is already a rate, so the
    // position-mode D term would differentiate acceleration and stall the motor.
    std::cout << "=== VELOCITY phase  (target omega = 3.14 rad/s, velocity = 0.5 m/s) ===\n";
    motor->set_pid({6.0f, 0.2f, 0.0f});
    slider->set_pid({8.0f, 0.2f, 0.0f});
    motor->set_control_mode(ControlMode::VELOCITY);
    slider->set_control_mode(ControlMode::VELOCITY);
    motor->set_omega(3.14f, /*instant=*/false);
    slider->set_velocity(0.5f, /*instant=*/false);
    for (int i = 0; i < steps; ++i) {
        scene.step(dt);
        if (i % print_every == 0) report(i);
    }

    // ---------------------------------------------------------------
    // EFFORT phase — arm driven by torque 15.0 N·m, slider pushed back by -2.0 N
    // ---------------------------------------------------------------
    std::cout << "=== EFFORT phase    (torque = 15.0 N*m, force = -2.0 N) ===\n";
    motor->set_control_mode(ControlMode::EFFORT);
    slider->set_control_mode(ControlMode::EFFORT);
    motor->set_torque(15.0f);
    slider->set_force(-2.0f);  // reverses the slider back toward the rail centre
    for (int i = 0; i < steps; ++i) {
        scene.step(dt);
        if (i % print_every == 0) report(i);
    }

    return 0;
}
