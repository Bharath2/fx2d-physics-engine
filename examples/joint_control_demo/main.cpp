// Headless revolute + prismatic motors through POSITION / VELOCITY / EFFORT.
// Build: ./scripts/build_headless.sh && ./build-headless/joint_control_demo
// Visual twin: examples/joint_control_demo/main_visual.cpp

#include "Fx2D/Scene.h"
#include "Fx2D/YamlUtils.h"
#include <iostream>
#include <memory>

int main() {
    FxScene scene = FxYAML::buildScene("examples/joint_control_demo/Scene.yml");

    auto motor = std::dynamic_pointer_cast<FxRevoluteJoint>(scene.get_joint("arm_motor"));
    auto slider = std::dynamic_pointer_cast<FxPrismaticJoint>(scene.get_joint("slider_motor"));
    if (!motor) {
        std::cerr << "ERROR: revolute joint 'arm_motor' not found in scene\n";
        return 1;
    }
    if (!slider) {
        std::cerr << "ERROR: prismatic joint 'slider_motor' not found in scene\n";
        return 1;
    }

    // 10 ms step: offset-pivot motors lose authority at 1 ms (float resolution).
    const double dt = 0.01;
    const int steps = 200; // 2 s per phase
    const int print_every = 40;

    auto report = [&](int step) {
        std::cout << "  step " << step << "  theta=" << motor->get_theta()
                  << "  omega=" << motor->get_omega() << "  pos=" << slider->get_position()
                  << "  vel=" << slider->get_velocity() << "\n";
    };

    // POSITION — strong D near 2*sqrt(P*inertia) / 2*sqrt(P*mass).
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

    // VELOCITY — drop D; position-mode D would stall a rate loop.
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

    std::cout << "=== EFFORT phase    (torque = 15.0 N*m, force = -2.0 N) ===\n";
    motor->set_control_mode(ControlMode::EFFORT);
    slider->set_control_mode(ControlMode::EFFORT);
    motor->set_torque(15.0f);
    slider->set_force(-2.0f); // reverses the slider back toward the rail centre
    for (int i = 0; i < steps; ++i) {
        scene.step(dt);
        if (i % print_every == 0) report(i);
    }

    return 0;
}
