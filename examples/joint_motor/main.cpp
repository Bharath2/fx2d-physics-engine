#include "Fx2D/Core.h"

#include <iomanip>
#include <iostream>
#include <memory>

int main() {
    auto scene = FxYAML::buildScene("./Scene.yml");

    // self-check: both named joints must be present after load
    if (scene.joint_count() != 2u ||
        !scene.joint_exists("arm_motor") ||
        !scene.joint_exists("slider_motor")) {
        std::cerr << "Error: expected 2 joints (arm_motor, slider_motor); got "
                  << scene.joint_count() << "\n";
        return 1;
    }

    auto rev = std::dynamic_pointer_cast<FxRevoluteJoint>(scene.get_joint("arm_motor"));
    auto pri = std::dynamic_pointer_cast<FxPrismaticJoint>(scene.get_joint("slider_motor"));

    if (!rev || !pri) {
        std::cerr << "Error: joint type cast failed — check type fields in Scene.yml\n";
        return 1;
    }

    const double dt    = 0.01; // within Scene step clamp [1e-3, 0.06]
    const int    steps = 200;

    std::cout << std::fixed << std::setprecision(4);

    // --- POSITION: PID drives each joint to an angle / translation target ---
    std::cout << "=== POSITION ===\n";
    const float p_rev_start  = rev->get_theta();
    const float p_pri_start  = pri->get_position();
    const float p_rev_target = 0.8f;   // radians
    const float p_pri_target = 1.5f;   // metres along axis
    rev->set_control_mode(ControlMode::POSITION);
    pri->set_control_mode(ControlMode::POSITION);
    rev->set_theta(p_rev_target, false);       // instant=false: PID drives, no teleport
    pri->set_position(p_pri_target, false);
    for (int i = 0; i < steps; ++i) scene.step(dt);
    std::cout << "  arm_motor:    start=" << p_rev_start
              << " target=" << p_rev_target
              << " end="    << rev->get_theta() << " rad\n";
    std::cout << "  slider_motor: start=" << p_pri_start
              << " target=" << p_pri_target
              << " end="    << pri->get_position() << " m\n";

    // --- VELOCITY: PID drives each joint to a constant angular / linear rate ---
    std::cout << "=== VELOCITY ===\n";
    const float v_rev_start  = rev->get_omega();
    const float v_pri_start  = pri->get_velocity();
    const float v_rev_target = 2.0f;   // rad/s
    const float v_pri_target = 1.5f;   // m/s
    rev->set_control_mode(ControlMode::VELOCITY);
    pri->set_control_mode(ControlMode::VELOCITY);
    rev->set_omega(v_rev_target, false);       // instant=false: PID drives, no teleport
    pri->set_velocity(v_pri_target, false);
    for (int i = 0; i < steps; ++i) scene.step(dt);
    std::cout << "  arm_motor:    start=" << v_rev_start
              << " target=" << v_rev_target
              << " end="    << rev->get_omega() << " rad/s\n";
    std::cout << "  slider_motor: start=" << v_pri_start
              << " target=" << v_pri_target
              << " end="    << pri->get_velocity() << " m/s\n";

    // --- EFFORT: fixed torque / force applied each step, no PID ---
    std::cout << "=== EFFORT ===\n";
    const float e_rev_start  = rev->get_omega();
    const float e_pri_start  = pri->get_velocity();
    const float e_rev_torque = 3.0f;   // N·m
    const float e_pri_force  = 5.0f;   // N
    rev->set_control_mode(ControlMode::EFFORT);
    pri->set_control_mode(ControlMode::EFFORT);
    rev->set_torque(e_rev_torque);
    pri->set_force(e_pri_force);
    for (int i = 0; i < steps; ++i) scene.step(dt);
    std::cout << "  arm_motor:    start=" << e_rev_start
              << " torque=" << e_rev_torque
              << " end="    << rev->get_omega() << " rad/s\n";
    std::cout << "  slider_motor: start=" << e_pri_start
              << " force="  << e_pri_force
              << " end="    << pri->get_velocity() << " m/s\n";

    return 0;
}
