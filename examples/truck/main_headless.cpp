// Truck — Fx2D headless example
//
// Same scene and constraint rig as examples/truck/main.cpp, but stepped in a
// plain loop and reported as text instead of being handed to the renderer.
// Because it includes "Fx2D/Physics.h" rather than "Fx2D/Core.h", nothing here
// touches raylib / Dear ImGui / rlImGui — only yaml-cpp, Eigen and TBB.
//
// Build (from the repo root, no graphics stack required):
//   ./scripts/build_headless.sh
//   ./build-headless/truck_headless
//
// Scene.yml is resolved relative to the working directory, so run from the
// repo root.

#include "Fx2D/Physics.h"

#include <iomanip>
#include <iostream>
#include <memory>

int main() {
    FxScene scene = FxYAML::buildScene("examples/truck/Scene.yml");

    auto truck_head = scene.get_entity("truck_head");
    auto truck_back = scene.get_entity("truck_back");
    auto wheel1 = scene.get_entity("wheel1");
    auto wheel2 = scene.get_entity("wheel2");
    auto box = scene.get_entity("box");
    if (!truck_head || !truck_back || !wheel1 || !wheel2 || !box) {
        std::cerr << "ERROR: expected entities missing from examples/truck/Scene.yml\n";
        return 1;
    }

    // Keep truck head and back aligned horizontally.
    auto motion_constraint =
        std::make_shared<FxMotionAlongAxisConstraint>(truck_head, truck_back, FxVec2f(1.0f, 0.0f),
                                                      true);
    motion_constraint->setCompliance(1e-5);

    // Maintain fixed separation between truck head and back.
    auto separation_constraint =
        std::make_shared<FxSeparationConstraint>(truck_head, truck_back, FxVec2f(1.0f, 0.0f), true);
    separation_constraint->lower_limit = 0.0f;
    separation_constraint->upper_limit = 0.0f;
    separation_constraint->setCompliance(1e-5);

    // Lock relative angle between truck head and back.
    auto angle_lock = std::make_shared<FxAngleLockConstraint>(truck_head, truck_back);
    angle_lock->setCompliance(1e-5);

    // Attach wheels to truck head and back.
    auto wheel2_anchor =
        std::make_shared<FxAnchorConstraint>(truck_head, wheel2, FxVec2f(0.1f, -0.65f), true);
    auto wheel1_anchor =
        std::make_shared<FxAnchorConstraint>(truck_back, wheel1, FxVec2f(0.48f, -0.475f), true);

    scene.add_constraint(motion_constraint);
    scene.add_constraint(separation_constraint);
    scene.add_constraint(angle_lock);
    scene.add_constraint(wheel2_anchor);
    scene.add_constraint(wheel1_anchor);

    scene.disable_collision("wheel2", "truck_back");

    const double dt = 0.005;
    const int steps = 600; // 3 s
    const int print_every = 40; // every 0.2 s

    std::cout << std::fixed << std::setprecision(3);
    std::cout << "Truck headless run: " << steps << " steps of " << dt << " s\n"
              << "   t     head.x  head.y  head.th   w1.omega  w2.omega   box.y\n";

    auto report = [&](int step) {
        std::cout << std::setw(6) << step * dt << std::setw(9) << truck_head->pose.x()
                  << std::setw(8) << truck_head->pose.y() << std::setw(9) << truck_head->pose.z()
                  << std::setw(11) << wheel1->velocity.z() << std::setw(10) << wheel2->velocity.z()
                  << std::setw(8) << box->pose.y() << "\n";
    };

    report(0);
    for (int i = 1; i <= steps; ++i) {
        scene.step(dt);
        if (i % print_every == 0) report(i);
    }

    std::cout << "\nTruck settled at x=" << truck_head->pose.x() << " y=" << truck_head->pose.y()
              << " theta=" << truck_head->pose.z() << " rad\n";
    return 0;
}
