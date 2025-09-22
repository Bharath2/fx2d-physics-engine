#include <iostream>
#include <string>
#include <memory>
#include <thread>
#include <chrono>

#include "Fx2D/Core.h"

int main(int, char**){
    // Load scene configuration from YAML file
    auto scene = FxYAML::buildScene("./Scene.yml");

    // Get entities from the scene
    auto truck_head = scene.get_entity("truck_head");
    auto truck_back = scene.get_entity("truck_back");
    auto wheel1 = scene.get_entity("wheel1");
    auto wheel2 = scene.get_entity("wheel2");

    // Create constraints to connect truck head and back
    // Keep truck head and back aligned horizontally
    auto motion_constraint = std::make_shared<FxMotionAlongAxisConstraint>(
                                        truck_head, truck_back, FxVec2f(1.0f, 0.0f), true);
    motion_constraint->compliance = 1e-5f;

    // Maintain fixed separation between truck head and back
    auto separation_constraint = std::make_shared<FxSeparationConstraint>(
                                         truck_head, truck_back, FxVec2f(1.0f, 0.0f), true);
    separation_constraint->lower_limit = 0.0f;
    separation_constraint->upper_limit = 0.0f;
    separation_constraint->compliance = 1e-5f;

    // Lock relative angle between truck head and back
    auto angle_lock = std::make_shared<FxAngleLockConstraint>(truck_head, truck_back);
    angle_lock->compliance = 1e-5f;

    // Attach wheels to truck head and back
    auto wheel2_anchor = std::make_shared<FxAnchorConstraint>(
                                 truck_head, wheel2, FxVec2f(0.1f, -0.65f), true);
    auto wheel1_anchor = std::make_shared<FxAnchorConstraint>(
                                 truck_back, wheel1, FxVec2f(0.48f, -0.475f), true);

    // Add all constraints to the scene
    scene.add_constraint(motion_constraint);
    scene.add_constraint(separation_constraint);
    scene.add_constraint(angle_lock);
    scene.add_constraint(wheel2_anchor);
    scene.add_constraint(wheel1_anchor);
    
    // Disable collision between wheel2 and truck_back to prevent interference
    scene.disable_collision("wheel2", "truck_back");

    // Initialize renderer with 60 FPS target
    FxRylbRenderer renderer(scene, 60);
    
    renderer.run();

    return 0;
}
