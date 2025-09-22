#include <iostream>
#include <string>
#include <memory>

#include "Fx2D/Core.h"

int main(int, char**){
    // Load scene configuration from YAML file
    auto scene = FxYAML::buildScene("./Scene.yml");

    // // Get entities from the scene
    // auto truck_head = scene.get_entity("truck_head");
    // auto truck_back = scene.get_entity("truck_back");
    // auto wheel1 = scene.get_entity("wheel1");
    // auto wheel2 = scene.get_entity("wheel2");

    // Initialize renderer with 60 FPS target
    FxRylbRenderer renderer(scene, 60);    
 
    renderer.run(false);

    return 0;
}
