#include "Fx2D/Core.h"

#include <iostream>
#include <string>
#include <memory>

int main(int, char**){
    // Load scene configuration from YAML file
    auto scene = FxYAML::buildScene("./Scene.yml");
    
    // Initialize renderer with 60 FPS target
    FxRylbRenderer renderer(scene, 60);
    // Start the simulation loop
    renderer.run(false);

    return 0;
}
