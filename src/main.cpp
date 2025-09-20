#include "Fx2D/Core.h"

#include <iostream>
#include <string>
#include <memory>

int main(int, char**){
    auto scene = FxYAML::buildScene("./Scene_2.yml");
    
    FxRylbRenderer renderer(scene, 60);
    renderer.run(false);

    return 0;
}
