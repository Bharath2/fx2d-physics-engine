#include <iostream>
#include <memory>
#include <string>

#include "Fx2D/Core.h"

int main(int, char**) {
    auto scene = FxYAML::buildScene("./Scene.yml");
    FxRylbRenderer renderer(scene, 60);
    renderer.run();
    return 0;
}
