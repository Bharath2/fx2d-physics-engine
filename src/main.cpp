#include "Core.h"
#include "Renderer.h"
#include "YamlUtils.h"

#include <iostream>
#include <string>


// rewrite FxShape with inbuilt add_offset, get_bounds, and rotate. 
// and also write sat_collision using vectorized ops
// think about collison callback and step callback, provide as virtual override methods or as attributes



// custom function is called every time step
// void Fx_sceneloop(FxScene& scene, float dt){


// }


int main(int, char**){
    auto scene = FxYAML::buildScene("./Scene.yml");
    Renderer renderer(scene, 120);
    renderer.run();

    return 0;
}
