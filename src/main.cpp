#include <iostream>
#include <string>
#include <memory>

#include "Fx2D/Core.h"


// fxmath_probe.cpp
#include "Fx2D/math.h"

// If your tight loops are in member funcs/templates of FxArray<float>, call them here.
// Mark wrappers noinline so the codegen lives in THIS TU (so remarks land here).

// #if defined(_MSC_VER)
//   #define NOINLINE __declspec(noinline)
// #elif defined(__GNUC__) || defined(__clang__)
//   #define NOINLINE __attribute__((noinline))
// #else
//   #define NOINLINE
// #endif

// NOINLINE void fx_probe_add(FxArray<float>& a, float k) { a += k; }
// NOINLINE void fx_probe_mul(FxArray<float>& a, const FxArray<float>& b) { a *= b; }

// int main() {
//     std::cout << "started" << std::endl;
//     // FxArray<float> a(1<<16), b(1<<16);
//     // for (size_t i=0; i<a.size(); ++i) { a[i] = float(i)*0.5f; b[i] = 1.0f + float(i%7); }
//     // fx_probe_add(a, 3.14159f);
//     // fx_probe_mul(a, b);
//     // return int(a[123] != 0.0f);
//     return 0;
// }
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
