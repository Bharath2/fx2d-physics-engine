// Fx2D step-time benchmark.
//
// Build:  cmake -S . -B build -DFX2D_BUILD_BENCH=ON -DCMAKE_BUILD_TYPE=Release
//         cmake --build build --target Fx2DBench
// Run:    ./build/Fx2DBench [steps_per_size]
//
// Reports wall time and CPU time per step. Both matter: a change that halves wall time by
// spending eight times the CPU is usually the wrong trade for a library that may be one
// subsystem among many, and is a bad trade for RL rollouts where many independent sims
// already saturate the machine. cpu/wall near 1.0 means serial; much above 1.0 means threads
// are being spun up, and is only worth it if wall time actually fell.
//
// This exists because ToDo item 7 requires an A/B measurement before any threading change.

#include "Fx2D/Physics.h"

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <iomanip>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

namespace {

double cpu_seconds() {
    return static_cast<double>(std::clock()) / CLOCKS_PER_SEC;
}

std::shared_ptr<FxEntity> add_box(FxScene& scene, const std::string& name, float cx, float cy,
                                  float w, float h, float mass) {
    auto e = std::make_shared<FxEntity>(name);
    // Both shapes: set_inertia() derives from the visual one.
    e->set_visual_geometry(FxVisualShape(FxVec2f{w, h}));
    e->set_collision_geometry(FxCollisionShape(FxVec2f{w, h}));
    e->set_init_pose(FxVec3f{cx, cy, 0.0f});
    e->set_mass(mass);
    e->set_inertia();
    e->elasticity = 0.0f;
    e->static_friction = 0.5f;
    e->dynamic_friction = 0.4f;
    scene.add_entity(e);
    return e;
}

struct Result {
    double wall_ms_per_step = 0.0;
    double cpu_ms_per_step = 0.0;
    double steps_per_second = 0.0;
};

// A loose grid of boxes settling onto the ground: contacts, stacking and sleeping all exercised.
Result bench_settling_boxes(int body_count, int steps) {
    FxScene scene({400.0f, 400.0f});
    scene.set_gravity(FxVec2f{0.0f, -10.0f});

    auto ground = add_box(scene, "ground", 200.0f, 1.0f, 380.0f, 1.0f, 1.0f);
    ground->set_mass(0.0f);
    ground->set_inertia(0.0f);
    ground->enable_external_forces(false);
    ground->gravity_scale = 0.0f;

    const int per_row = static_cast<int>(std::ceil(std::sqrt(static_cast<double>(body_count))));
    for (int i = 0; i < body_count; ++i) {
        const float x = 50.0f + static_cast<float>(i % per_row) * 1.5f;
        const float y = 2.0f + static_cast<float>(i / per_row) * 1.5f;
        add_box(scene, "b" + std::to_string(i), x, y, 1.0f, 1.0f, 1.0f);
    }

    // Warm up so first-touch allocation and tree construction are not timed.
    for (int i = 0; i < 20; ++i)
        scene.step(1.0 / 60.0);

    const auto wall_start = std::chrono::steady_clock::now();
    const double cpu_start = cpu_seconds();
    for (int i = 0; i < steps; ++i)
        scene.step(1.0 / 60.0);
    const double wall =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - wall_start).count();
    const double cpu = cpu_seconds() - cpu_start;

    Result r;
    r.wall_ms_per_step = wall / steps * 1e3;
    r.cpu_ms_per_step = cpu / steps * 1e3;
    r.steps_per_second = wall > 0.0 ? steps / wall : 0.0;
    return r;
}

} // namespace

int main(int argc, char** argv) {
    int steps = 300;
    if (argc > 1) {
        steps = std::atoi(argv[1]);
        if (steps <= 0) {
            std::cerr << "steps_per_size must be a positive integer\n";
            return 1;
        }
    }

    std::cout << std::fixed << std::setprecision(3);
    std::cout << "Fx2D step benchmark - settling boxes, " << steps << " steps per size\n";
    std::cout << "cpu/wall near 1.0 = serial. Above 1.0 is only a win if wall time fell too.\n\n";
    std::cout << "  bodies   wall/step    cpu/step   cpu/wall     steps/s\n";

    for (int n : {10, 50, 200, 400, 800, 1600, 3000}) {
        const Result r = bench_settling_boxes(n, steps);
        const double ratio = r.wall_ms_per_step > 0.0 ? r.cpu_ms_per_step / r.wall_ms_per_step : 0.0;
        std::cout << std::setw(8) << n << std::setw(10) << r.wall_ms_per_step << " ms"
                  << std::setw(10) << r.cpu_ms_per_step << " ms" << std::setw(11) << ratio
                  << std::setw(12) << r.steps_per_second << "\n";
    }
    return 0;
}
