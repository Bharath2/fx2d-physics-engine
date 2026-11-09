// Fx2D step-time benchmark: Fx2DBench [steps] [scene-filter]. Build with -DFX2D_BUILD_BENCH=ON.
// Reports wall and CPU time per step plus the mean live contact count, which is the solver's
// real workload. Add -DFX2D_PROFILE=ON for the per-phase split. See docs/contributing.md.

#include "Fx2D/Profile.h"
#include "Fx2D/Scene.h"
#include "bench_scenes.h"

#include <chrono>
#include <cstdlib>
#include <ctime>
#include <iomanip>
#include <iostream>
#include <string>

namespace {

double cpu_seconds() {
    return static_cast<double>(std::clock()) / CLOCKS_PER_SEC;
}

struct Result {
    double wall_ms_per_step = 0.0;
    double cpu_ms_per_step = 0.0;
    double steps_per_second = 0.0;
    double mean_contacts = 0.0;
};

// Runs the timed loop. Warm-up is untimed so first-touch allocation, tree construction and the
// scene settling into contact do not distort the measurement.
Result run(FxScene& scene, int steps, int warmup) {
    const double dt = 1.0 / 60.0;
    for (int i = 0; i < warmup; ++i)
        scene.step(dt);

    FxProfile::reset();
    double contact_sum = 0.0;
    const auto wall_start = std::chrono::steady_clock::now();
    const double cpu_start = cpu_seconds();
    for (int i = 0; i < steps; ++i) {
        scene.step(dt);
        contact_sum += static_cast<double>(scene.contacts().size());
    }
    const double wall =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - wall_start).count();
    const double cpu = cpu_seconds() - cpu_start;

    Result r;
    r.wall_ms_per_step = wall / steps * 1e3;
    r.cpu_ms_per_step = cpu / steps * 1e3;
    r.steps_per_second = wall > 0.0 ? steps / wall : 0.0;
    r.mean_contacts = contact_sum / steps;
    return r;
}

void print_profile() {
    if (!FxProfile::enabled()) return;
    const std::size_t n = FxProfile::steps();
    if (n == 0) return;
    const double total = FxProfile::ms(FxProfile::StepTotal);
    std::cout << "    phase split (ms/step, share of step):\n";
    for (int i = 0; i < FxProfile::SlotCount; ++i) {
        const auto slot = static_cast<FxProfile::Slot>(i);
        const double slot_ms = FxProfile::ms(slot);
        const double share = total > 0.0 ? 100.0 * slot_ms / total : 0.0;
        std::cout << "      " << std::left << std::setw(18) << FxProfile::slot_name(slot)
                  << std::right << std::setw(9) << slot_ms / static_cast<double>(n) << " ms"
                  << std::setw(8) << share << " %\n";
    }
}

} // namespace

int main(int argc, char** argv) {
    int steps = 300;
    if (argc > 1) {
        steps = std::atoi(argv[1]);
        if (steps <= 0) {
            std::cerr << "usage: Fx2DBench [steps] [scene-filter]\n";
            return 1;
        }
    }
    const std::string filter = (argc > 2) ? argv[2] : "";

    std::cout << std::fixed << std::setprecision(3);
    std::cout << "Fx2D step benchmark - " << steps << " steps per size\n";
    std::cout << "cpu/wall near 1.0 = serial. Above 1.0 is only a win if wall time fell too.\n";
    if (FxProfile::enabled()) std::cout << "profiler: ON\n";

    for (const FxBench::Scene& scene : FxBench::scenes()) {
        if (!filter.empty() && filter != scene.name) continue;
        std::cout << "\n== " << scene.name << " ==\n";
        std::cout << "  bodies   wall/step    cpu/step   cpu/wall     steps/s    contacts\n";
        for (int n : scene.sizes) {
            FxScene world = scene.build(n);
            const Result r = run(world, steps, scene.warmup_steps);
            const double ratio =
                r.wall_ms_per_step > 0.0 ? r.cpu_ms_per_step / r.wall_ms_per_step : 0.0;
            std::cout << std::setw(8) << n << std::setw(10) << r.wall_ms_per_step << " ms"
                      << std::setw(10) << r.cpu_ms_per_step << " ms" << std::setw(11) << ratio
                      << std::setw(12) << r.steps_per_second << std::setw(12) << r.mean_contacts
                      << "\n";
            print_profile();
        }
    }
    return 0;
}
