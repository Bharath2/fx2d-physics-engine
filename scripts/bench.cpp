// Fx2D step-time benchmark: Fx2DBench [steps] [scene-filter]. Build with -DFX2D_BUILD_BENCH=ON.
// Reports wall and CPU time per step plus the mean live contact count, which is the solver's
// real workload. Add -DFX2D_PROFILE=ON for the per-phase split. See docs/CONTRIBUTING.md.

#include "Fx2D/Profile.h"
#include "Fx2D/Scene.h"

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

// Kept local rather than sharing the test builders: the benchmark is a standalone tool and
// should not depend on test scaffolding.
std::shared_ptr<FxEntity> add_box(FxScene& scene, const std::string& name, float cx, float cy,
                                  float w, float h, float mass) {
    auto e = std::make_shared<FxEntity>(name);
    e->set_visual_geometry(FxVisualShape(FxVec2f{w, h}));
    e->set_collision_geometry(FxCollisionShape(FxVec2f{w, h}));
    e->set_init_pose(FxVec3f{cx, cy, 0.0f});
    e->set_mass(mass);
    e->set_inertia();
    e->static_friction = 0.5f;
    e->dynamic_friction = 0.4f;
    scene.add_entity(e);
    return e;
}

std::shared_ptr<FxEntity> add_circle(FxScene& scene, const std::string& name, float cx, float cy,
                                     float radius, float mass) {
    auto e = std::make_shared<FxEntity>(name);
    e->set_visual_geometry(FxVisualShape(radius));
    e->set_collision_geometry(FxCollisionShape(radius));
    e->set_init_pose(FxVec3f{cx, cy, 0.0f});
    e->set_mass(mass);
    e->set_inertia();
    e->static_friction = 0.4f;
    e->dynamic_friction = 0.3f;
    scene.add_entity(e);
    return e;
}

// Immovable scenery: zero mass, no external forces, no gravity.
void make_static(const std::shared_ptr<FxEntity>& e) {
    e->set_mass(0.0f);
    e->set_inertia(0.0f);
    e->enable_external_forces(false);
    e->gravity_scale = 0.0f;
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

// A loose grid of boxes settling onto the ground. Sparse contacts; dominated by per-body work.
Result bench_settling_boxes(int body_count, int steps) {
    FxScene scene({400.0f, 400.0f});
    scene.set_gravity(FxVec2f{0.0f, -10.0f});

    make_static(add_box(scene, "ground", 200.0f, 1.0f, 380.0f, 1.0f, 1.0f));

    const int per_row = static_cast<int>(std::ceil(std::sqrt(static_cast<double>(body_count))));
    for (int i = 0; i < body_count; ++i) {
        const float x = 50.0f + static_cast<float>(i % per_row) * 1.5f;
        const float y = 2.0f + static_cast<float>(i / per_row) * 1.5f;
        add_box(scene, "b" + std::to_string(i), x, y, 1.0f, 1.0f, 1.0f);
    }
    return run(scene, steps, 20);
}

// Circles packed into a walled container. This is the contact-dense case the solver profile was
// taken on: every body touches several neighbours, so the velocity passes carry the step.
Result bench_pile(int body_count, int steps) {
    FxScene scene({200.0f, 200.0f});
    scene.set_gravity(FxVec2f{0.0f, -10.0f});

    const float floor_y = 5.0f;
    const float half_width = 14.0f;
    make_static(add_box(scene, "floor", 100.0f, floor_y, 2.0f * half_width, 1.0f, 1.0f));
    make_static(add_box(scene, "wall_l", 100.0f - half_width, floor_y + 15.0f, 1.0f, 30.0f, 1.0f));
    make_static(add_box(scene, "wall_r", 100.0f + half_width, floor_y + 15.0f, 1.0f, 30.0f, 1.0f));

    // Staggered rows so the pack settles into a dense pile rather than a lattice that never
    // rearranges. Radius 0.5 against a 26-wide floor gives roughly 24 per row.
    const int per_row = 24;
    for (int i = 0; i < body_count; ++i) {
        const int row = i / per_row;
        const int col = i % per_row;
        const float stagger = (row % 2 == 0) ? 0.0f : 0.5f;
        const float x = 87.5f + (static_cast<float>(col) + stagger) * 1.05f;
        const float y = floor_y + 1.5f + static_cast<float>(row) * 1.05f;
        add_circle(scene, "p" + std::to_string(i), x, y, 0.5f, 1.0f);
    }
    // Longer warm-up: the pile has to actually pack before the contact count is representative.
    return run(scene, steps, 120);
}

// Free-standing columns of boxes. Deep contact chains with few contacts per body -- the case
// where solver convergence, not contact volume, decides the cost.
Result bench_stacks(int body_count, int steps) {
    FxScene scene({400.0f, 200.0f});
    scene.set_gravity(FxVec2f{0.0f, -10.0f});

    make_static(add_box(scene, "ground", 200.0f, 1.0f, 380.0f, 1.0f, 1.0f));

    const int column_height = 10;
    const int columns = (body_count + column_height - 1) / column_height;
    int made = 0;
    for (int c = 0; c < columns && made < body_count; ++c) {
        const float x = 30.0f + static_cast<float>(c) * 3.0f;
        for (int r = 0; r < column_height && made < body_count; ++r, ++made) {
            add_box(scene, "s" + std::to_string(made), x, 2.0f + static_cast<float>(r) * 1.01f,
                    1.0f, 1.0f, 1.0f);
        }
    }
    return run(scene, steps, 60);
}

struct BenchScene {
    const char* name;
    Result (*fn)(int, int);
    std::vector<int> sizes;
};

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

    const std::vector<BenchScene> scenes = {
        {"settling_boxes", &bench_settling_boxes, {10, 50, 200, 400, 800, 1600, 3000}},
        {"pile", &bench_pile, {50, 150, 300, 600, 1200}},
        {"stacks", &bench_stacks, {50, 200, 600}},
    };

    std::cout << std::fixed << std::setprecision(3);
    std::cout << "Fx2D step benchmark - " << steps << " steps per size\n";
    std::cout << "cpu/wall near 1.0 = serial. Above 1.0 is only a win if wall time fell too.\n";
    if (FxProfile::enabled()) std::cout << "profiler: ON\n";

    for (const BenchScene& scene : scenes) {
        if (!filter.empty() && filter != scene.name) continue;
        std::cout << "\n== " << scene.name << " ==\n";
        std::cout << "  bodies   wall/step    cpu/step   cpu/wall     steps/s    contacts\n";
        for (int n : scene.sizes) {
            const Result r = scene.fn(n, steps);
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
