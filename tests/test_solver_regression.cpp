// Solver regression net: guards refactors meant to change speed and nothing else. Other suites
// assert physical properties, which stay true under a solver that quietly drifts; this one pins
// the numbers. See docs/CONTRIBUTING.md for how to re-baseline and when that is legitimate.

// Two independent nets. Repeatability: the same scene stepped twice in one process must be
// bit-identical, exact equality on purpose, because it catches iteration order leaking into
// results -- which every tolerance-based test would pass.

// Golden state: short non-chaotic scenes against values from a known-good build, with tight but
// not bitwise tolerances, since a different compiler or ISA moves the last bits. Re-baseline by
// running with FX2D_PRINT_GOLDENS=1, and only when the physics change was deliberate.

#include "Fx2D/Scene.h"
#include "test_harness.h"
#include "test_scene_builders.h"

#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

namespace {

bool print_goldens_requested() {
    const char* value = std::getenv("FX2D_PRINT_GOLDENS");
    return value != nullptr && std::string(value) != "0" && *value != '\0';
}

// A flat snapshot of everything the solver is allowed to determine. Comparing whole vectors
// rather than hand-picked scalars means a refactor cannot move a body the test forgot to name.
struct Snapshot {
    std::vector<float> values; // x, y, theta, vx, vy, w per entity, in registry order
};

// Registry order, which is insertion order for a scene nothing was removed from, so the
// snapshot lines up value-for-value with the golden table.
Snapshot capture(FxScene& scene) {
    Snapshot snap;
    snap.values.reserve(scene.entity_count() * 6);
    scene.for_each_entity(std::execution::seq, [&](const auto& e) {
        snap.values.push_back(e->pose.x());
        snap.values.push_back(e->pose.y());
        snap.values.push_back(e->pose.theta());
        snap.values.push_back(e->velocity.x());
        snap.values.push_back(e->velocity.y());
        snap.values.push_back(e->velocity.theta());
    });
    return snap;
}

void run_steps(FxScene& scene, int steps) {
    for (int i = 0; i < steps; ++i)
        scene.step(1.0 / 60.0);
}

// --- Scenes -------------------------------------------------------------------------------
// Every scene is built by a function so the repeatability net can construct it twice from
// scratch, with no state carried between the two runs.

// A single box dropped onto static ground. The simplest thing that exercises integration,
// one contact, penetration resolution and the velocity passes together.
void build_drop(FxScene& scene) {
    make_static(add_box(scene, "ground", {10.0f, 1.0f}, {18.0f, 1.0f}));
    add_box(scene, "box", {10.0f, 6.0f}, {1.0f, 1.0f}, FxBody{1.0f, 0.0f, 0.5f, 0.4f});
}

// A five-box column. Contact chains are where solver convergence shows up, and where a
// weakened velocity solve (the risk in the colored Jacobi phase) will show first.
void build_stack(FxScene& scene) {
    make_static(add_box(scene, "ground", {10.0f, 1.0f}, {18.0f, 1.0f}));
    for (int i = 0; i < 5; ++i) {
        add_box(scene, "s" + std::to_string(i), {10.0f, 2.0f + static_cast<float>(i) * 1.01f},
                {1.0f, 1.0f}, FxBody{1.0f, 0.0f, 0.6f, 0.5f});
    }
}

// A bouncing ball. Restitution is fixed from the closing speed captured at substep start, so
// this is the scene that catches a broken restitution target or a lost warm-start.
void build_bounce(FxScene& scene) {
    make_static(add_box(scene, "ground", {10.0f, 1.0f}, {18.0f, 1.0f}));
    add_circle(scene, "ball", {10.0f, 5.0f}, 0.5f, FxBody{1.0f, 0.6f, 0.0f, 0.0f});
}

// A small pile of circles between two walls. Many simultaneous contacts per body, which is
// what the contact cache, the warm start and the friction cone all act on.
void build_pile(FxScene& scene) {
    make_static(add_box(scene, "floor", {10.0f, 1.0f}, {12.0f, 1.0f}));
    make_static(add_box(scene, "wall_l", {4.5f, 5.0f}, {1.0f, 8.0f}));
    make_static(add_box(scene, "wall_r", {15.5f, 5.0f}, {1.0f, 8.0f}));
    for (int i = 0; i < 20; ++i) {
        const int row = i / 5;
        const int col = i % 5;
        const float stagger = (row % 2 == 0) ? 0.0f : 0.4f;
        add_circle(scene, "p" + std::to_string(i),
                   {7.5f + (static_cast<float>(col) + stagger) * 1.05f,
                    2.2f + static_cast<float>(row) * 1.05f},
                   0.5f, FxBody{1.0f, 0.1f, 0.4f, 0.3f});
    }
}

struct RegressionScene {
    const char* name;
    void (*build)(FxScene&);
    int steps;
    float tolerance; // absolute, on every captured value
};

const RegressionScene kScenes[] = {
    {"drop", build_drop, 90, 1e-4f},
    {"stack", build_stack, 150, 1e-3f},
    {"bounce", build_bounce, 120, 1e-3f},
    // The pile rearranges, so it amplifies differences faster than the others. A looser
    // tolerance keeps it a useful net without making it a tripwire for the last bit.
    {"pile", build_pile, 90, 5e-3f},
};

// --- Goldens ------------------------------------------------------------------------------
// Captured on x86-64 GCC 13, Release, -ffp-contract=off, at the 14x4 substep/pass default.
// Regenerate with FX2D_PRINT_GOLDENS=1 -- and only when the physics change was intended.

struct Golden {
    const char* scene;
    std::vector<float> values;
};

const std::vector<Golden> kGoldens = {
#include "goldens_solver_regression.inc"
};

const Golden* find_golden(const char* scene) {
    for (const auto& g : kGoldens) {
        if (std::string(g.scene) == scene) return &g;
    }
    return nullptr;
}

void print_golden_table(const char* scene, const Snapshot& snap) {
    std::cout << "    {\"" << scene << "\", {";
    // showpoint keeps a decimal point on whole numbers, so "10" prints as "10.0000000" and the
    // pasted literal is "10.0000000f" rather than the ill-formed "10f".
    std::cout << std::setprecision(9) << std::showpoint;
    for (std::size_t i = 0; i < snap.values.size(); ++i) {
        if (i % 6 == 0) std::cout << "\n     ";
        std::cout << snap.values[i] << "f";
        if (i + 1 < snap.values.size()) std::cout << ", ";
    }
    std::cout << "}},\n";
}

// --- The two nets -------------------------------------------------------------------------

void check_repeatable(const RegressionScene& rs) {
    FxScene first = make_scene({20.0f, 20.0f});
    rs.build(first);
    run_steps(first, rs.steps);
    const Snapshot a = capture(first);

    FxScene second = make_scene({20.0f, 20.0f});
    rs.build(second);
    run_steps(second, rs.steps);
    const Snapshot b = capture(second);

    require(a.values.size() == b.values.size(),
            std::string("regression/") + rs.name + ": snapshot sizes differ between runs");
    for (std::size_t i = 0; i < a.values.size(); ++i) {
        if (a.values[i] != b.values[i]) {
            std::ostringstream msg;
            msg << "regression/" << rs.name << ": run-to-run mismatch at value " << i << " ("
                << std::setprecision(9) << a.values[i] << " vs " << b.values[i]
                << "). The solver is not reproducible within one process.";
            throw std::runtime_error(msg.str());
        }
    }
}

void check_golden(const RegressionScene& rs, bool print_mode) {
    FxScene scene = make_scene({20.0f, 20.0f});
    rs.build(scene);
    run_steps(scene, rs.steps);
    const Snapshot snap = capture(scene);

    if (print_mode) {
        print_golden_table(rs.name, snap);
        return;
    }

    const Golden* golden = find_golden(rs.name);
    require(golden != nullptr, std::string("regression/") + rs.name + ": no golden recorded");
    require(golden->values.size() == snap.values.size(),
            std::string("regression/") + rs.name + ": golden has " +
                std::to_string(golden->values.size()) + " values, scene produced " +
                std::to_string(snap.values.size()) + " -- the scene changed, not the solver");

    for (std::size_t i = 0; i < snap.values.size(); ++i) {
        const std::size_t entity = i / 6;
        static const char* kField[6] = {"x", "y", "theta", "vx", "vy", "w"};
        require_near(snap.values[i], golden->values[i], rs.tolerance,
                     std::string("regression/") + rs.name + ": entity " + std::to_string(entity) +
                         " " + kField[i % 6] + " drifted from golden");
    }
}

} // namespace

void run_solver_regression_tests() {
    const bool print_mode = print_goldens_requested();
    if (print_mode) {
        std::cout << "// Regenerated goldens -- paste into "
                     "tests/goldens_solver_regression.inc\n";
    }

    for (const RegressionScene& rs : kScenes) {
        if (!print_mode) check_repeatable(rs);
        check_golden(rs, print_mode);
    }

    if (!print_mode) std::cout << "Solver regression tests passed." << std::endl;
}
