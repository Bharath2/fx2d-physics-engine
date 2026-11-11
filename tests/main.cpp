// Test suite entry point. Every suite runs even if an earlier one throws, so one run reports
// every broken area. Suites marked slow are skipped when FX2D_SKIP_SLOW_TESTS is set, and suites
// that load YAML scenes are skipped in a build without the YAML loader (FX2D_YAML=OFF).

#include <cstdlib>
#include <exception>
#include <iostream>
#include <iterator>
#include <string>

void run_aabb_tree_tests();
void run_ccd_tests();
void run_capsule_tests();
void run_edge_tests();
void run_angle_precision_tests();
void run_resting_stability_tests();
void run_contact_event_tests();
void run_yaml_inertia_tests();
void run_group_tests();
void run_chain_tests();
void run_query_tests();
void run_input_tests();
void run_slingshot_tests();
void run_adversarial_tests();
void run_joint_tests();
void run_mouse_joint_tests();
void run_solver_regression_tests();
void run_contact_graph_tests();
void run_distance_joint_tests();

namespace {

struct Suite {
    const char* name;
    void (*run)(); // null when the suite is not built
    bool slow;
};

// Suites that load YAML scenes; without the YAML loader their files are not built.
#ifdef FX2D_NO_YAML
#define FX2D_YAML_SUITE(run) nullptr
#else
#define FX2D_YAML_SUITE(run) run
#endif

const Suite kSuites[] = {
    {"aabb_tree", run_aabb_tree_tests, false},
    {"ccd", run_ccd_tests, false},
    {"capsule", FX2D_YAML_SUITE(run_capsule_tests), false},
    {"edge", FX2D_YAML_SUITE(run_edge_tests), false},
    {"angle_precision", run_angle_precision_tests, false},
    {"resting_stability", run_resting_stability_tests, false},
    {"contact_events", run_contact_event_tests, false},
    {"yaml_inertia", FX2D_YAML_SUITE(run_yaml_inertia_tests), false},
    {"groups", run_group_tests, false},
    {"chain", FX2D_YAML_SUITE(run_chain_tests), false},
    {"queries", run_query_tests, false},
    {"input", run_input_tests, false},
    {"slingshot", run_slingshot_tests, false},
    {"adversarial", run_adversarial_tests, true},
    {"joints", FX2D_YAML_SUITE(run_joint_tests), false},
    {"mouse_joint", FX2D_YAML_SUITE(run_mouse_joint_tests), false},
    {"contact_graph", run_contact_graph_tests, false},
    {"solver_regression", run_solver_regression_tests, false},
    {"distance_joint", FX2D_YAML_SUITE(run_distance_joint_tests), false},
};

bool skip_slow_requested() {
    const char* value = std::getenv("FX2D_SKIP_SLOW_TESTS");
    return value != nullptr && std::string(value) != "0" && *value != '\0';
}

} // namespace

int main() {
    const bool skip_slow = skip_slow_requested();
    int failures = 0;
    int ran = 0;
    int skipped = 0;

    for (const Suite& suite : kSuites) {
        if (suite.run == nullptr) {
            std::cout << "[SKIP] " << suite.name << " (built without the YAML loader)" << std::endl;
            ++skipped;
            continue;
        }
        if (suite.slow && skip_slow) {
            // Announced, never silent: a skip must not read as a pass.
            std::cout << "[SKIP] " << suite.name << " (FX2D_SKIP_SLOW_TESTS is set)" << std::endl;
            ++skipped;
            continue;
        }
        ++ran;
        try {
            suite.run();
        } catch (const std::exception& error) {
            std::cerr << "[FAIL] " << suite.name << ": " << error.what() << std::endl;
            ++failures;
        } catch (...) {
            std::cerr << "[FAIL] " << suite.name << ": unknown exception" << std::endl;
            ++failures;
        }
    }

    if (failures > 0) {
        std::cerr << failures << " of " << ran << " suites failed." << std::endl;
        return 1;
    }

    std::cout << "All " << ran << " suites passed";
    if (skipped > 0) std::cout << " (" << skipped << " suite(s) skipped)";
    std::cout << "." << std::endl;
    return 0;
}
