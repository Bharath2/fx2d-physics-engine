// Entry point for the Fx2D test suite.
//
// Every suite runs even if an earlier one fails, so a single run reports all
// broken areas rather than only the first. A failing suite throws; main turns
// that into a non-zero exit status for ctest and CI.
//
// Suites marked slow simulate many bodies for many steps. They are fast enough in
// Release but cost minutes under ASan/UBSan, and they exercise no code path the other
// suites miss, so setting FX2D_SKIP_SLOW_TESTS=1 skips them. CI uses that for its Debug
// job only; the Release job always runs everything.

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
void run_input_tests();
void run_adversarial_tests();
void run_joint_tests();

namespace {

struct Suite {
    const char* name;
    void (*run)();
    bool slow;
};

const Suite kSuites[] = {
    {"aabb_tree", run_aabb_tree_tests, false},
    {"ccd", run_ccd_tests, false},
    {"capsule", run_capsule_tests, false},
    {"edge", run_edge_tests, false},
    {"angle_precision", run_angle_precision_tests, false},
    {"resting_stability", run_resting_stability_tests, false},
    {"contact_events", run_contact_event_tests, false},
    {"yaml_inertia", run_yaml_inertia_tests, false},
    {"input", run_input_tests, false},
    {"adversarial", run_adversarial_tests, true},
    {"joints", run_joint_tests, false},
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
        if (suite.slow && skip_slow) {
            // Announced, never silent: a skipped suite must not read as a passing one.
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
    if (skipped > 0) std::cout << " (" << skipped << " slow suite(s) skipped)";
    std::cout << "." << std::endl;
    return 0;
}
