// Entry point for the Fx2D test suite.
//
// Every suite runs even if an earlier one fails, so a single run reports all
// broken areas rather than only the first. A failing suite throws; main turns
// that into a non-zero exit status for ctest and CI.

#include <exception>
#include <iostream>
#include <iterator>

void run_aabb_tree_tests();
void run_ccd_tests();
void run_capsule_tests();
void run_edge_tests();
void run_angle_precision_tests();
void run_resting_stability_tests();
void run_joint_tests();

namespace {

struct Suite {
    const char* name;
    void (*run)();
};

const Suite kSuites[] = {
    {"aabb_tree", run_aabb_tree_tests},
    {"ccd", run_ccd_tests},
    {"capsule", run_capsule_tests},
    {"edge", run_edge_tests},
    {"angle_precision", run_angle_precision_tests},
    {"resting_stability", run_resting_stability_tests},
    {"joints", run_joint_tests},
};

} // namespace

int main() {
    int failures = 0;

    for (const Suite& suite : kSuites) {
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
        std::cerr << failures << " of " << std::size(kSuites) << " suites failed." << std::endl;
        return 1;
    }

    std::cout << "All " << std::size(kSuites) << " suites passed." << std::endl;
    return 0;
}
