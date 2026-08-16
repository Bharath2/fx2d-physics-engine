// Inertia derived from YAML must match the same shape built in C++. Regression: the implicit
// set_inertia() once ran before the visual shape was attached, so every entity got the default
// 0.5-circle inertia of 0.125 * mass whatever its geometry.

#include "Fx2D/Physics.h"
#include "Fx2D/YamlUtils.h"

#include "test_harness.h"

#include <cmath>
#include <iostream>
#include <memory>
#include <string>

namespace {

// Inertia of a default-constructed FxVisualShape (circle, radius 0.5): 0.5 * m * r^2.
float default_circle_inertia(float mass) {
    return 0.5f * mass * 0.5f * 0.5f;
}

std::shared_ptr<FxEntity> build(const std::string& yaml) {
    YAML::Node node = YAML::Load(yaml);
    return FxYAML::buildEntity("probe", node);
}

// A rectangle must get the rectangle's inertia, not the default circle's.
void test_rectangle_inertia_comes_from_authored_shape() {
    auto entity = build(R"YAML(
physics:
    mass: 5.0
visual:
    geometry:
        rectangle: [2.0, 1.0]
collision:
    geometry:
        rectangle: [2.0, 1.0]
)YAML");

    // Solid rectangle about its centre: I = m (w^2 + h^2) / 12
    const float expected = 5.0f * (2.0f * 2.0f + 1.0f * 1.0f) / 12.0f;

    require_near(entity->inertia(), expected, 1e-3f,
                 "a YAML rectangle must take its inertia from its own geometry");
    require(std::fabs(entity->inertia() - default_circle_inertia(5.0f)) > 1e-3f,
            "the inertia must not be the default 0.5-circle value");
    require_near(entity->inv_inertia(), 1.0f / expected, 1e-3f,
                 "inv_inertia must be the reciprocal of the computed inertia");
}

// The YAML path and the equivalent hand-built entity must agree, for every shape.
void test_yaml_matches_hand_built_for_each_shape() {
    struct Case {
        const char* name;
        const char* geometry;
        FxVisualShape shape;
    };

    const std::vector<Case> cases = {
        {"circle", "circle: 0.75", FxVisualShape(0.75f)},
        {"rectangle", "rectangle: [2.0, 1.0]", FxVisualShape(FxVec2f{2.0f, 1.0f})},
        {"capsule", "capsule: [2.0, 0.3]", FxVisualShape(2.0f, 0.3f)},
        {"rounded_rect", "rectangle: [2.0, 2.0]\n        radius: 0.25",
         FxVisualShape(FxVec2f{2.0f, 2.0f}, 0.25f)},
    };

    const float mass = 3.0f;

    for (const auto& test_case : cases) {
        std::string yaml =
            std::string("physics:\n    mass: 3.0\nvisual:\n    geometry:\n        ") +
            test_case.geometry + "\n";
        auto from_yaml = build(yaml);

        auto hand_built = std::make_shared<FxEntity>("hand_built");
        hand_built->set_visual_geometry(test_case.shape);
        hand_built->set_mass(mass);
        hand_built->set_inertia();

        require_near(from_yaml->inertia(), hand_built->inertia(), 1e-4f,
                     std::string("YAML and hand-built inertia must agree for ") + test_case.name);
        require(from_yaml->inertia() > 0.0f,
                std::string("a solid shape must have non-zero inertia: ") + test_case.name);
    }
}

// An explicit physics.inertia must still win over the computed value.
void test_explicit_inertia_wins() {
    auto entity = build(R"YAML(
physics:
    mass: 5.0
    inertia: 42.0
visual:
    geometry:
        rectangle: [2.0, 1.0]
)YAML");

    require_near(entity->inertia(), 42.0f, 1e-4f,
                 "an explicit physics.inertia must override the computed value");
    require_near(entity->inv_inertia(), 1.0f / 42.0f, 1e-4f,
                 "inv_inertia must follow the explicit inertia");
}

// Static bodies (mass 0) get zero inertia and zero inverse inertia.
void test_zero_mass_gives_zero_inertia() {
    auto entity = build(R"YAML(
physics:
    mass: 0.0
visual:
    geometry:
        rectangle: [8.0, 0.5]
)YAML");

    require(entity->inertia() == 0.0f, "a zero-mass body must have zero inertia");
    require(entity->inv_inertia() == 0.0f, "a zero-mass body must have zero inverse inertia");
}

// An edge has no area, so it has no inertia even with mass.
void test_edge_has_zero_inertia() {
    auto entity = build(R"YAML(
physics:
    mass: 2.0
visual:
    geometry:
        edge: [[-2.0, 0.0], [2.0, 0.0]]
)YAML");

    require(entity->inertia() == 0.0f,
            "a zero-thickness edge has no area and must have zero inertia");
}

// external_forces_enabled works by zeroing inv_mass and inv_inertia, so it must be applied
// after the deferred inertia calculation or a static body becomes spinnable.
void test_external_forces_disabled_stays_static() {
    auto entity = build(R"YAML(
physics:
    mass: 1.0
    gravity_scale: 0.0
    external_forces_enabled: false
visual:
    geometry:
        rectangle: [12.0, 0.5]
)YAML");

    require(entity->inertia() > 0.0f,
            "the shape still has a real inertia value even with external forces disabled");
    require(entity->inv_inertia() == 0.0f,
            "external_forces_enabled: false must leave inverse inertia at zero, or a static "
            "body can be rotated by contacts");
    require(entity->inv_mass() == 0.0f,
            "external_forces_enabled: false must leave inverse mass at zero");
}

// The same, with an explicit inertia rather than a computed one.
void test_external_forces_disabled_with_explicit_inertia() {
    auto entity = build(R"YAML(
physics:
    mass: 1.0
    inertia: 7.0
    external_forces_enabled: false
visual:
    geometry:
        rectangle: [2.0, 1.0]
)YAML");

    require_near(entity->inertia(), 7.0f, 1e-4f, "the explicit inertia must be preserved");
    require(entity->inv_inertia() == 0.0f,
            "external_forces_enabled: false must win over an explicit inertia too");
}

// Enabling external forces explicitly must leave a normal dynamic body movable.
void test_external_forces_enabled_keeps_body_dynamic() {
    auto entity = build(R"YAML(
physics:
    mass: 2.0
    external_forces_enabled: true
visual:
    geometry:
        rectangle: [2.0, 1.0]
)YAML");

    require(entity->inv_inertia() > 0.0f, "a dynamic body must keep a non-zero inverse inertia");
    require_near(entity->inv_inertia(), 1.0f / entity->inertia(), 1e-4f,
                 "inverse inertia must match the computed inertia");
}

// Inertia scales with the square of size: doubling a box's extents quadruples it.
void test_inertia_scales_with_geometry() {
    auto small = build(R"YAML(
physics:
    mass: 1.0
visual:
    geometry:
        rectangle: [1.0, 1.0]
)YAML");
    auto large = build(R"YAML(
physics:
    mass: 1.0
visual:
    geometry:
        rectangle: [2.0, 2.0]
)YAML");

    require_near(large->inertia(), 4.0f * small->inertia(), 1e-4f,
                 "doubling both extents at equal mass must quadruple the inertia");
}

} // namespace

void run_yaml_inertia_tests() {
    test_rectangle_inertia_comes_from_authored_shape();
    test_yaml_matches_hand_built_for_each_shape();
    test_explicit_inertia_wins();
    test_zero_mass_gives_zero_inertia();
    test_edge_has_zero_inertia();
    test_external_forces_disabled_stays_static();
    test_external_forces_disabled_with_explicit_inertia();
    test_external_forces_enabled_keeps_body_dynamic();
    test_inertia_scales_with_geometry();
    std::cout << "YAML inertia tests passed." << std::endl;
}
