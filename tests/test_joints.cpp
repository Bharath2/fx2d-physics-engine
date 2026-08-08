#include "Fx2D/Joints.h"
#include "Fx2D/Scene.h"
#include "Fx2D/YamlUtils.h"

#include <cmath>
#include <iostream>
#include <memory>
#include <stdexcept>

namespace {

bool approx_equal(float lhs, float rhs, float eps = 1e-4f) {
    return std::fabs(lhs - rhs) <= eps;
}

void require(bool condition, const char* message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

std::shared_ptr<FxEntity> make_entity(const std::string& name) {
    auto entity = std::make_shared<FxEntity>(name);
    entity->set_mass(1.0f);
    entity->set_inertia(1.0f);
    return entity;
}

std::shared_ptr<FxEntity> make_collision_entity(const std::string& name, float x, float y) {
    auto entity = make_entity(name);
    entity->set_init_pose(FxVec3f{x, y, 0.0f});
    entity->set_collision_geometry(FxCollisionShape(FxVec2f{1.0f, 1.0f}));
    entity->step(FxVec2f{0.0f, 0.0f}, 0.001);
    return entity;
}

void reset_entity_state(const std::shared_ptr<FxEntity>& entity) {
    entity->pose = FxVec3f{0.0f, 0.0f, 0.0f};
    entity->prev_pose = entity->pose;
    entity->velocity = FxVec3f{0.0f, 0.0f, 0.0f};
    entity->prev_velocity = entity->velocity;
}

void test_pid_round_trip() {
    auto e1 = make_entity("joint_pid_a");
    auto e2 = make_entity("joint_pid_b");
    FxRevoluteJoint joint("joint_pid", e1, e2, FxVec2f{0.0f, 0.0f});

    joint.set_pid(FxVec3f{2.0f, 3.0f, 4.0f});
    FxVec3f pid = joint.get_pid();

    require(approx_equal(pid.x(), 2.0f), "PID p round-trip failed");
    require(approx_equal(pid.y(), 3.0f), "PID i round-trip failed");
    require(approx_equal(pid.z(), 4.0f), "PID d round-trip failed");
}

void test_effort_aliases() {
    auto e1 = make_entity("joint_alias_a");
    auto e2 = make_entity("joint_alias_b");

    FxRevoluteJoint revolute("joint_alias_r", e1, e2, FxVec2f{0.0f, 0.0f});
    revolute.set_torque(5.0f);
    require(approx_equal(revolute.get_effort(), 5.0f), "Torque alias did not store effort");

    FxPrismaticJoint prismatic("joint_alias_p", e1, e2, FxVec2f{1.0f, 0.0f});
    prismatic.set_force(6.0f);
    require(approx_equal(prismatic.get_effort(), 6.0f), "Force alias did not store effort");
}

void test_effort_mode_applies_direct_effort() {
    {
        auto e1 = make_entity("joint_effort_r_a");
        auto e2 = make_entity("joint_effort_r_b");
        FxRevoluteJoint joint("joint_effort_r", e1, e2, FxVec2f{0.0f, 0.0f});

        joint.set_pid(FxVec3f{100.0f, 100.0f, 100.0f});
        joint.set_control_mode(ControlMode::EFFORT);
        joint.set_torque(2.0f);
        joint.apply_controls(0.25);
        e1->step(FxVec2f{0.0f, 0.0f}, 1.0);
        e2->step(FxVec2f{0.0f, 0.0f}, 1.0);

        require(approx_equal(e1->velocity.theta(), -2.0f),
                "Revolute effort mode did not apply torque to entity1");
        require(approx_equal(e2->velocity.theta(), 2.0f),
                "Revolute effort mode did not apply torque to entity2");
    }

    {
        auto e1 = make_entity("joint_effort_p_a");
        auto e2 = make_entity("joint_effort_p_b");
        FxPrismaticJoint joint("joint_effort_p", e1, e2, FxVec2f{1.0f, 0.0f});

        joint.set_pid(FxVec3f{100.0f, 100.0f, 100.0f});
        joint.set_control_mode(ControlMode::EFFORT);
        joint.set_force(3.0f);
        joint.apply_controls(0.25);
        e1->step(FxVec2f{0.0f, 0.0f}, 1.0);
        e2->step(FxVec2f{0.0f, 0.0f}, 1.0);

        require(approx_equal(e1->velocity.x(), -3.0f),
                "Prismatic effort mode did not apply force to entity1");
        require(approx_equal(e2->velocity.x(), 3.0f),
                "Prismatic effort mode did not apply force to entity2");
    }
}

void test_pid_state_resets_on_target_and_mode_changes() {
    auto e1 = make_entity("joint_reset_a");
    auto e2 = make_entity("joint_reset_b");
    FxRevoluteJoint joint("joint_reset", e1, e2, FxVec2f{0.0f, 0.0f});

    joint.set_pid(FxVec3f{0.0f, 1.0f, 0.0f});
    joint.set_control_mode(ControlMode::POSITION);
    joint.set_theta(1.0f, false);
    joint.apply_controls(1.0);
    e1->step(FxVec2f{0.0f, 0.0f}, 1.0);
    e2->step(FxVec2f{0.0f, 0.0f}, 1.0);

    reset_entity_state(e1);
    reset_entity_state(e2);

    joint.set_theta(0.0f, false);
    joint.apply_controls(1.0);
    e1->step(FxVec2f{0.0f, 0.0f}, 1.0);
    e2->step(FxVec2f{0.0f, 0.0f}, 1.0);

    require(approx_equal(e1->velocity.theta(), 0.0f),
            "PID state was not reset for entity1 on target change");
    require(approx_equal(e2->velocity.theta(), 0.0f),
            "PID state was not reset for entity2 on target change");

    reset_entity_state(e1);
    reset_entity_state(e2);

    joint.set_theta(1.0f, false);
    joint.apply_controls(1.0);
    e1->step(FxVec2f{0.0f, 0.0f}, 1.0);
    e2->step(FxVec2f{0.0f, 0.0f}, 1.0);

    reset_entity_state(e1);
    reset_entity_state(e2);

    joint.set_control_mode(ControlMode::EFFORT);
    joint.set_effort(0.0f);
    joint.apply_controls(1.0);
    e1->step(FxVec2f{0.0f, 0.0f}, 1.0);
    e2->step(FxVec2f{0.0f, 0.0f}, 1.0);

    require(approx_equal(e1->velocity.theta(), 0.0f),
            "PID state was not reset for entity1 on mode change");
    require(approx_equal(e2->velocity.theta(), 0.0f),
            "PID state was not reset for entity2 on mode change");
}

void test_yaml_joint_build_supports_effort_mode_and_aliases() {
    FxScene scene = FxYAML::buildScene(R"yaml(
scene:
  size: [100, 100]
entities:
  base:
    pose: [0, 0, 0]
    physics:
      mass: 1
      inertia: 1
  link:
    pose: [1, 0, 0]
    physics:
      mass: 1
      inertia: 1
joints:
  revolute_effort:
    type: revolute
    parent: base
    child: link
    control_mode: effort
    target: 1.5
    max_effort: 4.0
    pid: [2.0, 3.0, 4.0]
  prismatic_alias:
    type: prismatic
    parent: base
    child: link
    axis: [1, 0]
    control_mode: effort
    target: 2.5
    max_force: 6.0
  revolute_alias:
    type: revolute
    parent: base
    child: link
    control_mode: effort
    target: 3.5
    max_torque: 7.0
)yaml");

    auto revolute_effort =
        std::dynamic_pointer_cast<FxRevoluteJoint>(scene.get_joint("revolute_effort"));
    auto prismatic_alias =
        std::dynamic_pointer_cast<FxPrismaticJoint>(scene.get_joint("prismatic_alias"));
    auto revolute_alias =
        std::dynamic_pointer_cast<FxRevoluteJoint>(scene.get_joint("revolute_alias"));

    require(static_cast<bool>(revolute_effort), "Failed to build revolute_effort joint");
    require(static_cast<bool>(prismatic_alias), "Failed to build prismatic_alias joint");
    require(static_cast<bool>(revolute_alias), "Failed to build revolute_alias joint");

    require(revolute_effort->get_control_mode() == ControlMode::EFFORT,
            "YAML did not set revolute effort mode");
    require(approx_equal(revolute_effort->get_effort(), 1.5f),
            "YAML did not set revolute effort target");
    require(approx_equal(revolute_effort->get_max_effort(), 4.0f),
            "YAML did not set unified max_effort");
    FxVec3f pid = revolute_effort->get_pid();
    require(approx_equal(pid.x(), 2.0f), "YAML did not set pid p");
    require(approx_equal(pid.y(), 3.0f), "YAML did not set pid i");
    require(approx_equal(pid.z(), 4.0f), "YAML did not set pid d");

    require(prismatic_alias->get_control_mode() == ControlMode::EFFORT,
            "YAML did not set prismatic effort mode");
    require(approx_equal(prismatic_alias->get_effort(), 2.5f),
            "YAML did not set prismatic effort target");
    require(approx_equal(prismatic_alias->get_max_effort(), 6.0f),
            "YAML max_force alias did not map to max_effort");

    require(revolute_alias->get_control_mode() == ControlMode::EFFORT,
            "YAML did not set revolute alias effort mode");
    require(approx_equal(revolute_alias->get_effort(), 3.5f),
            "YAML did not set revolute alias effort target");
    require(approx_equal(revolute_alias->get_max_effort(), 7.0f),
            "YAML max_torque alias did not map to max_effort");
}

void test_entity_registry_remove_keeps_broad_phase_consistent() {
    FxEntityRegistry registry(16);
    auto a = make_collision_entity("registry_a", 0.0f, 0.0f);
    auto b = make_collision_entity("registry_b", 0.4f, 0.0f);
    auto c = make_collision_entity("registry_c", 3.0f, 0.0f);

    require(registry.add(a), "Failed to add registry_a");
    require(registry.add(b), "Failed to add registry_b");
    require(registry.add(c), "Failed to add registry_c");

    auto pairs = registry.get_broad_phase_pairs();
    require(pairs.size() == 1, "Expected one initial broad-phase pair");
    require(registry.items()[pairs[0].first]->get_name() == "registry_a",
            "Unexpected first initial pair name");
    require(registry.items()[pairs[0].second]->get_name() == "registry_b",
            "Unexpected second initial pair name");

    require(registry.remove("registry_a"), "Failed to remove registry_a");

    auto d = make_collision_entity("registry_d", 3.4f, 0.0f);
    require(registry.add(d), "Failed to add registry_d");

    pairs = registry.get_broad_phase_pairs();
    require(pairs.size() == 1, "Expected one post-remove broad-phase pair");
    require(registry.items()[pairs[0].first]->get_name() == "registry_c",
            "Moved entity index was not updated");
    require(registry.items()[pairs[0].second]->get_name() == "registry_d",
            "New entity pair was not discovered");
}

} // namespace

void run_aabb_tree_tests(); // defined in test_aabb_tree.cpp
void run_ccd_tests(); // defined in test_ccd.cpp
void run_capsule_tests(); // defined in test_capsule_collision.cpp
void run_edge_tests(); // defined in test_collisions_edge.cpp
void run_angle_precision_tests(); // defined in test_angle_precision.cpp
void run_resting_stability_tests(); // defined in test_resting_stability.cpp

int main() {
    run_aabb_tree_tests();
    run_ccd_tests();
    run_capsule_tests();
    run_edge_tests();
    run_angle_precision_tests();
    run_resting_stability_tests();
    test_pid_round_trip();
    test_effort_aliases();
    test_effort_mode_applies_direct_effort();
    test_pid_state_resets_on_target_and_mode_changes();
    test_yaml_joint_build_supports_effort_mode_and_aliases();
    test_entity_registry_remove_keeps_broad_phase_consistent();

    std::cout << "Joint tests passed." << std::endl;
    return 0;
}
