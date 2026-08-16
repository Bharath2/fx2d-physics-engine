// Entity groups: identity plus bulk operations over a set of entities, with intra-group
// collision filtering through one integer per body instead of O(N^2) pair exclusions.

#include "Fx2D/Scene.h"

#include "test_harness.h"
#include "test_scene_builders.h"

#include <cmath>
#include <iostream>
#include <memory>
#include <string>

namespace {

constexpr double kFrame = 1.0 / 60.0;

// Two overlapping boxes, so any contact between them is unambiguous.
void spawn_overlapping_pair(FxScene& scene, const std::shared_ptr<FxEntityGroup>& group) {
    auto a = std::make_shared<FxEntity>("member_a");
    a->set_visual_geometry(FxVisualShape(FxVec2f{1.0f, 1.0f}));
    a->set_collision_geometry(FxCollisionShape(FxVec2f{1.0f, 1.0f}));
    a->set_init_pose(FxVec3f{10.0f, 10.0f, 0.0f});
    a->set_mass(1.0f);
    a->set_inertia();
    scene.add_to_group(group, a);
    a->reset();

    auto b = std::make_shared<FxEntity>("member_b");
    b->set_visual_geometry(FxVisualShape(FxVec2f{1.0f, 1.0f}));
    b->set_collision_geometry(FxCollisionShape(FxVec2f{1.0f, 1.0f}));
    b->set_init_pose(FxVec3f{10.6f, 10.0f, 0.0f});
    b->set_mass(1.0f);
    b->set_inertia();
    scene.add_to_group(group, b);
    b->reset();
}

bool pair_in_contacts(const FxScene& scene, const std::string& a, const std::string& b) {
    for (const auto& c : scene.contacts()) {
        if (!c.entity1 || !c.entity2) continue;
        const std::string& n1 = c.entity1->get_name();
        const std::string& n2 = c.entity2->get_name();
        if ((n1 == a && n2 == b) || (n1 == b && n2 == a)) return true;
    }
    return false;
}

void test_group_creation_and_lookup() {
    FxScene scene = make_scene(FxVec2f{20.0f, 20.0f}, 0.0f);
    auto group = scene.create_group("rig");
    require(group != nullptr, "creating a group with a fresh name must succeed");
    require(scene.create_group("rig") == nullptr, "a taken name must be refused");
    require(scene.get_group("rig") == group, "the group must be retrievable by name");
    require(scene.group_count() == 1, "one group must be registered");

    spawn_overlapping_pair(scene, group);
    require(group->size() == 2, "both members must be in the group");
    require(scene.entity_count() == 2, "members must also be scene entities");
}

// The default: members never collide with one another, outsiders are unaffected.
void test_members_do_not_collide_with_each_other() {
    FxScene scene = make_scene(FxVec2f{20.0f, 20.0f}, 0.0f);
    auto group = scene.create_group("rig");
    spawn_overlapping_pair(scene, group);

    // An outsider overlapping member_a, to prove filtering is scoped to the group.
    add_box(scene, "outsider", {9.4f, 10.0f}, {1.0f, 1.0f});

    scene.step(kFrame);

    require(!pair_in_contacts(scene, "member_a", "member_b"),
            "overlapping members of a group must not contact each other");
    require(pair_in_contacts(scene, "member_a", "outsider"),
            "a non-member must still contact a member it overlaps");
}

void test_self_collide_group_members_do_collide() {
    FxScene scene = make_scene(FxVec2f{20.0f, 20.0f}, 0.0f);
    auto group = scene.create_group("pile", /*self_collide=*/true);
    spawn_overlapping_pair(scene, group);

    scene.step(kFrame);
    require(pair_in_contacts(scene, "member_a", "member_b"),
            "a self-colliding group must leave member pairs alone");
}

void test_delete_group_removes_members() {
    FxScene scene = make_scene(FxVec2f{20.0f, 20.0f}, 0.0f);
    auto group = scene.create_group("rig");
    spawn_overlapping_pair(scene, group);

    require(scene.delete_group("rig"), "deleting an existing group must succeed");
    require(scene.group_count() == 0, "the group must be gone");
    require(!scene.entity_exists("member_a") && !scene.entity_exists("member_b"),
            "deleting a group must delete its members");
    require(!scene.delete_group("rig"), "deleting a missing group must report failure");
}

void test_disable_group_removes_members_from_simulation() {
    FxScene scene = make_scene(FxVec2f{20.0f, 20.0f});
    make_static(add_box(scene, "ground", {10.0f, 1.0f}, {18.0f, 1.0f}));

    auto group = scene.create_group("rig", true);
    auto faller = std::make_shared<FxEntity>("faller");
    faller->set_visual_geometry(FxVisualShape(FxVec2f{1.0f, 1.0f}));
    faller->set_collision_geometry(FxCollisionShape(FxVec2f{1.0f, 1.0f}));
    faller->set_init_pose(FxVec3f{10.0f, 8.0f, 0.0f});
    faller->set_mass(1.0f);
    faller->set_inertia();
    scene.add_to_group(group, faller);
    faller->reset();

    group->set_enabled(false);
    for (int i = 0; i < 30; ++i)
        scene.step(kFrame);
    require_near(faller->pose.y(), 8.0f, 1e-4f, "a disabled member must not move");

    group->set_enabled(true);
    for (int i = 0; i < 30; ++i)
        scene.step(kFrame);
    require(faller->pose.y() < 7.5f, "a re-enabled member must fall again");
}

// A reset restores groups with the membership they had at capture.
void test_reset_restores_groups() {
    FxScene scene = make_scene(FxVec2f{20.0f, 20.0f}, 0.0f);
    auto group = scene.create_group("rig");
    spawn_overlapping_pair(scene, group);

    scene.step(kFrame); // captures the composition

    require(scene.delete_group("rig"), "the group must delete mid-run");
    scene.step(kFrame);
    require(scene.group_count() == 0 && !scene.entity_exists("member_a"),
            "group and members must be gone before the reset");

    scene.reset();

    auto restored = scene.get_group("rig");
    require(restored != nullptr, "reset must bring the group back");
    require(restored->size() == 2, "with its captured membership");
    require(scene.entity_exists("member_a") && scene.entity_exists("member_b"),
            "and its member entities");

    scene.step(kFrame);
    require(!pair_in_contacts(scene, "member_a", "member_b"),
            "restored members must still not collide with each other");
}

void test_unique_name_generation() {
    FxScene scene = make_scene(FxVec2f{20.0f, 20.0f}, 0.0f);
    add_box(scene, "link", {5.0f, 5.0f}, {1.0f, 1.0f});
    add_box(scene, "link_1", {8.0f, 5.0f}, {1.0f, 1.0f});

    // Generated names skip past taken ones rather than colliding.
    require(scene.unique_entity_name("link") == "link_2",
            "the generator must skip link and link_1");
    require(scene.unique_entity_name("fresh") == "fresh",
            "an untaken base must come back unchanged");
}

} // namespace

void run_group_tests() {
    test_group_creation_and_lookup();
    test_members_do_not_collide_with_each_other();
    test_self_collide_group_members_do_collide();
    test_delete_group_removes_members();
    test_disable_group_removes_members_from_simulation();
    test_reset_restores_groups();
    test_unique_name_generation();
    std::cout << "Entity group tests passed." << std::endl;
}
