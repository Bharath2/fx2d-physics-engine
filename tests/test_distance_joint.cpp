// The distance joint: a rigid link holds its length, a rope hangs slack until it is taut,
// compliance stretches by the amount the spring constant predicts, the motor reels a load in,
// and a chain of links hangs without drifting.

#include "Fx2D/Scene.h"
#include "Fx2D/YamlUtils.h"

#include "test_harness.h"
#include "test_scene_builders.h"

#include <cmath>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

namespace {

constexpr double kFrame = 1.0 / 60.0;

void step_for(FxScene& scene, double seconds) {
    const int n = static_cast<int>(std::lround(seconds / kFrame));
    for (int i = 0; i < n; ++i)
        scene.step(kFrame);
}

// A static ceiling anchor with one body hanging under it, joined by a distance joint.
struct Pendulum {
    FxScene scene = make_scene({20.0f, 20.0f});
    std::shared_ptr<FxEntity> anchor;
    std::shared_ptr<FxEntity> bob;
    std::shared_ptr<FxDistanceJoint> link;

    Pendulum(float drop, float min_length, float max_length, float mass = 1.0f) {
        anchor = make_static(add_box(scene, "anchor", {10.0f, 15.0f}, {0.4f, 0.4f}));
        FxBody body;
        body.mass = mass;
        bob = add_circle(scene, "bob", {10.0f, 15.0f - drop}, 0.2f, body);
        link = std::make_shared<FxDistanceJoint>("link", anchor, bob, FxVec2f{0.0f, 0.0f},
                                                 FxVec2f{0.0f, 0.0f}, min_length, max_length);
        scene.add_joint(link);
    }
};

// An unset limit means "whatever the anchors are apart right now".
void test_rest_length_from_the_scene() {
    Pendulum rig(1.5f, -1.0f, -1.0f);
    require_near(rig.link->get_min_length(), 1.5f, 1e-4f, "unset minimum takes the rest length");
    require_near(rig.link->get_max_length(), 1.5f, 1e-4f, "unset maximum takes the rest length");
    require_near(rig.link->get_length(), 1.5f, 1e-4f, "get_length reports the anchor separation");

    bool threw = false;
    try {
        FxDistanceJoint bad("bad", rig.anchor, rig.bob, FxVec2f{0.0f, 0.0f}, FxVec2f{0.0f, 0.0f},
                            2.0f, 1.0f);
    } catch (const std::invalid_argument&) {
        threw = true;
    }
    require(threw, "min_length above max_length must be rejected");
}

// A rigid link holds its length against gravity, and holds it wherever the bob swings to.
void test_rigid_link_holds_its_length() {
    Pendulum rig(1.5f, -1.0f, -1.0f);
    step_for(rig.scene, 2.0);
    require_near(rig.link->get_length(), 1.5f, 5e-3f, "a loaded rigid link must keep its length");
    require_near(rig.bob->pose.y(), 13.5f, 5e-3f, "the bob must hang directly below the anchor");

    // Push it sideways: the length is a constraint, not a vertical one.
    rig.bob->velocity.xy() = FxVec2f{4.0f, 0.0f};
    float worst = 0.0f;
    for (int i = 0; i < 120; ++i) {
        rig.scene.step(kFrame);
        worst = std::max(worst, std::fabs(rig.link->get_length() - 1.5f));
    }
    require(worst < 0.02f,
            "a swinging rigid link must keep its length, worst error " + std::to_string(worst));
    require(std::fabs(rig.bob->pose.x() - 10.0f) > 0.2f, "the bob must actually have swung");
}

// A rope is free below its maximum: the bob falls as gravity says until it is taut.
void test_rope_hangs_slack_then_catches() {
    Pendulum rig(0.4f, 0.0f, 2.0f);

    step_for(rig.scene, 0.3);
    // Free fall from rest is 0.5 g t^2, so 0.45 m at 0.3 s with g = 10.
    require_near(rig.bob->pose.y(), 14.6f - 0.45f, 0.03f, "a slack rope must not hold the bob up");
    require(rig.link->get_length() < 2.0f, "the rope must still be slack at 0.3 s");

    float worst_overshoot = 0.0f;
    for (int i = 0; i < 180; ++i) {
        rig.scene.step(kFrame);
        worst_overshoot = std::max(worst_overshoot, rig.link->get_length() - 2.0f);
    }
    require(worst_overshoot < 0.05f, "the rope must not stretch past its maximum, overshoot was " +
                                         std::to_string(worst_overshoot));
    require_near(rig.link->get_length(), 2.0f, 5e-3f, "a hanging rope must rest at its maximum");
    require_near(rig.bob->pose.y(), 13.0f, 5e-3f, "the bob must hang a rope-length below");
}

// Compliance is metres per newton: a load m on a link of compliance c stretches by m g c.
void test_compliance_stretches_by_the_predicted_amount() {
    Pendulum rigid(1.0f, -1.0f, -1.0f);
    step_for(rigid.scene, 2.0);
    const float rigid_stretch = rigid.link->get_length() - 1.0f;
    require(rigid_stretch < 5e-3f,
            "the default link must be near-rigid, stretched " + std::to_string(rigid_stretch));

    Pendulum soft(1.0f, -1.0f, -1.0f);
    soft.link->set_compliance(1e-2); // 100 N/m against a 10 N load: 0.1 m
    step_for(soft.scene, 3.0);
    const float soft_stretch = soft.link->get_length() - 1.0f;
    require_near(soft_stretch, 0.1f, 0.03f,
                 "a compliant link must stretch by load / stiffness, got " +
                     std::to_string(soft_stretch));
}

// The motor reels a hanging load in, and stops where its force balances the weight.
void test_motor_reels_a_load_in() {
    Pendulum rig(3.0f, 0.0f, 5.0f);
    rig.link->set_control_mode(ControlMode::POSITION);
    rig.link->set_pid(FxVec3f{60.0f, 0.0f, 12.0f});
    rig.link->set_max_force(80.0f);
    rig.link->set_length(1.0f, false); // no instant jump: the motor has to do the work

    step_for(rig.scene, 4.0);
    // Equilibrium is where P (target - length) cancels the weight: 1.0 + m g / P.
    require_near(rig.link->get_length(), 1.0f + 10.0f / 60.0f, 0.15f,
                 "the motor must hold the load near its target length");
    require(rig.bob->pose.y() > 13.5f, "the load must have been lifted");
}

// Reeling both limits in is the winch a rigid link wants: the load follows the shortening link.
void test_limits_reel_a_rigid_link_in() {
    Pendulum rig(2.0f, -1.0f, -1.0f);
    step_for(rig.scene, 1.0);
    require_near(rig.bob->pose.y(), 13.0f, 5e-3f, "hangs at the authored length first");

    // A winch reels at a rate. Shortening a little per frame keeps the load under the anchor;
    // see the test below for what one large jump does instead.
    for (float len = 2.0f; len > 0.5f; len -= 0.02f) {
        rig.link->set_limits(len, len);
        rig.scene.step(kFrame);
    }
    rig.link->set_limits(0.5f, 0.5f);
    step_for(rig.scene, 1.0);
    require_near(rig.link->get_length(), 0.5f, 5e-3f, "the link must hold its new length");
    require_near(rig.bob->pose.y(), 14.5f, 2e-2f, "the load must have been drawn up");
}

// A large limit change can carry the load past its anchor while preserving separation.
// Reel gradually to keep the load on the same side; see docs/guides/joints.md.
void test_a_large_instant_reel_keeps_the_length_not_the_side() {
    Pendulum rig(2.0f, -1.0f, -1.0f);
    step_for(rig.scene, 1.0);

    rig.link->set_limits(0.5f, 0.5f);
    step_for(rig.scene, 1.0);
    require_near(rig.link->get_length(), 0.5f, 5e-3f,
                 "the link holds its new length however it got there");
    require(rig.bob->pose.y() > 15.0f, "the documented overshoot: the load ends above the anchor");
}

// A chain of links hangs its full length with no per-link drift, which is what a rope builder
// and the bridge demo are built on.
void test_chain_of_links_hangs_without_drift() {
    FxScene scene = make_scene({20.0f, 20.0f});
    auto anchor = make_static(add_box(scene, "anchor", {10.0f, 18.0f}, {0.4f, 0.4f}));

    constexpr int kLinks = 6;
    constexpr float kSpacing = 0.4f;
    FxBody body;
    body.mass = 0.5f;

    // Links never collide with one another: one group instead of pairwise exclusions.
    auto group = scene.create_group("rope");
    std::vector<std::shared_ptr<FxEntity>> links;
    std::vector<std::shared_ptr<FxDistanceJoint>> joints;
    auto previous = anchor;
    for (int i = 0; i < kLinks; ++i) {
        const float y = 18.0f - kSpacing * static_cast<float>(i + 1);
        auto e = add_circle(scene, "link_" + std::to_string(i), {10.0f, y}, 0.12f, body);
        scene.add_to_group(group, e);
        auto joint = std::make_shared<FxDistanceJoint>("j_" + std::to_string(i), previous, e,
                                                       FxVec2f{0.0f, 0.0f}, FxVec2f{0.0f, 0.0f},
                                                       kSpacing, kSpacing);
        scene.add_joint(joint);
        links.push_back(e);
        joints.push_back(joint);
        previous = e;
    }

    step_for(scene, 3.0);

    for (size_t i = 0; i < joints.size(); ++i) {
        require_near(joints[i]->get_length(), kSpacing, 0.02f,
                     "link " + std::to_string(i) + " must keep its length under the load below it");
    }
    require_near(links.back()->pose.y(), 18.0f - kSpacing * static_cast<float>(kLinks), 0.05f,
                 "the free end must hang the chain's full length below the anchor");
    require_near(links.back()->pose.x(), 10.0f, 0.05f, "a hanging chain must not drift sideways");

    // Swing it and let it settle: the links must still be the length they started.
    links.back()->velocity.xy() = FxVec2f{5.0f, 0.0f};
    step_for(scene, 4.0);
    for (size_t i = 0; i < joints.size(); ++i) {
        require_near(joints[i]->get_length(), kSpacing, 0.03f,
                     "link " + std::to_string(i) + " must survive being swung");
    }
}

// One constraint, not a pin: a body hung from an offset anchor is free to rotate, and hangs
// with that anchor above its centre of mass.
void test_offset_anchor_leaves_rotation_free() {
    FxScene scene = make_scene({20.0f, 20.0f});
    auto anchor = make_static(add_box(scene, "anchor", {10.0f, 15.0f}, {0.4f, 0.4f}));
    FxBody body;
    body.mass = 1.0f;
    // Placed so the joint is already satisfied and the plate starts at rest: its left end sits
    // exactly one link-length below the anchor. A scene that starts violated is yanked straight
    // to the constraint, and the energy that puts in never leaves an undamped pendulum.
    auto plate = add_box(scene, "plate", {11.0f, 14.0f}, {2.0f, 0.3f}, body);
    auto group = scene.create_group("rig");
    scene.add_to_group(group, anchor);
    scene.add_to_group(group, plate);

    auto link = std::make_shared<FxDistanceJoint>("link", anchor, plate, FxVec2f{0.0f, 0.0f},
                                                  FxVec2f{-1.0f, 0.0f}, 1.0f, 1.0f);
    scene.add_joint(link);
    require_near(link->get_length(), 1.0f, 1e-4f, "the scene starts on the constraint");

    // Free to rotate and to swing, but never above the height it was released from: this is a
    // pendulum on a pendulum, so bound the energy rather than the pose.
    float most_rotation = 0.0f;
    float highest = plate->pose.y();
    float worst_length = 0.0f;
    for (int i = 0; i < 600; ++i) {
        scene.step(kFrame);
        most_rotation = std::max(most_rotation, std::fabs(plate->pose.theta()));
        highest = std::max(highest, plate->pose.y());
        worst_length = std::max(worst_length, std::fabs(link->get_length() - 1.0f));
    }
    require(most_rotation > 0.5f, "a distance joint must leave rotation free, most rotation was " +
                                      std::to_string(most_rotation));
    require(worst_length < 0.02f, "the link must hold its length while swinging, worst error " +
                                      std::to_string(worst_length));
    require(highest < 14.05f, "the plate must never swing above the height it fell from, reached " +
                                  std::to_string(highest));
}

// Damping takes energy out: the same swinging plate comes to rest with it, and rests hanging
// straight down with the held corner above its centre of mass.
void test_damping_settles_a_swinging_body() {
    FxScene scene = make_scene({20.0f, 20.0f});
    auto anchor = make_static(add_box(scene, "anchor", {10.0f, 15.0f}, {0.4f, 0.4f}));
    FxBody body;
    body.mass = 1.0f;
    auto plate = add_box(scene, "plate", {11.0f, 14.0f}, {2.0f, 0.3f}, body);
    plate->vel_damping = 2.0f;
    auto group = scene.create_group("rig");
    scene.add_to_group(group, anchor);
    scene.add_to_group(group, plate);

    auto link = std::make_shared<FxDistanceJoint>("link", anchor, plate, FxVec2f{0.0f, 0.0f},
                                                  FxVec2f{-1.0f, 0.0f}, 1.0f, 1.0f);
    scene.add_joint(link);

    step_for(scene, 20.0);
    require(plate->velocity.xy().norm() < 0.05f, "a damped body must come to rest, speed was " +
                                                     std::to_string(plate->velocity.xy().norm()));
    // At rest the whole assembly hangs vertically under the anchor: corner at 14, centre at 13.
    const FxVec2f held = plate->to_world_frame(FxVec2f{-1.0f, 0.0f});
    require(held.y() > plate->pose.y() + 0.5f,
            "the held corner must come to rest above the centre of mass");
    require_near(plate->pose.y(), 13.0f, 0.1f, "the centre must hang two link-lengths down");
}

// YAML: both spellings build, and the limits come out as authored.
void test_yaml_builds_distance_and_rope() {
    const char* yaml = R"(
scene:
  size: [20, 20]
  gravity: [0, -10]
entities:
  hook:
    pose: [10, 15, 0]
    physics: {mass: 0.0, gravity_scale: 0.0, external_forces_enabled: false}
    collision: {geometry: {rectangle: [0.4, 0.4]}}
  bob:
    pose: [10, 13, 0]
    physics: {mass: 1.0}
    collision: {geometry: {circle: 0.2}}
  swing:
    pose: [10, 12, 0]
    physics: {mass: 1.0}
    collision: {geometry: {circle: 0.2}}
joints:
  rigid:
    type: distance
    parent: hook
    child: bob
    length: 2.0
    compliance: 0.0001
  slack:
    type: rope
    parent: hook
    child: swing
    max_length: 3.5
)";
    FxScene scene = FxYAML::buildScene(YAML::Load(yaml));

    auto rigid = std::dynamic_pointer_cast<FxDistanceJoint>(scene.get_joint("rigid"));
    require(rigid != nullptr, "the distance joint must be built");
    require(rigid->is_distance(), "is_distance must report the type");
    require_near(rigid->get_min_length(), 2.0f, 1e-4f, "length pins the minimum");
    require_near(rigid->get_max_length(), 2.0f, 1e-4f, "length pins the maximum");

    auto rope = std::dynamic_pointer_cast<FxDistanceJoint>(scene.get_joint("slack"));
    require(rope != nullptr, "the rope joint must be built");
    require_near(rope->get_min_length(), 0.0f, 1e-4f, "a rope has no minimum");
    require_near(rope->get_max_length(), 3.5f, 1e-4f, "max_length carries through");

    // The rope is authored slack, so the swing falls before it catches.
    step_for(scene, 3.0);
    require_near(rope->get_length(), 3.5f, 2e-2f, "the rope must catch at its maximum");
    require_near(rigid->get_length(), 2.0f, 2e-2f, "the rigid link must hold its length");
}

// Deleting a joined body sweeps the joint rather than leaving it holding a dead entity.
void test_deleting_a_body_sweeps_the_joint() {
    Pendulum rig(1.5f, -1.0f, -1.0f);
    step_for(rig.scene, 0.5);
    require(rig.scene.joint_exists("link"), "the joint is registered");

    rig.scene.delete_entity("bob");
    rig.scene.step(kFrame);
    require(!rig.scene.joint_exists("link"), "a joint with a deleted body must be swept");
}

} // namespace

void run_distance_joint_tests() {
    test_rest_length_from_the_scene();
    test_rigid_link_holds_its_length();
    test_rope_hangs_slack_then_catches();
    test_compliance_stretches_by_the_predicted_amount();
    test_motor_reels_a_load_in();
    test_limits_reel_a_rigid_link_in();
    test_a_large_instant_reel_keeps_the_length_not_the_side();
    test_chain_of_links_hangs_without_drift();
    test_offset_anchor_leaves_rotation_free();
    test_damping_settles_a_swinging_body();
    test_yaml_builds_distance_and_rope();
    test_deleting_a_body_sweeps_the_joint();
    std::cout << "[PASS] distance_joint" << std::endl;
}
