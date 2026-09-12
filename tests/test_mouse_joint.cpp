// The click-drag spring: a body follows a moving target, holds against gravity, refuses what
// it should refuse, lets go when told, and is driven from injected input without a renderer.

#include "Fx2D/Scene.h"
#include "Fx2D/YamlUtils.h"

#include "test_harness.h"
#include "test_scene_builders.h"

#include <cmath>
#include <iostream>
#include <memory>
#include <string>

namespace {

constexpr double kFrame = 1.0 / 60.0;

void step_for(FxScene& scene, double seconds) {
    const int n = static_cast<int>(std::lround(seconds / kFrame));
    for (int i = 0; i < n; ++i)
        scene.step(kFrame);
}

// A floor and one box resting on it, at rest before any test touches it.
struct Rig {
    FxScene scene = make_scene({20.0f, 12.0f});
    std::shared_ptr<FxEntity> floor;
    std::shared_ptr<FxEntity> box;

    Rig() {
        floor = make_static(add_box(scene, "floor", {10.0f, 0.5f}, {20.0f, 1.0f}));
        FxBody body;
        body.static_friction = 0.5f;
        body.dynamic_friction = 0.4f;
        box = add_box(scene, "box", {5.0f, 1.5f}, {1.0f, 1.0f}, body);
        step_for(scene, 0.5);
    }
};

void test_refuses_static_sensor_and_null() {
    Rig rig;
    FxMouseJoint& joint = rig.scene.mouse_joint();

    require(!joint.attach(nullptr, {1.0f, 1.0f}), "null entity must be refused");
    require(!joint.attach(rig.floor, {10.0f, 0.5f}), "static body must be refused");

    rig.box->is_sensor = true;
    require(!joint.attach(rig.box, {5.0f, 1.5f}), "sensor must be refused");
    rig.box->is_sensor = false;

    rig.box->enabled = false;
    require(!joint.attach(rig.box, {5.0f, 1.5f}), "disabled body must be refused");
    rig.box->enabled = true;

    require(joint.attach(rig.box, {5.0f, 1.5f}), "dynamic body must attach");
    require(joint.attached() && joint.entity() == rig.box, "joint must report the held body");
    joint.release();
    require(!joint.attached(), "release must detach");
}

// Lift the box off the floor and hold it: it settles within a few cm of the target, with the
// gravity sag the spring stiffness predicts, and stops moving.
void test_holds_against_gravity() {
    Rig rig;
    FxMouseJoint& joint = rig.scene.mouse_joint();
    require(joint.attach(rig.box, {5.0f, 1.5f}), "attach at the box centre");

    const FxVec2f target{5.0f, 5.0f};
    joint.set_target(target);
    step_for(rig.scene, 2.0);

    const FxVec2f anchor = joint.anchor_world();
    // k = m (2 pi f)^2 with f = 5 Hz, m = 1 kg -> ~987 N/m; weight 10 N -> ~1 cm of sag.
    require_near(anchor.x(), target.x(), 0.02f, "held box must sit under the target in x");
    require_near(anchor.y(), target.y() - 0.0101f, 0.02f, "held box must sag by weight / k");
    require(rig.box->velocity.head<2>().norm() < 0.05f, "held box must come to rest");
    require(!rig.box->is_sleeping(), "a held body must stay awake");
}

// Sweep the target sideways at 3 m/s: the anchor tracks it with bounded lag, and the box is
// back on the floor within a second of letting go.
void test_follows_moving_target_then_drops() {
    Rig rig;
    FxMouseJoint& joint = rig.scene.mouse_joint();
    require(joint.attach(rig.box, {5.0f, 1.5f}), "attach");

    FxVec2f target{5.0f, 4.0f};
    joint.set_target(target);
    step_for(rig.scene, 1.0);

    float worst_lag = 0.0f;
    for (int i = 0; i < 60; ++i) {
        target.x() += 3.0f * static_cast<float>(kFrame);
        joint.set_target(target);
        rig.scene.step(kFrame);
        worst_lag = std::max(worst_lag, (joint.anchor_world() - target).norm());
    }
    require(worst_lag < 0.25f,
            "anchor must track a 3 m/s target within 25 cm, lag was " + std::to_string(worst_lag));
    require_near(joint.anchor_world().x(), target.x(), 0.15f, "box must have followed in x");

    joint.release();
    step_for(rig.scene, 1.5);
    require_near(rig.box->pose.y(), 1.5f, 0.05f, "released box must fall back onto the floor");
}

// The force cap keeps a violent target jump from launching the body.
void test_force_cap_bounds_speed() {
    Rig rig;
    FxMouseJoint& joint = rig.scene.mouse_joint();
    joint.max_force_per_kg = 50.0f; // 50 N on a 1 kg box: at most 5 g of pull
    require(joint.attach(rig.box, {5.0f, 1.5f}), "attach");

    joint.set_target({15.0f, 8.0f}); // ten metres away, at once
    float peak_speed = 0.0f;
    for (int i = 0; i < 30; ++i) {
        rig.scene.step(kFrame);
        peak_speed = std::max(peak_speed, rig.box->velocity.head<2>().norm());
    }
    // 50 N for 0.5 s on 1 kg is 25 m/s at most, before friction and gravity take their share.
    require(peak_speed < 25.0f,
            "capped pull must not exceed a = F/m, peak was " + std::to_string(peak_speed));
    require(peak_speed > 3.0f, "the cap must still let the body move");
}

// With mouse drag enabled the scene drives the joint from input(): press on the box grabs it,
// holding moves it, releasing drops it, and pressing on empty space grabs nothing even when the
// cursor later crosses the body.
void test_driven_from_input() {
    Rig rig;
    rig.scene.enable_mouse_drag(true);
    FxInput& in = rig.scene.input();
    const FxMouseJoint& joint = rig.scene.mouse_joint();

    // Press on empty space, then drag across the box while held: nothing attaches.
    in.begin_frame();
    in.set_mouse_position({2.0f, 1.5f}, {0.0f, 0.0f});
    in.set_mouse_button(FxMouseButton::Left, true);
    rig.scene.step(kFrame);
    require(!joint.attached(), "a press on empty space must grab nothing");
    for (int i = 0; i < 20; ++i) {
        in.begin_frame();
        in.set_mouse_position({2.0f + 0.2f * static_cast<float>(i), 1.5f}, {0.0f, 0.0f});
        in.set_mouse_button(FxMouseButton::Left, true);
        rig.scene.step(kFrame);
    }
    require(!joint.attached(), "a held drag crossing the box must not pick it up");
    in.begin_frame();
    in.set_mouse_button(FxMouseButton::Left, false);
    rig.scene.step(kFrame);

    // Press on the box: attached, and it follows the cursor.
    in.begin_frame();
    in.set_mouse_position({5.0f, 1.6f}, {0.0f, 0.0f});
    in.set_mouse_button(FxMouseButton::Left, true);
    rig.scene.step(kFrame);
    require(joint.attached() && joint.entity() == rig.box, "a press on the box must grab it");

    for (int i = 0; i < 90; ++i) {
        in.begin_frame();
        in.set_mouse_position({5.0f + 3.0f * static_cast<float>(i) / 90.0f, 4.0f}, {0.0f, 0.0f});
        in.set_mouse_button(FxMouseButton::Left, true);
        rig.scene.step(kFrame);
    }
    require_near(rig.box->pose.x(), 8.0f, 0.3f, "held box must have followed the cursor in x");
    require(rig.box->pose.y() > 3.0f, "held box must have been lifted");

    in.begin_frame();
    in.set_mouse_button(FxMouseButton::Left, false);
    rig.scene.step(kFrame);
    require(!joint.attached(), "releasing the button must let go");
}

// Deleting the held body, or resetting the scene, releases the joint rather than leaving it
// pulling on a body the scene no longer owns.
void test_deletion_and_reset_release() {
    Rig rig;
    FxMouseJoint& joint = rig.scene.mouse_joint();

    require(joint.attach(rig.box, {5.0f, 1.5f}), "attach");
    rig.scene.delete_entity("box");
    rig.scene.step(kFrame);
    require(!joint.attached(), "deleting the held body must release the joint");

    rig.scene.reset();
    auto box = rig.scene.get_entity("box");
    require(box != nullptr, "reset must restore the box");
    require(joint.attach(box, {5.0f, 1.5f}), "attach after reset");
    rig.scene.reset();
    require(!joint.attached(), "reset must release the joint");
}

// YAML opt-in reaches the scene.
void test_yaml_opt_in() {
    // Only the scene block matters here; the entity is a minimum viable body.
    const char* yaml = "scene:\n  size: [10, 10]\n  mouse_drag: true\n"
                       "entities:\n  b:\n    pose: [5, 5, 0]\n    physics: {mass: 1.0}\n"
                       "    collision:\n      geometry:\n        circle: 0.5\n";
    FxScene scene = FxYAML::buildScene(YAML::Load(yaml));
    require(scene.mouse_drag_enabled(), "scene: mouse_drag: true must enable input-driven drag");
}

} // namespace

void run_mouse_joint_tests() {
    test_refuses_static_sensor_and_null();
    test_holds_against_gravity();
    test_follows_moving_target_then_drops();
    test_force_cap_bounds_speed();
    test_driven_from_input();
    test_deletion_and_reset_release();
    test_yaml_opt_in();
    std::cout << "[PASS] mouse_joint" << std::endl;
}
