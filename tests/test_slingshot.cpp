// The slingshot mechanic, driven headlessly by injecting mouse state through the same
// producer API a renderer fills, since a GUI cannot run in CI.

#include "Fx2D/Physics.h"

#include "angry_boxes/slingshot.h"
#include "test_harness.h"
#include "test_scene_builders.h"

#include <cmath>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

namespace {

constexpr double kFrame = 1.0 / 60.0;

struct World {
    FxScene scene{FxVec2ui{16u, 9u}};
    std::shared_ptr<FxEntity> ball;
    std::vector<std::shared_ptr<FxEntity>> tower;
};

std::shared_ptr<FxEntity> add_box(FxScene& scene, const std::string& name, float cx, float cy,
                                  float w, float h, float mass) {
    return ::add_box(scene, name, {cx, cy}, {w, h},
                     {.mass = mass,
                      .elasticity = 0.05f,
                      .static_friction = 0.7f,
                      .dynamic_friction = 0.6f});
}

// A ground plane, a short tower, and the projectile parked at the anchor.
World make_world() {
    World w;
    w.scene.set_gravity(FxVec2f{0.0f, -10.0f});

    auto ground = add_box(w.scene, "ground", 8.0f, 0.5f, 16.0f, 1.0f, 1.0f);
    ground->set_mass(0.0f);
    ground->set_inertia(0.0f);
    ground->enable_external_forces(false);
    ground->gravity_scale = 0.0f;
    ground->static_friction = 0.8f;
    ground->dynamic_friction = 0.7f;

    w.tower.push_back(add_box(w.scene, "box_l1", 10.5f, 1.4f, 0.5f, 0.8f, 0.6f));
    w.tower.push_back(add_box(w.scene, "box_r1", 12.5f, 1.4f, 0.5f, 0.8f, 0.6f));
    w.tower.push_back(add_box(w.scene, "lintel1", 11.5f, 1.95f, 2.6f, 0.3f, 0.8f));

    w.ball = ::add_circle(w.scene, "ball", {3.0f, 3.0f}, 0.32f,
                          {.mass = 1.6f, .elasticity = 0.25f, .ccd = true});
    return w;
}

// One frame: set the mouse state, then step.
void frame(FxScene& scene, FxSlingshot& sling, const std::shared_ptr<FxEntity>& ball,
           const FxVec2f& cursor, bool button_down) {
    scene.input().begin_frame();
    scene.input().set_mouse_position(cursor, FxVec2f{0.0f, 0.0f});
    scene.input().set_mouse_button(FxMouseButton::Left, button_down);
    sling.update(scene, ball);
    scene.step(kFrame);
}

// Grab at the anchor, drag to `pull_to`, release. Returns the ball's speed just after launch.
float grab_drag_release(World& w, FxSlingshot& sling, const FxVec2f& pull_to) {
    // Settle while parked.
    for (int i = 0; i < 5; ++i)
        frame(w.scene, sling, w.ball, sling.anchor, false);

    // Press on the ball.
    frame(w.scene, sling, w.ball, sling.anchor, true);

    // Drag out to the pull position over several frames.
    for (int i = 1; i <= 10; ++i) {
        const float t = static_cast<float>(i) / 10.0f;
        frame(w.scene, sling, w.ball, sling.anchor + (pull_to - sling.anchor) * t, true);
    }

    // Release.
    frame(w.scene, sling, w.ball, pull_to, false);
    return w.ball->velocity.head<2>().norm();
}

// Parked before launch, the ball must not fall or drift.
void test_ball_waits_at_the_anchor() {
    World w = make_world();
    FxSlingshot sling;
    sling.reset(w.ball);

    for (int i = 0; i < 120; ++i)
        frame(w.scene, sling, w.ball, FxVec2f{8.0f, 8.0f}, false); // cursor nowhere near

    require_near(w.ball->pose.x(), sling.anchor.x(), 1e-3f, "a parked ball must not drift in x");
    require_near(w.ball->pose.y(), sling.anchor.y(), 1e-3f, "a parked ball must not fall");
    require(!sling.dragging, "nothing should be grabbed when the cursor is far away");
    require(!sling.launched, "the ball must not launch itself");
}

// Clicking far from the ball must not pick it up.
void test_click_away_from_ball_does_not_grab() {
    World w = make_world();
    FxSlingshot sling;
    sling.reset(w.ball);

    frame(w.scene, sling, w.ball, FxVec2f{9.0f, 5.0f}, true);
    require(!sling.dragging, "a click outside the grab radius must not grab the ball");
    require(!sling.launched, "a click outside the grab radius must not launch");
}

// Dragging moves the ball with the cursor, and the pull is capped.
void test_drag_follows_cursor_and_clamps() {
    World w = make_world();
    FxSlingshot sling;
    sling.reset(w.ball);

    frame(w.scene, sling, w.ball, sling.anchor, true);
    require(sling.dragging, "pressing on the ball must start a drag");

    // A modest pull is followed exactly.
    const FxVec2f modest = sling.anchor + FxVec2f{-1.0f, -0.5f};
    frame(w.scene, sling, w.ball, modest, true);
    require_near(w.ball->pose.x(), modest.x(), 1e-3f, "the ball must follow the cursor in x");
    require_near(w.ball->pose.y(), modest.y(), 1e-3f, "the ball must follow the cursor in y");

    // A wild pull is clamped to max_pull.
    frame(w.scene, sling, w.ball, sling.anchor + FxVec2f{-40.0f, -30.0f}, true);
    const float distance = (w.ball->pose.xy() - sling.anchor).norm();
    require(distance <= sling.max_pull + 1e-3f,
            "the pull must be clamped to max_pull, got " + std::to_string(distance));
    require(distance > sling.max_pull - 1e-2f,
            "a wild pull should sit at the clamp, got " + std::to_string(distance));

    // Held still, the ball must not accumulate velocity from being moved.
    require(w.ball->velocity.head<2>().norm() < 1e-3f,
            "a dragged ball must carry no velocity, has " +
                std::to_string(w.ball->velocity.head<2>().norm()));
}

// Releasing launches the ball away from the anchor, at a speed set by the pull.
void test_release_launches_the_ball() {
    World w = make_world();
    FxSlingshot sling;
    sling.reset(w.ball);

    const FxVec2f pull_to = sling.anchor + FxVec2f{-1.6f, -0.8f}; // back and down
    const float speed = grab_drag_release(w, sling, pull_to);

    require(sling.launched, "releasing must launch the ball");
    require(!sling.dragging, "releasing must end the drag");
    require(speed > 5.0f, "the launch must be fast, got " + std::to_string(speed) + " m/s");

    // Pulled back-and-down, so it must fly forward-and-up.
    require(w.ball->velocity.x() > 0.0f, "pulling back must fire the ball forward");
    require(w.ball->velocity.y() > 0.0f, "pulling down must fire the ball upward");

    // And gravity must be back on: it should arc, not fly straight.
    for (int i = 0; i < 40; ++i)
        frame(w.scene, sling, w.ball, pull_to, false);
    require(w.ball->velocity.y() < 5.0f, "gravity must reassert itself after launch");
}

// A bigger pull throws harder.
void test_bigger_pull_launches_faster() {
    World small = make_world();
    FxSlingshot sling_small;
    sling_small.reset(small.ball);
    const float slow =
        grab_drag_release(small, sling_small, sling_small.anchor + FxVec2f{-0.5f, 0.0f});

    World big = make_world();
    FxSlingshot sling_big;
    sling_big.reset(big.ball);
    const float fast = grab_drag_release(big, sling_big, sling_big.anchor + FxVec2f{-2.0f, 0.0f});

    require(fast > slow * 2.0f, "a 4x longer pull must launch much faster: " +
                                    std::to_string(slow) + " vs " + std::to_string(fast));
}

// The whole point: a launched ball reaches the tower and knocks it over.
void test_launched_ball_topples_the_tower() {
    World w = make_world();
    FxSlingshot sling;
    sling.reset(w.ball);

    std::vector<float> start_y;
    for (const auto& piece : w.tower)
        start_y.push_back(piece->pose.y());

    // Aim: pull straight back. Launched flat from y=3.0 at 18 m/s, the ball falls ~1.5 m over
    // the 10 m to the tower and arrives around y=1.45, which is square in the lower boxes.
    // Pulling back *and down* fires it upward instead, and it sails over the top.
    grab_drag_release(w, sling, sling.anchor + FxVec2f{-2.0f, 0.0f});

    bool ball_hit_tower = false;
    for (int i = 0; i < 400; ++i) {
        frame(w.scene, sling, w.ball, sling.anchor, false);
        for (const auto& contact : w.scene.contacts()) {
            if (!contact.entity1 || !contact.entity2) continue;
            const std::string& a = contact.entity1->get_name();
            const std::string& b = contact.entity2->get_name();
            const bool involves_ball = (a == "ball" || b == "ball");
            const bool involves_piece = (a.rfind("box_", 0) == 0 || a.rfind("lintel", 0) == 0 ||
                                         b.rfind("box_", 0) == 0 || b.rfind("lintel", 0) == 0);
            if (involves_ball && involves_piece) ball_hit_tower = true;
        }
    }

    require(w.ball->pose.x() > 8.0f,
            "the ball must travel down-range, reached x=" + std::to_string(w.ball->pose.x()));
    require(ball_hit_tower, "the launched ball must actually strike the tower");

    int disturbed = 0;
    for (size_t i = 0; i < w.tower.size(); ++i) {
        const bool fell = (start_y[i] - w.tower[i]->pose.y()) > 0.2f;
        const bool tipped = std::fabs(w.tower[i]->pose.theta()) > 0.3f;
        const bool shoved = std::fabs(w.tower[i]->pose.x() - (i == 0 ? 10.5f :
                                                              i == 1 ? 12.5f :
                                                                       11.5f)) > 0.3f;
        if (fell || tipped || shoved) ++disturbed;
    }
    require(disturbed > 0, "the hit must knock the tower about, nothing moved");
}

// R re-arms the slingshot and puts the ball back.
void test_reset_rearms_the_slingshot() {
    World w = make_world();
    FxSlingshot sling;
    sling.reset(w.ball);

    grab_drag_release(w, sling, sling.anchor + FxVec2f{-2.0f, 0.0f});
    require(sling.launched, "the ball must be in flight");
    for (int i = 0; i < 30; ++i)
        frame(w.scene, sling, w.ball, sling.anchor, false);
    require(w.ball->pose.x() > sling.anchor.x() + 1.0f, "the ball must have left the anchor");

    // Press R.
    w.scene.input().begin_frame();
    w.scene.input().set_key(FxKey::R, true);
    sling.update(w.scene, w.ball);
    w.scene.step(kFrame);

    require(!sling.launched, "reset must re-arm the slingshot");
    require_near(w.ball->pose.x(), sling.anchor.x(), 1e-3f, "reset must park the ball in x");
    require_near(w.ball->pose.y(), sling.anchor.y(), 1e-3f, "reset must park the ball in y");
    require(w.ball->velocity.head<2>().norm() < 1e-3f, "a reset ball must be at rest");
}

// R must re-arm from any state, including mid-drag.
void test_reset_key_works_while_dragging() {
    World w = make_world();
    FxSlingshot sling;
    sling.reset(w.ball);

    frame(w.scene, sling, w.ball, sling.anchor, true);
    frame(w.scene, sling, w.ball, sling.anchor + FxVec2f{-1.8f, 0.0f}, true);
    require(sling.dragging, "the ball must be under drag before the reset");

    w.scene.input().begin_frame();
    w.scene.input().set_key(FxKey::R, true);
    sling.update(w.scene, w.ball);
    w.scene.step(kFrame);

    require(!sling.dragging, "R must cancel an in-progress drag");
    require(!sling.launched, "R must leave the slingshot armed");
    require_near(w.ball->pose.x(), sling.anchor.x(), 1e-3f, "R must park the ball back at rest");
    require(w.ball->velocity.head<2>().norm() < 1e-3f,
            "a cancelled drag must not leave the ball moving");
}

// A scene reset must re-arm through the callback, or the ball returns still flagged launched.
void test_scene_reset_rearms_through_callback() {
    World w = make_world();
    FxSlingshot sling;
    sling.reset(w.ball);
    w.scene.set_reset_callback([&](FxScene&) { sling.reset(w.ball); });

    grab_drag_release(w, sling, sling.anchor + FxVec2f{-2.0f, 0.0f});
    for (int i = 0; i < 60; ++i)
        frame(w.scene, sling, w.ball, sling.anchor, false);
    require(sling.launched, "the ball must be in flight before the scene reset");
    require(w.ball->pose.x() > 5.0f, "the ball must have travelled");

    w.scene.reset();

    require(!sling.launched, "a scene reset must re-arm the slingshot");
    require_near(w.ball->pose.x(), sling.anchor.x(), 1e-3f, "the ball must be parked in x");
    require_near(w.ball->pose.y(), sling.anchor.y(), 1e-3f, "the ball must be parked in y");

    // And it must stay parked rather than dropping off the post.
    for (int i = 0; i < 120; ++i)
        frame(w.scene, sling, w.ball, FxVec2f{15.0f, 8.0f}, false);
    require_near(w.ball->pose.y(), sling.anchor.y(), 1e-3f,
                 "the re-armed ball must hold its position, not fall, y=" +
                     std::to_string(w.ball->pose.y()));
}

// A heavy ball shoves light crates aside fast enough to look like tunneling; this pins that
// the contacts are genuinely there.
void test_fast_ball_hits_every_crate_in_its_path() {
    for (float speed : {20.0f, 35.0f, 50.0f}) {
        FxScene scene({30.0f, 12.0f});
        scene.set_gravity(FxVec2f{0.0f, -10.0f});

        auto ground = add_box(scene, "ground", 15.0f, 0.5f, 30.0f, 1.0f, 1.0f);
        ground->set_mass(0.0f);
        ground->set_inertia(0.0f);
        ground->enable_external_forces(false);
        ground->gravity_scale = 0.0f;

        add_box(scene, "c1", 10.5f, 1.4f, 0.5f, 0.8f, 0.32f);
        add_box(scene, "c2", 11.5f, 1.4f, 0.5f, 0.8f, 0.32f);

        auto ball = ::add_circle(scene, "ball", {4.0f, 1.4f}, 0.32f,
                                 {.mass = 1.6f, .elasticity = 0.5f, .ccd = true});
        ball->velocity = FxVec3f{speed, 0.0f, 0.0f};

        bool hit_c1 = false, hit_c2 = false;
        for (int i = 0; i < 300; ++i) {
            scene.step(kFrame);
            for (const auto& c : scene.contacts()) {
                if (!c.entity1 || !c.entity2) continue;
                const std::string a = c.entity1->get_name(), b = c.entity2->get_name();
                if (a != "ball" && b != "ball") continue;
                if (a == "c1" || b == "c1") hit_c1 = true;
                if (a == "c2" || b == "c2") hit_c2 = true;
            }
        }
        require(hit_c1, "a ball at " + std::to_string(speed) +
                            " m/s must register contact with the first crate");
        require(hit_c2, "a ball at " + std::to_string(speed) +
                            " m/s must register contact with the second crate too");
    }
}

} // namespace

void run_slingshot_tests() {
    test_reset_key_works_while_dragging();
    test_scene_reset_rearms_through_callback();
    test_fast_ball_hits_every_crate_in_its_path();
    test_ball_waits_at_the_anchor();
    test_click_away_from_ball_does_not_grab();
    test_drag_follows_cursor_and_clamps();
    test_release_launches_the_ball();
    test_bigger_pull_launches_faster();
    test_launched_ball_topples_the_tower();
    test_reset_rearms_the_slingshot();
    std::cout << "Slingshot tests passed." << std::endl;
}
