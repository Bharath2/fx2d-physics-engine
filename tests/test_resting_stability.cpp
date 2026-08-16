// Resting-contact stability: no jitter/creep, stacks sleep; bounce/friction still work.

#include "Fx2D/Physics.h"

#include "test_harness.h"

#include <cmath>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>

namespace {

// Static collider: zero inverse mass, unaffected by gravity or impulses.
std::shared_ptr<FxEntity> make_ground(FxScene& scene, float cx, float cy, float w, float h,
                                      float elasticity = 0.6f) {
    auto e = std::make_shared<FxEntity>("ground");
    e->set_collision_geometry(FxCollisionShape(FxVec2f{w, h}));
    e->set_init_pose(FxVec3f{cx, cy, 0.0f});
    e->set_mass(0.0f);
    e->set_inertia(0.0f);
    e->enable_external_forces(false);
    e->gravity_scale = 0.0f;
    e->elasticity = elasticity;
    e->static_friction = 0.5f;
    e->dynamic_friction = 0.4f;
    scene.add_entity(e);
    return e;
}

std::shared_ptr<FxEntity> make_box(FxScene& scene, const std::string& name, float cx, float cy,
                                   float w, float h, float mass, float elasticity) {
    auto e = std::make_shared<FxEntity>(name);
    e->set_collision_geometry(FxCollisionShape(FxVec2f{w, h}));
    e->set_init_pose(FxVec3f{cx, cy, 0.0f});
    e->set_mass(mass);
    e->set_inertia();
    e->elasticity = elasticity;
    e->static_friction = 0.5f;
    e->dynamic_friction = 0.4f;
    scene.add_entity(e);
    return e;
}

std::shared_ptr<FxEntity> make_ball(FxScene& scene, const std::string& name, float cx, float cy,
                                    float r, float mass, float elasticity) {
    auto e = std::make_shared<FxEntity>(name);
    e->set_collision_geometry(FxCollisionShape(r));
    e->set_init_pose(FxVec3f{cx, cy, 0.0f});
    e->set_mass(mass);
    e->set_inertia();
    e->elasticity = elasticity;
    e->static_friction = 0.3f;
    e->dynamic_friction = 0.2f;
    scene.add_entity(e);
    return e;
}

FxScene make_scene() {
    FxScene scene({12.0f, 8.0f});
    scene.set_gravity(FxVec2f{0.0f, -10.0f});
    return scene;
}

struct Trace {
    float y_min = 1e9f;
    float y_max = -1e9f;
    float speed_max = 0.0f;

    void add(const std::shared_ptr<FxEntity>& e) {
        y_min = std::min(y_min, e->pose.y());
        y_max = std::max(y_max, e->pose.y());
        speed_max = std::max(speed_max, std::fabs(e->velocity.y()));
    }
    float span() const { return y_max - y_min; }
};

constexpr double kFrame = 1.0 / 60.0;

// A box placed exactly at rest must stay there. Before the fix this jittered
// 1.4 mm peak-to-peak at up to 0.049 m/s and never fell asleep.
void test_resting_box_does_not_jitter() {
    FxScene scene = make_scene();
    make_ground(scene, 6.0f, 0.75f, 12.0f, 1.5f);
    auto box = make_box(scene, "box", 6.0f, 2.125f, 1.0f, 1.25f, 5.0f, 0.6f);

    Trace trace;
    for (int i = 0; i < 180; ++i) { // 3 s
        scene.step(kFrame);
        trace.add(box);
    }

    require(trace.span() < 5e-4f, "resting box must not move more than 0.5 mm peak-to-peak");
    require(trace.speed_max < 0.02f, "resting box vertical speed must stay below 0.02 m/s");
    require(box->is_sleeping(), "a resting box must become quiet enough to fall asleep");
}

// With zero elasticity the box used to climb ~15 mm in 3 s, because the solver
// discarded exactly the negative corrections needed to cancel the warm-start kick.
void test_resting_box_does_not_creep_upward() {
    FxScene scene = make_scene();
    make_ground(scene, 6.0f, 0.75f, 12.0f, 1.5f);
    auto box = make_box(scene, "box", 6.0f, 2.125f, 1.0f, 1.25f, 5.0f, 0.0f);

    const float y0 = box->pose.y();
    for (int i = 0; i < 180; ++i)
        scene.step(kFrame);

    require(box->pose.y() - y0 < 1e-3f, "an inelastic resting box must not drift upward");
    require(std::fabs(box->pose.y() - y0) < 3e-3f,
            "an inelastic resting box must stay within 3 mm of its rest height");
    require(box->is_sleeping(), "an inelastic resting box must fall asleep");
}

// The headline goal: bodies stacked on each other stay stacked and go to sleep.
void test_three_box_stack_is_stable() {
    FxScene scene = make_scene();
    make_ground(scene, 6.0f, 0.75f, 12.0f, 1.5f);

    // 1 m cubes seated exactly on the ground top (y = 1.5) and on each other.
    auto b1 = make_box(scene, "b1", 6.0f, 2.0f, 1.0f, 1.0f, 2.0f, 0.4f);
    auto b2 = make_box(scene, "b2", 6.0f, 3.0f, 1.0f, 1.0f, 2.0f, 0.4f);
    auto b3 = make_box(scene, "b3", 6.0f, 4.0f, 1.0f, 1.0f, 2.0f, 0.4f);

    const float y1 = b1->pose.y(), y2 = b2->pose.y(), y3 = b3->pose.y();
    const float x1 = b1->pose.x(), x2 = b2->pose.x(), x3 = b3->pose.x();

    // Track the top box only after the stack has settled: the first second
    // legitimately compresses the contacts by a fraction of a millimetre.
    Trace top;
    for (int i = 0; i < 300; ++i) { // 5 s
        scene.step(kFrame);
        if (i >= 60) top.add(b3);
    }

    // Vertical: the stack must neither sink into itself nor inflate.
    require(std::fabs(b1->pose.y() - y1) < 6e-3f, "bottom box must hold its height");
    require(std::fabs(b2->pose.y() - y2) < 6e-3f, "middle box must hold its height");
    require(std::fabs(b3->pose.y() - y3) < 6e-3f, "top box must hold its height");

    // Horizontal: no sideways crawl out of the stack.
    require(std::fabs(b1->pose.x() - x1) < 5e-3f, "bottom box must not drift sideways");
    require(std::fabs(b2->pose.x() - x2) < 5e-3f, "middle box must not drift sideways");
    require(std::fabs(b3->pose.x() - x3) < 5e-3f, "top box must not drift sideways");

    require(top.span() < 2e-3f, "top of the stack must not oscillate");
    require(b1->is_sleeping() && b2->is_sleeping() && b3->is_sleeping(),
            "a settled stack must fall asleep");

    // Boxes must still be touching, not floating apart or fused together.
    const float gap_12 = (b2->pose.y() - b1->pose.y()) - 1.0f;
    const float gap_23 = (b3->pose.y() - b2->pose.y()) - 1.0f;
    require(std::fabs(gap_12) < 5e-3f && std::fabs(gap_23) < 5e-3f,
            "stacked boxes must stay in contact (1 m spacing for 1 m cubes)");
}

// Peak after lowest point — robust when impact spans multiple frames.
struct Bounce {
    float lowest = 1e9f;
    float peak_after_lowest = -1e9f;

    void add(float y) {
        if (y < lowest) {
            lowest = y;
            peak_after_lowest = y;
        }
        peak_after_lowest = std::max(peak_after_lowest, y);
    }
    float rebound() const { return peak_after_lowest - lowest; }
};

// Elastic bounce must survive the resting-contact fix.
void test_elastic_ball_still_bounces() {
    FxScene scene = make_scene();
    // Contact elasticity is min(A, B), so the ground must be elastic too.
    make_ground(scene, 6.0f, 0.75f, 12.0f, 1.5f, 0.9f);
    auto ball = make_ball(scene, "ball", 6.0f, 4.0f, 0.25f, 1.0f, 0.9f);

    const float start_y = ball->pose.y();
    const float floor_y = 1.5f + 0.25f; // ground top + radius
    const float drop = start_y - floor_y;

    Bounce bounce;
    for (int i = 0; i < 300; ++i) {
        scene.step(kFrame);
        bounce.add(ball->pose.y());
    }

    require(bounce.lowest < floor_y + 0.02f, "ball must reach the ground");
    require(bounce.rebound() > 0.15f * drop,
            "an elasticity 0.9 ball must rebound to more than 15% of its drop height");
    require(bounce.peak_after_lowest < start_y,
            "an elastic ball must never rebound above its drop height");
}

// Inelastic landings must not gain energy (old warm-start bug rebounded >100%).
void test_inelastic_ball_does_not_gain_energy() {
    FxScene scene = make_scene();
    make_ground(scene, 6.0f, 0.75f, 12.0f, 1.5f);
    auto ball = make_ball(scene, "ball", 6.0f, 4.0f, 0.25f, 1.0f, 0.0f);

    const float start_y = ball->pose.y();
    const float floor_y = 1.5f + 0.25f;

    Bounce bounce;
    for (int i = 0; i < 300; ++i) {
        scene.step(kFrame);
        bounce.add(ball->pose.y());
    }

    require(bounce.lowest < floor_y + 0.02f, "ball must reach the ground");
    require(bounce.rebound() < 0.02f, "an inelastic ball must not bounce back off the ground");
    require(bounce.peak_after_lowest < start_y,
            "an inelastic ball must never regain its drop height");
    require(ball->is_sleeping(), "a landed inelastic ball must fall asleep");
}

// Same guarantee for a flat-faced body with two contact points.
void test_inelastic_box_landing_does_not_bounce() {
    FxScene scene = make_scene();
    make_ground(scene, 6.0f, 0.75f, 12.0f, 1.5f);
    auto box = make_box(scene, "box", 6.0f, 2.6f, 1.0f, 1.0f, 3.0f, 0.0f);

    const float rest_y = 2.0f; // ground top 1.5 + half height
    Bounce bounce;
    for (int i = 0; i < 300; ++i) {
        scene.step(kFrame);
        bounce.add(box->pose.y());
    }

    require(bounce.lowest < rest_y + 0.02f, "box must land");
    require(bounce.rebound() < 0.02f, "an inelastic box must not rebound after landing");
    require(box->is_sleeping(), "a landed inelastic box must fall asleep");
}

// Friction regression: the friction cone budget changed (it used to double count
// each contact point), so sliding must still be damped out.
void test_friction_stops_a_sliding_box() {
    FxScene scene = make_scene();
    make_ground(scene, 6.0f, 0.75f, 12.0f, 1.5f);
    auto box = make_box(scene, "box", 4.0f, 2.0f, 1.0f, 1.0f, 2.0f, 0.0f);

    const float x0 = box->pose.x();
    box->velocity.x() = 3.0f;

    for (int i = 0; i < 300; ++i)
        scene.step(kFrame); // 5 s

    const float travelled = box->pose.x() - x0;
    require(travelled > 0.05f, "a box pushed at 3 m/s must actually slide");
    require(std::fabs(box->velocity.x()) < 0.1f,
            "friction must bring the sliding box to rest within 5 s");
    require(box->is_sleeping(), "a box stopped by friction must fall asleep");
}

// Penetration recovery must keep resting overlap small: the fix must not be a
// matter of simply letting bodies sink until contacts stop reporting.
void test_resting_penetration_stays_small() {
    FxScene scene = make_scene();
    make_ground(scene, 6.0f, 0.75f, 12.0f, 1.5f);
    auto box = make_box(scene, "box", 6.0f, 2.2f, 1.0f, 1.0f, 4.0f, 0.0f);

    for (int i = 0; i < 240; ++i)
        scene.step(kFrame);

    const float ground_top = 1.5f;
    const float box_bottom = box->pose.y() - 0.5f;
    const float overlap = ground_top - box_bottom;
    require(overlap < 6e-3f, "resting overlap must stay under 6 mm");
    require(overlap > -6e-3f, "a resting box must not hover above the ground");
}

// Restitution mixes with max, so the bounciest surface in a pair decides the bounce.
//
// Under the old min rule a lively ball inherited whatever dead thing it landed on and simply
// stopped, which made "make this ball bouncy" mean nothing unless every surface agreed.
void test_bouncy_body_bounces_off_a_dead_surface() {
    FxScene scene = make_scene();
    make_ground(scene, 6.0f, 1.0f, 8.0f, 0.5f, 0.0f); // completely inelastic floor
    auto ball = make_ball(scene, "ball", 6.0f, 5.0f, 0.3f, 1.0f, 0.7f); // but a lively ball

    float lowest = ball->pose.y();
    float rebound_peak = 0.0f;
    bool touched_down = false;
    for (int i = 0; i < 400; ++i) {
        scene.step(kFrame);
        lowest = std::min(lowest, ball->pose.y());
        if (!touched_down && ball->velocity.y() > 0.5f) touched_down = true;
        if (touched_down) rebound_peak = std::max(rebound_peak, ball->pose.y());
    }

    require(touched_down, "the ball must land and rebound rather than sticking to the floor");
    require(rebound_peak > lowest + 0.5f,
            "a 0.7-elasticity ball must bounce clear of a dead floor, rose only " +
                std::to_string(rebound_peak - lowest) + " above its lowest point");
}

// Friction still mixes with min, so the slipperiest surface wins and ice stays slippery.
void test_slippery_surface_wins_over_grippy_body() {
    FxScene scene = make_scene();
    auto ground = make_ground(scene, 6.0f, 1.0f, 8.0f, 0.5f, 0.0f);
    ground->static_friction = 0.0f; // ice
    ground->dynamic_friction = 0.0f;

    auto box = make_box(scene, "box", 6.0f, 1.75f, 0.5f, 0.5f, 1.0f, 0.0f);
    box->static_friction = 1.0f; // grippy box
    box->dynamic_friction = 1.0f;
    box->velocity = FxVec3f{3.0f, 0.0f, 0.0f};

    const float x0 = box->pose.x();
    for (int i = 0; i < 120; ++i)
        scene.step(kFrame);

    require(box->pose.x() - x0 > 4.0f, "a grippy box on ice must keep sliding, travelled only " +
                                           std::to_string(box->pose.x() - x0));
}

} // namespace

void run_resting_stability_tests() {
    test_bouncy_body_bounces_off_a_dead_surface();
    test_slippery_surface_wins_over_grippy_body();
    test_resting_box_does_not_jitter();
    test_resting_box_does_not_creep_upward();
    test_three_box_stack_is_stable();
    test_elastic_ball_still_bounces();
    test_inelastic_ball_does_not_gain_energy();
    test_inelastic_box_landing_does_not_bounce();
    test_friction_stops_a_sliding_box();
    test_resting_penetration_stays_small();
    std::cout << "Resting stability tests passed." << std::endl;
}
