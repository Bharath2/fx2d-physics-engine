// Adversarial scenes: tall stacks, high mass ratios, degenerate geometry, restitution
// chains, and spinning bodies. These are the scenes mature engines are judged on, and the
// point of the suite is to pin down the envelope the solver provably owns.
//
// The thresholds here are measured, not aspirational. Where the solver currently fails, the
// limit is recorded in docs/ToDo.md rather than asserted, so this file stays green and any
// regression inside the proven envelope shows up immediately.
//
// Assert invariants, not appearances. Two scenes here look like solver failures and are not:
// a fast-spinning box vaults metres into the air (it has the rotational energy to pay for the
// height), and a heavy box dropped on a light column scatters it across the floor (a topple,
// with every body still separated). Both were briefly mistaken for bugs. The lesson is in the
// tests: check conserved energy rather than height, and check pairwise overlap rather than
// final height, because a toppled stack and a collapsed one both end up flat on the ground.
//
// Note every helper sets BOTH visual and collision geometry. FxEntity::set_inertia() derives
// inertia from the *visual* shape, so a collision-only body silently gets the inertia of the
// default 0.5-radius circle, which would make every rotational result here meaningless.

#include "Fx2D/Physics.h"

#include "test_harness.h"

#include <cmath>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

namespace {

constexpr double kFrame = 1.0 / 60.0;

// Settle time for the stacking scenes. Long enough for a stack to fail if it is going
// to, short enough that the suite stays usable under ASan/UBSan in CI.
constexpr int kSettleSteps = 250;

FxScene make_scene() {
    FxScene scene({40.0f, 40.0f});
    scene.set_gravity(FxVec2f{0.0f, -10.0f});
    return scene;
}

std::shared_ptr<FxEntity> add_box(FxScene& scene, const std::string& name, float cx, float cy,
                                  float w, float h, float mass) {
    auto e = std::make_shared<FxEntity>(name);
    e->set_visual_geometry(FxVisualShape(FxVec2f{w, h}));
    e->set_collision_geometry(FxCollisionShape(FxVec2f{w, h}));
    e->set_init_pose(FxVec3f{cx, cy, 0.0f});
    e->set_mass(mass);
    e->set_inertia();
    e->elasticity = 0.0f;
    e->static_friction = 0.6f;
    e->dynamic_friction = 0.5f;
    scene.add_entity(e);
    return e;
}

std::shared_ptr<FxEntity> add_ground(FxScene& scene) {
    auto e = add_box(scene, "ground", 20.0f, 1.0f, 30.0f, 1.0f, 1.0f);
    e->set_mass(0.0f);
    e->set_inertia(0.0f);
    e->enable_external_forces(false);
    e->gravity_scale = 0.0f;
    return e;
}

void run(FxScene& scene, int steps) {
    for (int i = 0; i < steps; ++i)
        scene.step(kFrame);
}

// Worst-case deviation of a stack from the column it started as.
struct StackError {
    float max_lateral_drift = 0.0f;
    float max_sink = 0.0f;
    float max_tilt = 0.0f;
};

StackError measure(const std::vector<std::shared_ptr<FxEntity>>& boxes,
                   const std::vector<float>& start_y, float column_x) {
    StackError e;
    for (size_t i = 0; i < boxes.size(); ++i) {
        e.max_lateral_drift =
            std::max(e.max_lateral_drift, std::fabs(boxes[i]->pose.x() - column_x));
        e.max_sink = std::max(e.max_sink, start_y[i] - boxes[i]->pose.y());
        e.max_tilt = std::max(e.max_tilt, std::fabs(boxes[i]->pose.theta()));
    }
    return e;
}

// Builds a column of unit boxes resting on the ground at x = 20.
std::vector<std::shared_ptr<FxEntity>> build_stack(FxScene& scene, int count,
                                                   std::vector<float>& start_y) {
    std::vector<std::shared_ptr<FxEntity>> boxes;
    for (int i = 0; i < count; ++i) {
        const float y = 2.0f + static_cast<float>(i);
        boxes.push_back(add_box(scene, "b" + std::to_string(i), 20.0f, y, 1.0f, 1.0f, 1.0f));
        start_y.push_back(y);
    }
    return boxes;
}

// ------------------------------------------------------------------ tall stacks

// Ten boxes hold at the default substep count. Measured: sink 0.033, tilt 1e-4.
void test_stack_of_ten_holds() {
    FxScene scene = make_scene();
    add_ground(scene);
    std::vector<float> start_y;
    auto boxes = build_stack(scene, 10, start_y);

    run(scene, kSettleSteps);
    StackError e = measure(boxes, start_y, 20.0f);

    require(e.max_lateral_drift < 0.05f, "a 10-box stack must not drift sideways, drifted " +
                                             std::to_string(e.max_lateral_drift));
    require(e.max_tilt < 0.05f,
            "a 10-box stack must not tilt, tilted " + std::to_string(e.max_tilt) + " rad");
    require(e.max_sink < 0.10f,
            "a 10-box stack must not sink into itself, sank " + std::to_string(e.max_sink));
}

// Fifteen is the tallest column that holds at the default 11 substeps. Measured sink 0.151.
void test_stack_of_fifteen_holds_at_default_substeps() {
    FxScene scene = make_scene();
    add_ground(scene);
    std::vector<float> start_y;
    auto boxes = build_stack(scene, 15, start_y);

    run(scene, kSettleSteps);
    StackError e = measure(boxes, start_y, 20.0f);

    require(e.max_lateral_drift < 0.20f,
            "a 15-box stack must stay a column, drifted " + std::to_string(e.max_lateral_drift));
    require(e.max_tilt < 0.20f,
            "a 15-box stack must not topple, tilted " + std::to_string(e.max_tilt) + " rad");
    require(e.max_sink < 0.30f,
            "a 15-box stack must not sink appreciably, sank " + std::to_string(e.max_sink));
}

// Twenty boxes need more substeps than the default. This documents the trade explicitly:
// the same scene that collapses at 11 substeps is stable at 22.
void test_stack_of_twenty_holds_with_more_substeps() {
    FxScene scene = make_scene();
    scene.set_substeps(22);
    add_ground(scene);
    std::vector<float> start_y;
    auto boxes = build_stack(scene, 20, start_y);

    run(scene, kSettleSteps);
    StackError e = measure(boxes, start_y, 20.0f);

    require(e.max_lateral_drift < 0.25f,
            "a 20-box stack at 22 substeps must stay a column, drifted " +
                std::to_string(e.max_lateral_drift));
    require(e.max_tilt < 0.25f, "a 20-box stack at 22 substeps must not topple, tilted " +
                                    std::to_string(e.max_tilt) + " rad");
    require(e.max_sink < 0.35f, "a 20-box stack at 22 substeps must not sink appreciably, sank " +
                                    std::to_string(e.max_sink));
}

// Raising the substep count must monotonically improve stack convergence. This is the knob a
// user reaches for when a stack misbehaves, so it needs to actually work.
void test_more_substeps_improve_stack_convergence() {
    auto settle_sink = [](size_t substeps) {
        FxScene scene = make_scene();
        scene.set_substeps(substeps);
        add_ground(scene);
        std::vector<float> start_y;
        auto boxes = build_stack(scene, 10, start_y);
        run(scene, kSettleSteps);
        return measure(boxes, start_y, 20.0f).max_sink;
    };

    const float sink_11 = settle_sink(11);
    const float sink_22 = settle_sink(22);

    require(sink_22 < sink_11, "doubling substeps must reduce stack sink, got " +
                                   std::to_string(sink_22) + " vs " + std::to_string(sink_11));
}

// A pyramid spreads load across several contacts per body rather than a single column.
void test_pyramid_holds() {
    FxScene scene = make_scene();
    add_ground(scene);

    std::vector<std::shared_ptr<FxEntity>> boxes;
    std::vector<float> start_y;
    const int base = 5;
    for (int row = 0; row < base; ++row) {
        const int in_row = base - row;
        const float y = 2.0f + static_cast<float>(row);
        for (int i = 0; i < in_row; ++i) {
            const float x = 20.0f - static_cast<float>(in_row - 1) * 0.5f + static_cast<float>(i);
            boxes.push_back(add_box(scene, "p" + std::to_string(row) + "_" + std::to_string(i), x,
                                    y, 1.0f, 1.0f, 1.0f));
            start_y.push_back(y);
        }
    }

    run(scene, kSettleSteps);

    float max_sink = 0.0f, max_tilt = 0.0f;
    for (size_t i = 0; i < boxes.size(); ++i) {
        max_sink = std::max(max_sink, start_y[i] - boxes[i]->pose.y());
        max_tilt = std::max(max_tilt, std::fabs(boxes[i]->pose.theta()));
    }
    require(max_tilt < 0.20f,
            "a 5-wide pyramid must not topple, tilted " + std::to_string(max_tilt) + " rad");
    require(max_sink < 0.30f,
            "a 5-wide pyramid must not sink appreciably, sank " + std::to_string(max_sink));
}

// ------------------------------------------------------------------ mass ratios

// A 100:1 body resting on a 1 kg box: the classic solver killer. Measured sink 0.073.
void test_hundred_to_one_mass_ratio_does_not_crush() {
    FxScene scene = make_scene();
    add_ground(scene);
    auto light = add_box(scene, "light", 20.0f, 2.0f, 1.0f, 1.0f, 1.0f);
    add_box(scene, "heavy", 20.0f, 3.0f, 1.0f, 1.0f, 100.0f);

    const float start_y = light->pose.y();
    run(scene, kSettleSteps);

    const float sank = start_y - light->pose.y();
    require(sank < 0.15f, "a 1 kg box under a 100 kg body must not be crushed, sank " +
                              std::to_string(sank) + " of its 1.0 height");
    require(light->pose.y() > 1.8f, "the light box must stay above the ground surface, y=" +
                                        std::to_string(light->pose.y()));
}

// A heavy body dropped on a light column topples it. What must NOT happen is bodies ending up
// inside one another: a scattered stack and an interpenetrating one both finish flat on the
// ground, so overlap is the thing to check, not height.
void test_heavy_body_topples_light_stack_without_interpenetration() {
    FxScene scene = make_scene();
    add_ground(scene);

    std::vector<std::shared_ptr<FxEntity>> bodies;
    for (int i = 0; i < 5; ++i)
        bodies.push_back(add_box(scene, "l" + std::to_string(i), 20.0f,
                                 2.0f + static_cast<float>(i), 1.0f, 1.0f, 1.0f));
    bodies.push_back(add_box(scene, "heavy", 20.0f, 7.0f, 1.0f, 1.0f, 100.0f));

    run(scene, kSettleSteps);

    for (size_t i = 0; i < bodies.size(); ++i) {
        require(bodies[i]->pose.y() > 1.4f, "no body may be pushed through the ground, " +
                                                bodies[i]->get_name() +
                                                " is at y=" + std::to_string(bodies[i]->pose.y()));
        for (size_t j = i + 1; j < bodies.size(); ++j) {
            const float dx = std::fabs(bodies[i]->pose.x() - bodies[j]->pose.x());
            const float dy = std::fabs(bodies[i]->pose.y() - bodies[j]->pose.y());
            require(dx > 0.85f || dy > 0.85f,
                    "unit boxes must not end up inside one another: " + bodies[i]->get_name() +
                        " and " + bodies[j]->get_name() + " are dx=" + std::to_string(dx) +
                        " dy=" + std::to_string(dy) + " apart");
        }
    }
}

// Ten to one is comfortably inside the envelope and must stay near-exact.
void test_ten_to_one_mass_ratio_is_solid() {
    FxScene scene = make_scene();
    add_ground(scene);
    auto light = add_box(scene, "light", 20.0f, 2.0f, 1.0f, 1.0f, 1.0f);
    add_box(scene, "heavy", 20.0f, 3.0f, 1.0f, 1.0f, 10.0f);

    const float start_y = light->pose.y();
    run(scene, kSettleSteps);

    require(start_y - light->pose.y() < 0.02f, "a 10:1 mass ratio must barely penetrate, sank " +
                                                   std::to_string(start_y - light->pose.y()));
}

// ------------------------------------------------------------ degenerate geometry

// Thin slivers are a classic source of bad SAT axes and jitter.
void test_thin_slivers_rest_stably() {
    for (float thickness : {0.1f, 0.02f}) {
        FxScene scene = make_scene();
        add_ground(scene);
        auto sliver = add_box(scene, "sliver", 20.0f, 3.0f, 4.0f, thickness, 1.0f);

        run(scene, kSettleSteps);

        const float expected_y = 1.5f + thickness * 0.5f;
        require(std::fabs(sliver->pose.y() - expected_y) < 0.05f,
                "a sliver of thickness " + std::to_string(thickness) +
                    " must come to rest on the ground surface, y=" +
                    std::to_string(sliver->pose.y()) + " expected " + std::to_string(expected_y));
        require(std::fabs(sliver->pose.theta()) < 0.05f,
                "a resting sliver must not tilt, tilted " + std::to_string(sliver->pose.theta()));
        require(sliver->velocity.head<2>().norm() < 0.01f,
                "a resting sliver must not jitter, |v|=" +
                    std::to_string(sliver->velocity.head<2>().norm()));
    }
}

// ------------------------------------------------------------ restitution chain

// Newton's cradle: a moving ball strikes a row of touching balls and the momentum should
// come out the far end, leaving the middle ones where they were.
void test_restitution_chain_transfers_momentum() {
    FxScene scene = make_scene();

    std::vector<std::shared_ptr<FxEntity>> balls;
    for (int i = 0; i < 5; ++i) {
        auto e = std::make_shared<FxEntity>("ball" + std::to_string(i));
        e->set_visual_geometry(FxVisualShape(0.5f));
        e->set_collision_geometry(FxCollisionShape(0.5f));
        e->set_init_pose(FxVec3f{18.0f + static_cast<float>(i), 2.0f, 0.0f});
        e->set_mass(1.0f);
        e->set_inertia();
        e->elasticity = 1.0f;
        e->static_friction = 0.0f;
        e->dynamic_friction = 0.0f;
        e->gravity_scale = 0.0f;
        scene.add_entity(e);
        balls.push_back(e);
    }

    // Pull the first ball back and send it in.
    balls[0]->pose.x() = 15.0f;
    balls[0]->prev_pose = balls[0]->pose;
    balls[0]->velocity = FxVec3f{6.0f, 0.0f, 0.0f};

    run(scene, 120);

    // The struck ball must carry the momentum away.
    require(balls[4]->pose.x() > 23.0f,
            "the far ball must be launched down-range, x=" + std::to_string(balls[4]->pose.x()));
    // The middle balls must stay put, near where they started.
    for (size_t i = 1; i <= 3; ++i) {
        const float expected = 18.0f + static_cast<float>(i);
        require(std::fabs(balls[i]->pose.x() - expected) < 0.5f,
                "middle ball " + std::to_string(i) +
                    " must stay in place, x=" + std::to_string(balls[i]->pose.x()) +
                    " expected near " + std::to_string(expected));
    }
    // The striker must have given up its momentum rather than passing through.
    require(balls[0]->velocity.x() < 1.0f,
            "the striking ball must transfer its momentum, still moving at " +
                std::to_string(balls[0]->velocity.x()));
    require(balls[0]->pose.x() < balls[1]->pose.x(),
            "the striking ball must not tunnel past the row");
}

// ------------------------------------------------------------ spinning bodies

// Total mechanical energy of a spinning box on an inelastic floor must never rise.
//
// This is the invariant that actually matters for a rotating body, and it is worth recording
// why height is not. A box spun fast enough legitimately vaults off a corner and ends up
// metres up: at 100 rad/s it carries ~838 J of rotational energy, while lifting a 1 kg box 7 m
// costs only 70 J. Judging the solver by how high the box goes therefore flags correct physics
// as a bug. Energy does not lie: with elasticity 0 every contact is dissipative, so the total
// may fall but must never climb.
void test_spinning_box_never_gains_energy() {
    auto mechanical_energy = [](const std::shared_ptr<FxEntity>& e, float reference_height) {
        const float w = e->velocity.theta();
        return 0.5f * e->mass() * e->velocity.head<2>().squaredNorm() +
               0.5f * e->inertia() * w * w + e->mass() * 10.0f * (e->pose.y() - reference_height);
    };

    for (float omega : {5.0f, 25.0f, 100.0f}) {
        FxScene scene = make_scene();
        add_ground(scene);
        auto spinner = add_box(scene, "spinner", 20.0f, 2.0f, 1.0f, 1.0f, 1.0f);
        spinner->set_init_velocity(FxVec3f{0.0f, 0.0f, omega});
        spinner->reset();

        const float initial = mechanical_energy(spinner, 1.5f);
        float peak = initial;
        for (int i = 0; i < kSettleSteps; ++i) {
            scene.step(kFrame);
            peak = std::max(peak, mechanical_energy(spinner, 1.5f));
        }

        // The tolerance absorbs float noise in the accumulation, nothing more.
        require(peak <= initial + 1e-3f * std::fabs(initial),
                "a box spinning at " + std::to_string(omega) +
                    " rad/s must never gain mechanical energy on an inelastic floor: started " +
                    std::to_string(initial) + " J, peaked at " + std::to_string(peak) + " J");
    }
}

// At modest spin the box settles in place rather than vaulting. This is the everyday case, a
// tumbling crate coming to rest, and it must stay well behaved.
void test_modest_spin_settles_in_place() {
    const float corner_pivot_height = 1.5f + std::sqrt(2.0f) * 0.5f; // 2.207

    for (float omega : {5.0f, 10.0f, 15.0f}) {
        FxScene scene = make_scene();
        add_ground(scene);
        auto spinner = add_box(scene, "spinner", 20.0f, 2.0f, 1.0f, 1.0f, 1.0f);
        spinner->set_init_velocity(FxVec3f{0.0f, 0.0f, omega});
        spinner->reset();

        float max_y = spinner->pose.y();
        for (int i = 0; i < kSettleSteps; ++i) {
            scene.step(kFrame);
            max_y = std::max(max_y, spinner->pose.y());
        }

        require(max_y < corner_pivot_height + 0.05f,
                "a box spinning at " + std::to_string(omega) +
                    " rad/s must not rise above corner-pivot height (" +
                    std::to_string(corner_pivot_height) + "), reached y=" + std::to_string(max_y));
        require(spinner->pose.y() > 1.4f, "a box spinning at " + std::to_string(omega) +
                                              " rad/s must stay on top of the ground, ended at y=" +
                                              std::to_string(spinner->pose.y()));
        require(std::fabs(spinner->velocity.theta()) < omega,
                "spin must decay under friction, not grow: started " + std::to_string(omega) +
                    " ended " + std::to_string(spinner->velocity.theta()));
    }
}

// ------------------------------------------------------------ kinematic platform

// A platform driven by prescribed velocity must carry a box on top of it, without the box
// sinking in or being left behind.
//
// Kinematic bodies must be driven by setting `velocity` and letting the integrator move them.
// Writing `pose` directly leaves `prev_pose` stale, so the velocity-derivation sweep in
// FxScene::step() reconstructs a bogus velocity from the teleport and friction cannot couple
// the rider to the surface — the rider is simply left behind and falls off the end.
void test_box_rides_kinematic_platform() {
    FxScene scene = make_scene();
    add_ground(scene);

    auto platform = add_box(scene, "platform", 20.0f, 3.0f, 6.0f, 0.5f, 1.0f);
    platform->set_mass(0.0f);
    platform->set_inertia(0.0f);
    platform->enable_external_forces(false);
    platform->gravity_scale = 0.0f;
    platform->static_friction = 0.9f;
    platform->dynamic_friction = 0.8f;

    auto rider = add_box(scene, "rider", 20.0f, 3.75f, 1.0f, 1.0f, 1.0f);
    rider->static_friction = 0.9f;
    rider->dynamic_friction = 0.8f;

    const float speed = 1.0f;
    const float start_x = platform->pose.x();
    for (int i = 0; i < 240; ++i) {
        platform->velocity.x() = speed; // prescribed motion, integrated like any other body
        platform->wake();
        rider->wake();
        scene.step(kFrame);
    }

    require(platform->pose.x() > start_x + 3.0f,
            "the prescribed velocity must actually move the platform, x=" +
                std::to_string(platform->pose.x()));
    require(rider->pose.y() > 3.6f,
            "the rider must stay on top of the platform, y=" + std::to_string(rider->pose.y()));
    require(std::fabs(rider->pose.x() - platform->pose.x()) < 0.5f,
            "friction must carry the rider along with the platform, rider x=" +
                std::to_string(rider->pose.x()) +
                " platform x=" + std::to_string(platform->pose.x()));
}

} // namespace

void run_adversarial_tests() {
    test_stack_of_ten_holds();
    test_stack_of_fifteen_holds_at_default_substeps();
    test_stack_of_twenty_holds_with_more_substeps();
    test_more_substeps_improve_stack_convergence();
    test_pyramid_holds();
    test_hundred_to_one_mass_ratio_does_not_crush();
    test_ten_to_one_mass_ratio_is_solid();
    test_heavy_body_topples_light_stack_without_interpenetration();
    test_thin_slivers_rest_stably();
    test_restitution_chain_transfers_momentum();
    test_spinning_box_never_gains_energy();
    test_modest_spin_settles_in_place();
    test_box_rides_kinematic_platform();
    std::cout << "Adversarial scene tests passed." << std::endl;
}
