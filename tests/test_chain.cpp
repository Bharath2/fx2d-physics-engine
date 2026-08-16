// Chain colliders: an open polyline of segments authored as one entity, for static level
// geometry a convex polygon approximates badly.

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

std::shared_ptr<FxEntity> add_chain(FxScene& scene, const std::string& name,
                                    const std::vector<FxVec2f>& points) {
    auto e = std::make_shared<FxEntity>(name);
    FxVec2fArray pts(points.size());
    for (size_t i = 0; i < points.size(); ++i)
        pts[i] = points[i];
    e->set_visual_geometry(FxVisualShape(FxShape::make_chain(pts)));
    e->set_collision_geometry(FxCollisionShape(FxShape::make_chain(pts)));
    e->set_init_pose(FxVec3f{0.0f, 0.0f, 0.0f});
    e->set_mass(0.0f);
    e->set_inertia(0.0f);
    e->gravity_scale = 0.0f;
    e->enable_external_forces(false);
    // Restitution mixes with max, and FxEntity::elasticity defaults to 1.0, so an unset
    // terrain body is a perfectly elastic floor.
    e->elasticity = 0.0f;
    e->static_friction = 0.6f;
    e->dynamic_friction = 0.5f;
    scene.add_entity(e);
    e->reset();
    return e;
}

// A chain is a distinct shape family with no interior.
void test_chain_shape_properties() {
    FxVec2fArray pts(4);
    pts[0] = {0.0f, 0.0f};
    pts[1] = {2.0f, 0.0f};
    pts[2] = {4.0f, 1.0f};
    pts[3] = {6.0f, 1.0f};
    FxShape chain = FxShape::make_chain(pts);

    require(chain.is_chain(), "make_chain must produce a chain");
    require(!chain.is_polygon() && !chain.is_capsule() && !chain.is_circle(),
            "a chain must not masquerade as another shape family");
    require(chain.segment_count() == 3,
            "4 points make 3 segments, got " + std::to_string(chain.segment_count()));
    require(chain.area() == 0.0f, "an open polyline encloses no area");
    require(chain.calc_inertia(5.0f) == 0.0f, "no area means no inertia");
    require(!chain.contains(FxVec2f{3.0f, 0.5f}),
            "a chain has no interior, so nothing is inside it");

    // Each segment comes back as a usable edge.
    const FxShape first = chain.segment(0);
    require(first.is_edge(), "a chain segment must be an edge");
    require(chain.segment(2).is_edge(), "the last segment too");
}

void test_chain_rejects_degenerate_input() {
    FxVec2fArray two(2);
    two[0] = {0.0f, 0.0f};
    two[1] = {1.0f, 0.0f};
    bool threw = false;
    try {
        FxShape::make_chain(two);
    } catch (const std::invalid_argument&) {
        threw = true;
    }
    require(threw, "a 2-point chain is just an edge and must be rejected");

    FxVec2fArray dup(3);
    dup[0] = {0.0f, 0.0f};
    dup[1] = {1.0f, 0.0f};
    dup[2] = {1.0f, 0.0f};
    threw = false;
    try {
        FxShape::make_chain(dup);
    } catch (const std::invalid_argument&) {
        threw = true;
    }
    require(threw, "repeated points make a zero-length segment and must be rejected");
}

// A box dropped on a flat chain rests on it, exactly as it would on an edge.
void test_box_rests_on_flat_chain() {
    FxScene scene = make_scene(FxVec2f{40.0f, 20.0f});
    add_chain(scene, "ground", {{2.0f, 5.0f}, {12.0f, 5.0f}, {22.0f, 5.0f}, {32.0f, 5.0f}});
    auto box = add_box(scene, "box", {12.0f, 8.0f}, {1.0f, 1.0f},
                       {.static_friction = 0.6f, .dynamic_friction = 0.5f});

    for (int i = 0; i < 240; ++i)
        scene.step(kFrame);

    require_near(box->pose.y(), 5.5f, 0.05f,
                 "the box must come to rest on the chain surface, y=" +
                     std::to_string(box->pose.y()));
    require(std::fabs(box->pose.x() - 12.0f) < 0.2f, "and must not slide off");
}

// The point of a chain: terrain a convex polygon cannot express. A ball dropped into a valley
// must be caught by the sloping segments, not fall between them.
void test_ball_settles_in_a_chain_valley() {
    FxScene scene = make_scene(FxVec2f{40.0f, 20.0f});
    add_chain(scene, "terrain", {{2.0f, 12.0f}, {10.0f, 4.0f}, {20.0f, 4.0f}, {28.0f, 12.0f}});
    auto ball = add_circle(scene, "ball", {12.0f, 14.0f}, 0.5f,
                           {.elasticity = 0.1f, .static_friction = 0.5f, .dynamic_friction = 0.4f});

    for (int i = 0; i < 600; ++i)
        scene.step(kFrame);

    require(ball->pose.y() > 4.0f,
            "the ball must be caught by the valley floor, not pass through, y=" +
                std::to_string(ball->pose.y()));
    require(ball->pose.x() > 9.0f && ball->pose.x() < 21.0f,
            "and settle inside the valley, x=" + std::to_string(ball->pose.x()));
    require(ball->velocity.head<2>().norm() < 0.5f, "and come to rest");
}

// A body sliding along the chain must cross the joints between segments without catching.
// The slope is gentle on purpose: a chain is zero-thickness and, like the edges it is built
// from, is excluded from speculative contacts, so a fast enough body tunnels through it.
void test_body_crosses_segment_joints() {
    FxScene scene = make_scene(FxVec2f{60.0f, 20.0f});
    add_chain(scene, "ramp", {{2.0f, 9.0f}, {14.0f, 8.0f}, {26.0f, 7.2f}, {40.0f, 6.8f}});
    auto ball =
        add_circle(scene, "ball", {4.0f, 9.3f}, 0.4f,
                   {.elasticity = 0.0f, .static_friction = 0.02f, .dynamic_friction = 0.02f});

    const float x0 = ball->pose.x();
    for (int i = 0; i < 400; ++i)
        scene.step(kFrame);

    require(ball->pose.x() > x0 + 8.0f,
            "the ball must slide past at least one segment joint, reached x=" +
                std::to_string(ball->pose.x()));
    require(ball->pose.y() > 6.0f,
            "and stay on top of the chain, y=" + std::to_string(ball->pose.y()));
}

// Chain against chain, and chain against edge, produce nothing: neither has volume to resolve.
void test_chain_against_chain_is_skipped() {
    FxScene scene = make_scene(FxVec2f{40.0f, 20.0f}, 0.0f);
    add_chain(scene, "a", {{2.0f, 10.0f}, {12.0f, 10.0f}, {22.0f, 10.0f}});
    add_chain(scene, "b", {{2.0f, 10.0f}, {12.0f, 10.0f}, {22.0f, 10.0f}});

    scene.step(kFrame);
    require(scene.contacts().empty(), "two overlapping chains must produce no contacts, got " +
                                          std::to_string(scene.contacts().size()));
}

// Chains are authorable from YAML.
void test_chain_from_yaml() {
    FxScene scene = FxYAML::buildScene(R"yaml(
scene:
  size: [40, 20]
  gravity: [0, -10]
entities:
  terrain:
    pose: [0, 0, 0]
    physics:
      mass: 0.0
      elasticity: 0.0
      gravity_scale: 0.0
      external_forces_enabled: false
      static_friction: 0.6
      dynamic_friction: 0.5
    visual:
      geometry:
        chain: [[2, 5], [12, 5], [22, 6], [32, 6]]
    collision:
      geometry:
        chain: [[2, 5], [12, 5], [22, 6], [32, 6]]
  crate:
    pose: [8, 9, 0]
    physics:
      mass: 1.0
      elasticity: 0.0
      static_friction: 0.6
      dynamic_friction: 0.5
    visual:
      geometry:
        rectangle: [1.0, 1.0]
    collision:
      geometry:
        rectangle: [1.0, 1.0]
)yaml");

    auto terrain = scene.get_entity("terrain");
    require(terrain != nullptr, "the chain entity must be built");
    require(terrain->collision_geometry()->is_chain(), "and must carry a chain collider");
    require(terrain->collision_geometry()->segment_count() == 3,
            "4 authored points make 3 segments");

    auto crate = scene.get_entity("crate");
    for (int i = 0; i < 240; ++i)
        scene.step(kFrame);
    require_near(crate->pose.y(), 5.5f, 0.1f,
                 "the crate must rest on the YAML-authored chain, y=" +
                     std::to_string(crate->pose.y()));
}

// A chain with too few points is rejected at parse time rather than silently accepted.
void test_yaml_chain_rejects_short_input() {
    bool threw = false;
    try {
        FxYAML::buildScene(R"yaml(
scene:
  size: [40, 20]
entities:
  bad:
    collision:
      geometry:
        chain: [[0, 0], [1, 0]]
)yaml");
    } catch (const std::runtime_error&) {
        threw = true;
    }
    require(threw, "a 2-point chain in YAML must be rejected");
}

} // namespace

void run_chain_tests() {
    test_chain_shape_properties();
    test_chain_rejects_degenerate_input();
    test_box_rests_on_flat_chain();
    test_ball_settles_in_a_chain_valley();
    test_body_crosses_segment_joints();
    test_chain_against_chain_is_skipped();
    test_chain_from_yaml();
    test_yaml_chain_rejects_short_input();
    std::cout << "Chain collider tests passed." << std::endl;
}
