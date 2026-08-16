// Spatial queries: ray casts, overlap and point queries.

#include "Fx2D/Physics.h"

#include "test_harness.h"
#include "test_scene_builders.h"

#include <cmath>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

namespace {

std::shared_ptr<FxEntity> add_box(FxScene& scene, const std::string& name, float cx, float cy,
                                  float w, float h) {
    return ::add_box(scene, name, {cx, cy}, {w, h});
}

std::shared_ptr<FxEntity> add_circle(FxScene& scene, const std::string& name, float cx, float cy,
                                     float r) {
    return ::add_circle(scene, name, {cx, cy}, r);
}

bool contains(const std::vector<std::shared_ptr<FxEntity>>& v, const std::string& name) {
    for (const auto& e : v)
        if (e && e->get_name() == name) return true;
    return false;
}

FxScene make_scene() {
    return ::make_scene(FxVec2f{40.0f, 40.0f}, 0.0f);
}

// A ray down a clear lane hits nothing.
void test_ray_through_empty_space_misses() {
    FxScene scene = make_scene();
    add_box(scene, "wall", 10.0f, 20.0f, 1.0f, 1.0f);

    FxRayHit hit;
    const bool found = scene.raycast(FxVec2f{0.0f, 5.0f}, FxVec2f{1.0f, 0.0f}, 30.0f, hit);
    require(!found, "a ray that passes well below the wall must miss");
    require(!hit.hit(), "a missed raycast must leave the hit empty");
}

// A ray straight at a box reports the near face, at the right distance, with an outward normal.
void test_ray_hits_box_face() {
    FxScene scene = make_scene();
    add_box(scene, "wall", 10.0f, 5.0f, 2.0f, 4.0f); // spans x 9..11

    FxRayHit hit;
    const bool found = scene.raycast(FxVec2f{0.0f, 5.0f}, FxVec2f{1.0f, 0.0f}, 30.0f, hit);

    require(found, "a ray fired at the wall must hit it");
    require(hit.entity && hit.entity->get_name() == "wall", "the hit must name the wall");
    require_near(hit.distance, 9.0f, 1e-2f, "the hit distance must be the near face at x=9");
    require_near(hit.point.x(), 9.0f, 1e-2f, "the hit point must sit on the near face");
    require_near(hit.point.y(), 5.0f, 1e-2f, "the hit point must keep the ray's height");
    require(hit.normal.x() < -0.9f,
            "the normal must face back along the ray, got x=" + std::to_string(hit.normal.x()));
}

// Rays respect max_distance.
void test_ray_stops_at_max_distance() {
    FxScene scene = make_scene();
    add_box(scene, "wall", 20.0f, 5.0f, 2.0f, 4.0f); // near face at x=19

    FxRayHit hit;
    require(!scene.raycast(FxVec2f{0.0f, 5.0f}, FxVec2f{1.0f, 0.0f}, 10.0f, hit),
            "a 10-unit ray must not reach a wall 19 units away");
    require(scene.raycast(FxVec2f{0.0f, 5.0f}, FxVec2f{1.0f, 0.0f}, 25.0f, hit),
            "a 25-unit ray must reach it");
}

// A circle is hit at its rim, not its centre.
void test_ray_hits_circle_rim() {
    FxScene scene = make_scene();
    add_circle(scene, "ball", 10.0f, 5.0f, 1.5f);

    FxRayHit hit;
    require(scene.raycast(FxVec2f{0.0f, 5.0f}, FxVec2f{1.0f, 0.0f}, 30.0f, hit),
            "the ray must hit the ball");
    require_near(hit.distance, 8.5f, 1e-2f, "the hit must be at the rim, 1.5 before the centre");
    require(hit.normal.x() < -0.9f, "the rim normal must face the ray");
}

// Direction need not be normalised, and the reported distance is in world units regardless.
void test_ray_direction_need_not_be_unit_length() {
    FxScene scene = make_scene();
    add_box(scene, "wall", 10.0f, 5.0f, 2.0f, 4.0f);

    FxRayHit unit, scaled;
    require(scene.raycast(FxVec2f{0.0f, 5.0f}, FxVec2f{1.0f, 0.0f}, 30.0f, unit), "unit ray hits");
    require(scene.raycast(FxVec2f{0.0f, 5.0f}, FxVec2f{7.3f, 0.0f}, 30.0f, scaled),
            "scaled ray hits");
    require_near(scaled.distance, unit.distance, 1e-3f,
                 "distance must be in world units, not multiples of the direction vector");
}

// raycast_all reports every body along the line, nearest first.
void test_raycast_all_is_sorted_by_distance() {
    FxScene scene = make_scene();
    add_box(scene, "far", 20.0f, 5.0f, 1.0f, 4.0f);
    add_box(scene, "near", 8.0f, 5.0f, 1.0f, 4.0f);
    add_box(scene, "middle", 14.0f, 5.0f, 1.0f, 4.0f);
    add_box(scene, "off_lane", 14.0f, 30.0f, 1.0f, 4.0f);

    std::vector<FxRayHit> hits;
    scene.raycast_all(FxVec2f{0.0f, 5.0f}, FxVec2f{1.0f, 0.0f}, 40.0f, hits);

    require(hits.size() == 3, "the ray must find exactly the three bodies in its lane, found " +
                                  std::to_string(hits.size()));
    require(hits[0].entity->get_name() == "near", "nearest must come first");
    require(hits[1].entity->get_name() == "middle", "then the middle one");
    require(hits[2].entity->get_name() == "far", "then the far one");
    require(hits[0].distance < hits[1].distance && hits[1].distance < hits[2].distance,
            "distances must increase");
}

// A ray starting inside a body reports it immediately.
void test_ray_from_inside_reports_that_body() {
    FxScene scene = make_scene();
    add_box(scene, "room", 10.0f, 10.0f, 6.0f, 6.0f);

    FxRayHit hit;
    require(scene.raycast(FxVec2f{10.0f, 10.0f}, FxVec2f{1.0f, 0.0f}, 20.0f, hit),
            "a ray starting inside a body must report it");
    require(hit.entity->get_name() == "room", "and name that body");
    require(hit.distance < 1e-3f, "at zero distance, got " + std::to_string(hit.distance));
}

// Disabled bodies and bodies without collision geometry are invisible to queries.
void test_queries_skip_disabled_and_geometryless() {
    FxScene scene = make_scene();
    auto disabled = add_box(scene, "disabled", 10.0f, 5.0f, 2.0f, 4.0f);
    disabled->enabled = false;
    auto ghost = add_box(scene, "ghost", 14.0f, 5.0f, 2.0f, 4.0f);
    ghost->del_collision_geometry();

    FxRayHit hit;
    require(!scene.raycast(FxVec2f{0.0f, 5.0f}, FxVec2f{1.0f, 0.0f}, 30.0f, hit),
            "neither a disabled body nor one without collision geometry may be hit");

    std::vector<std::shared_ptr<FxEntity>> found;
    scene.overlap_circle(FxVec2f{10.0f, 5.0f}, 2.0f, found);
    require(found.empty(), "overlap must skip them too");
}

// Overlap finds what it covers and nothing else.
void test_overlap_circle_selects_by_geometry() {
    FxScene scene = make_scene();
    add_box(scene, "inside", 10.0f, 10.0f, 1.0f, 1.0f);
    add_box(scene, "touching", 12.0f, 10.0f, 1.0f, 1.0f); // spans x 11.5..12.5
    add_box(scene, "outside", 20.0f, 10.0f, 1.0f, 1.0f);

    std::vector<std::shared_ptr<FxEntity>> found;
    scene.overlap_circle(FxVec2f{10.0f, 10.0f}, 2.0f, found); // reaches x=12

    require(contains(found, "inside"), "a body at the centre must be found");
    require(contains(found, "touching"), "a body the circle reaches must be found");
    require(!contains(found, "outside"), "a body well clear must not be found");
}

void test_overlap_box_selects_by_geometry() {
    FxScene scene = make_scene();
    add_box(scene, "in", 10.0f, 10.0f, 1.0f, 1.0f);
    add_box(scene, "out", 18.0f, 10.0f, 1.0f, 1.0f);

    std::vector<std::shared_ptr<FxEntity>> found;
    scene.overlap_box(FxVec2f{10.0f, 10.0f}, FxVec2f{4.0f, 4.0f}, found);

    require(contains(found, "in"), "the covered body must be found");
    require(!contains(found, "out"), "the distant body must not be");
}

// Point queries answer "what is under the cursor".
void test_point_query_picks_the_body_under_it() {
    FxScene scene = make_scene();
    add_box(scene, "target", 10.0f, 10.0f, 2.0f, 2.0f); // spans 9..11 in both axes

    auto picked = scene.entity_at_point(FxVec2f{10.4f, 9.6f});
    require(picked != nullptr, "a point inside the box must pick it");
    require(picked->get_name() == "target", "and name it");

    require(scene.entity_at_point(FxVec2f{14.0f, 10.0f}) == nullptr,
            "a point outside every body must pick nothing");
    require(scene.entity_at_point(FxVec2f{11.4f, 10.0f}) == nullptr,
            "a point just past the edge must not pick the box");
}

// A rotated body must be queried in its rotated position, not its axis-aligned box.
void test_queries_respect_rotation() {
    FxScene scene = make_scene();
    auto plank = add_box(scene, "plank", 10.0f, 10.0f, 4.0f, 0.4f);
    plank->pose.theta() = 1.5708f; // stand it on end
    plank->collision_geometry()->set_world_pose(plank->pose);

    // Upright, it now covers y 8..12 and only x 9.8..10.2.
    require(scene.entity_at_point(FxVec2f{10.0f, 11.5f}) != nullptr,
            "the rotated plank must be picked along its new long axis");
    require(scene.entity_at_point(FxVec2f{11.5f, 10.0f}) == nullptr,
            "and must not be picked where it no longer reaches");
}

// Queries agree with the simulation: whatever overlap reports is what the solver contacts.
void test_overlap_agrees_with_contacts() {
    FxScene scene = make_scene();
    scene.set_gravity(FxVec2f{0.0f, 0.0f});
    auto a = add_box(scene, "a", 10.0f, 10.0f, 1.0f, 1.0f);
    add_box(scene, "b", 10.8f, 10.0f, 1.0f, 1.0f); // deliberately overlapping a
    a->enable_external_forces(false);

    scene.step(1.0 / 60.0);
    require(!scene.contacts().empty(), "the two boxes must be in contact");

    // Query a volume that comfortably covers both, so the assertion is about the query seeing
    // what the solver sees rather than about exactly where the solver pushed them this step.
    std::vector<std::shared_ptr<FxEntity>> found;
    scene.overlap_box(FxVec2f{10.4f, 10.0f}, FxVec2f{4.0f, 4.0f}, found);
    require(contains(found, "a") && contains(found, "b"),
            "an overlap query covering both bodies must report both, as the solver contacts them");

    // And a query nowhere near them must report neither.
    scene.overlap_box(FxVec2f{25.0f, 25.0f}, FxVec2f{2.0f, 2.0f}, found);
    require(found.empty(), "a query far from every body must come back empty");
}

// FxShape::contains covers every shape family, skin included.
void test_shape_contains_point() {
    FxShape circle(1.0f);
    circle.set_world_pose(FxVec3f{5.0f, 5.0f, 0.0f});
    require(circle.contains(FxVec2f{5.0f, 5.0f}), "a circle must contain its centre");
    require(circle.contains(FxVec2f{5.9f, 5.0f}), "and a point inside its radius");
    require(!circle.contains(FxVec2f{6.1f, 5.0f}), "but not one outside it");

    FxShape box(FxVec2f{2.0f, 4.0f}); // spans 4..6 by 3..7 once placed
    box.set_world_pose(FxVec3f{5.0f, 5.0f, 0.0f});
    require(box.contains(FxVec2f{5.9f, 6.9f}), "a box must contain a point just inside a corner");
    require(!box.contains(FxVec2f{6.1f, 5.0f}), "and reject one past its face");

    FxShape capsule(2.0f, 0.5f); // segment along local x, 0.5 skin
    capsule.set_world_pose(FxVec3f{5.0f, 5.0f, 0.0f});
    require(capsule.contains(FxVec2f{5.0f, 5.4f}),
            "a capsule must contain a point within its skin");
    require(!capsule.contains(FxVec2f{5.0f, 5.6f}), "and reject one beyond it");

    // Rotation is respected, since the test runs on world vertices.
    FxShape plank(FxVec2f{4.0f, 0.4f});
    plank.set_world_pose(FxVec3f{5.0f, 5.0f, 1.5708f}); // stood on end
    require(plank.contains(FxVec2f{5.0f, 6.5f}),
            "a rotated plank must contain points along its new axis");
    require(!plank.contains(FxVec2f{6.5f, 5.0f}), "and not where it no longer reaches");
}

} // namespace

void run_query_tests() {
    test_shape_contains_point();
    test_ray_through_empty_space_misses();
    test_ray_hits_box_face();
    test_ray_stops_at_max_distance();
    test_ray_hits_circle_rim();
    test_ray_direction_need_not_be_unit_length();
    test_raycast_all_is_sorted_by_distance();
    test_ray_from_inside_reports_that_body();
    test_queries_skip_disabled_and_geometryless();
    test_overlap_circle_selects_by_geometry();
    test_overlap_box_selects_by_geometry();
    test_point_query_picks_the_body_under_it();
    test_queries_respect_rotation();
    test_overlap_agrees_with_contacts();
    std::cout << "Spatial query tests passed." << std::endl;
}
