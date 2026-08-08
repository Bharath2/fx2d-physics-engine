#include "Fx2D/Entity.h"
#include "Fx2D/Math.h"
#include "Fx2D/Solver.h"
#include "Fx2D/YamlUtils.h"

#include <cmath>
#include <iostream>
#include <memory>
#include <stdexcept>

namespace {

void require(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}

bool approx(float a, float b, float eps = 1e-3f) { return std::fabs(a - b) <= eps; }

// Build an entity with an edge collision shape; a zero-dt step syncs the AABB.
std::shared_ptr<FxEntity> make_edge(const std::string& name,
                                    const FxVec2f& a, const FxVec2f& b,
                                    float wx = 0.0f, float wy = 0.0f) {
    auto e = std::make_shared<FxEntity>(name);
    e->set_mass(1.0f);
    e->set_inertia(1.0f);
    e->set_init_pose(FxVec3f{wx, wy, 0.0f});
    e->set_collision_geometry(FxCollisionShape(a, b));
    e->step(FxVec2f{0.0f, 0.0f}, 0.0);
    return e;
}

std::shared_ptr<FxEntity> make_circle(const std::string& name, float x, float y, float r) {
    auto e = std::make_shared<FxEntity>(name);
    e->set_mass(1.0f);
    e->set_inertia(1.0f);
    e->set_init_pose(FxVec3f{x, y, 0.0f});
    e->set_collision_geometry(FxCollisionShape(r));
    e->step(FxVec2f{0.0f, 0.0f}, 0.0);
    return e;
}

std::shared_ptr<FxEntity> make_box(const std::string& name, float x, float y,
                                   float w = 1.0f, float h = 1.0f) {
    auto e = std::make_shared<FxEntity>(name);
    e->set_mass(1.0f);
    e->set_inertia(1.0f);
    e->set_init_pose(FxVec3f{x, y, 0.0f});
    e->set_collision_geometry(FxCollisionShape(FxVec2f{w, h}));
    e->step(FxVec2f{0.0f, 0.0f}, 0.0);
    return e;
}

// An edge is a zero-skin capsule: no area, no inertia, exact-endpoint AABB.
void test_edge_shape_properties() {
    FxShape edge(FxVec2f{-1.0f, 0.0f}, FxVec2f{1.0f, 0.0f});
    require(edge.is_edge(), "is_edge() must be true for a zero-skin segment");
    require(edge.is_capsule(), "an edge is stored as a capsule");
    require(approx(edge.skin_radius(), 0.0f), "edge skin radius must be 0");
    require(edge.area() == 0.0f, "area() must be 0 for an edge");
    require(edge.calc_inertia(2.0f) == 0.0f, "calc_inertia(mass) must be 0 for an edge");
}

void test_capsule_is_not_an_edge() {
    FxShape cap(2.0f, 0.3f);
    require(!cap.is_edge(), "a capsule with skin must not report as an edge");
}

void test_edge_aabb() {
    FxShape edge(FxVec2f{-1.0f, 0.0f}, FxVec2f{1.0f, 0.0f});
    auto bb = edge.set_world_pose(FxVec3f{0.0f, 5.0f, 0.0f});
    require(approx(bb[0], -1.0f, 1e-4f), "AABB minX must be -1");
    require(approx(bb[1], 5.0f, 1e-4f), "AABB minY must be 5");
    require(approx(bb[2], 1.0f, 1e-4f), "AABB maxX must be 1");
    require(approx(bb[3], 5.0f, 1e-4f), "AABB maxY must be 5");
}

void test_edge_degenerate_throws() {
    bool threw = false;
    try {
        FxShape bad(FxVec2f{0.0f, 0.0f}, FxVec2f{0.0f, 0.0f});
    } catch (const std::invalid_argument&) {
        threw = true;
    }
    require(threw, "zero-length edge ctor must throw std::invalid_argument");
}

// Edge (-2,0)-(2,0) vs circle at (0,0.4) r=0.5: closest segment point is (0,0), pen = 0.5 - 0.4.
void test_edge_circle_contact() {
    auto edge = make_edge("ecc_edge", FxVec2f{-2.0f, 0.0f}, FxVec2f{2.0f, 0.0f});
    auto circle = make_circle("ecc_circ", 0.0f, 0.4f, 0.5f);

    FxContact c = FxSolver::collision_check(edge, circle);
    require(c.is_valid(false), "edge-circle contact must be valid");
    require(approx(c.penetration_depth, 0.1f), "edge-circle penetration must be ~0.1");
    require(std::fabs(c.normal.x()) <= 1e-3f, "edge-circle normal.x must be ~0");
    require(approx(std::fabs(c.normal.y()), 1.0f), "edge-circle normal must be vertical");
    require(c.count >= 1, "edge-circle contact count must be >= 1");
}

void test_edge_circle_separated() {
    auto edge = make_edge("ecs_edge", FxVec2f{-2.0f, 0.0f}, FxVec2f{2.0f, 0.0f});
    auto circle = make_circle("ecs_circ", 0.0f, 1.0f, 0.5f);

    FxContact c = FxSolver::collision_check(edge, circle);
    require(!c.is_valid(false), "separated edge-circle must return an invalid contact");
}

// 1x1 box centred at (0,0.45): bottom face at y=-0.05, sunk 0.05 through the edge at y=0.
void test_edge_box_contact() {
    auto edge = make_edge("ebc_edge", FxVec2f{-2.0f, 0.0f}, FxVec2f{2.0f, 0.0f});
    auto box = make_box("ebc_box", 0.0f, 0.45f);

    FxContact c = FxSolver::collision_check(edge, box);
    require(c.is_valid(false), "edge-box contact must be valid");
    require(std::fabs(c.normal.x()) <= 1e-2f, "edge-box normal.x must be ~0");
    require(approx(std::fabs(c.normal.y()), 1.0f, 1e-2f), "edge-box normal must be vertical");
    require(approx(c.penetration_depth, 0.05f, 1e-2f), "edge-box penetration must be ~0.05");
    require(c.count >= 1, "edge-box contact count must be >= 1");
}

// The generic capsule reduction misses this case: the segment lies inside the box.
void test_edge_through_box_contact() {
    auto edge = make_edge("etb_edge", FxVec2f{-2.0f, 0.0f}, FxVec2f{2.0f, 0.0f});
    auto box = make_box("etb_box", 0.0f, 0.0f, 1.0f, 1.0f);

    FxContact c = FxSolver::collision_check(edge, box);
    require(c.is_valid(false), "an edge crossing a box interior must report contact");
    require(c.penetration_depth > 0.0f, "penetration depth must be positive");
}

void test_edge_box_offspan_no_contact() {
    auto edge = make_edge("eos_edge", FxVec2f{-2.0f, 0.0f}, FxVec2f{2.0f, 0.0f});
    auto box = make_box("eos_box", 5.0f, 0.45f);

    FxContact c = FxSolver::collision_check(edge, box);
    require(!c.is_valid(false), "a box outside the segment span must not contact");
}

// Two zero-thickness segments have no volume to resolve.
void test_edge_edge_no_contact() {
    auto e1 = make_edge("ee1", FxVec2f{-2.0f, 0.0f}, FxVec2f{2.0f, 0.0f});
    auto e2 = make_edge("ee2", FxVec2f{-1.0f, 0.0f}, FxVec2f{1.0f, 0.0f});

    FxContact c = FxSolver::collision_check(e1, e2);
    require(!c.is_valid(false), "edge-edge must return an invalid contact");
}

// A rounded box resting on an edge must account for the polygon's skin.
void test_edge_vs_rounded_box() {
    auto edge = make_edge("erb_edge", FxVec2f{-2.0f, 0.0f}, FxVec2f{2.0f, 0.0f});
    auto box = std::make_shared<FxEntity>("erb_box");
    box->set_mass(1.0f);
    box->set_inertia(1.0f);
    box->set_init_pose(FxVec3f{0.0f, 0.7f, 0.0f});
    box->set_collision_geometry(FxCollisionShape(FxVec2f{1.0f, 1.0f}, 0.25f));
    box->step(FxVec2f{0.0f, 0.0f}, 0.0);

    // Box vertices reach y = 0.2; the 0.25 skin sinks 0.05 below the edge.
    FxContact c = FxSolver::collision_check(edge, box);
    require(c.is_valid(false), "rounded box overlapping an edge must contact");
    require(approx(c.penetration_depth, 0.05f, 1e-2f), "rounded-box penetration must be ~0.05");
}

// YAML nodes are built directly to avoid the buildShape(string) filesystem-path probe.
void test_edge_yaml_load() {
    YAML::Node cfg;
    YAML::Node a, b;
    a.push_back(-2.0f); a.push_back(0.0f);
    b.push_back(2.0f);  b.push_back(0.0f);
    cfg["geometry"]["edge"].push_back(a);
    cfg["geometry"]["edge"].push_back(b);
    FxShape shape = FxYAML::buildShape(cfg);
    require(shape.is_edge(), "YAML 'edge:' must build an edge");
    require(approx(shape.skin_radius(), 0.0f), "YAML edge must have zero skin");
}

void test_edge_yaml_bad_arity() {
    YAML::Node cfg;
    YAML::Node a;
    a.push_back(0.0f); a.push_back(0.0f);
    cfg["geometry"]["edge"].push_back(a);
    bool threw = false;
    try {
        FxYAML::buildShape(cfg);
    } catch (const std::runtime_error&) {
        threw = true;
    }
    require(threw, "single-point edge YAML must throw std::runtime_error");
}

} // namespace

void run_edge_tests() {
    test_edge_shape_properties();
    test_capsule_is_not_an_edge();
    test_edge_aabb();
    test_edge_degenerate_throws();
    test_edge_circle_contact();
    test_edge_circle_separated();
    test_edge_box_contact();
    test_edge_through_box_contact();
    test_edge_box_offspan_no_contact();
    test_edge_edge_no_contact();
    test_edge_vs_rounded_box();
    test_edge_yaml_load();
    test_edge_yaml_bad_arity();
    std::cout << "Edge collision tests passed." << std::endl;
}
