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

// Build an entity with an edge collision shape; zero-dt step syncs the AABB.
std::shared_ptr<FxEntity> make_edge_entity(const std::string& name,
                                           const FxVec2f& a, const FxVec2f& b,
                                           float wx = 0.f, float wy = 0.f) {
    auto e = std::make_shared<FxEntity>(name);
    e->set_mass(1.0f);
    e->set_inertia(1.0f);
    e->set_init_pose(FxVec3f{wx, wy, 0.0f});
    e->set_collision_geometry(FxCollisionShape(a, b));
    e->step(FxVec2f{0.0f, 0.0f}, 0.0);
    return e;
}

std::shared_ptr<FxEntity> make_circle_entity(const std::string& name,
                                             float x, float y, float r) {
    auto e = std::make_shared<FxEntity>(name);
    e->set_mass(1.0f);
    e->set_inertia(1.0f);
    e->set_init_pose(FxVec3f{x, y, 0.0f});
    e->set_collision_geometry(FxCollisionShape(r));
    e->step(FxVec2f{0.0f, 0.0f}, 0.0);
    return e;
}

std::shared_ptr<FxEntity> make_box_entity(const std::string& name,
                                          float x, float y, float w = 1.f, float h = 1.f) {
    auto e = std::make_shared<FxEntity>(name);
    e->set_mass(1.0f);
    e->set_inertia(1.0f);
    e->set_init_pose(FxVec3f{x, y, 0.0f});
    e->set_collision_geometry(FxCollisionShape(FxVec2f{w, h}));
    e->step(FxVec2f{0.0f, 0.0f}, 0.0);
    return e;
}

// ─────────────────────────────────────────────────────────────────────────────
// T1: FxShape edge primitive (Math.h)
// ─────────────────────────────────────────────────────────────────────────────

void test_edge_shape_properties() {
    FxShape edge(FxVec2f{-1.f, 0.f}, FxVec2f{1.f, 0.f});
    require(edge.is_edge(),                              "is_edge() must be true");
    require(edge.shape_type() == FxShapeType::Edge,     "shape_type() must be FxShapeType::Edge");
    require(edge.area() == 0.0f,                        "area() must be 0 for an edge");
    require(edge.calc_inertia(2.0f) == 0.0f,            "calc_inertia(mass) must be 0 for an edge");
}

void test_edge_aabb() {
    FxShape edge(FxVec2f{-1.f, 0.f}, FxVec2f{1.f, 0.f});
    auto bb = edge.set_world_pose(FxVec3f{0.f, 5.f, 0.f});
    require(std::fabs(bb[0] - (-1.f)) <= 1e-4f, "AABB minX must be -1");
    require(std::fabs(bb[1] - 5.f)    <= 1e-4f, "AABB minY must be 5");
    require(std::fabs(bb[2] - 1.f)    <= 1e-4f, "AABB maxX must be 1");
    require(std::fabs(bb[3] - 5.f)    <= 1e-4f, "AABB maxY must be 5");
}

void test_edge_degenerate_throws() {
    bool threw = false;
    try {
        FxShape bad(FxVec2f{0.f, 0.f}, FxVec2f{0.f, 0.f});
    } catch (const std::invalid_argument&) {
        threw = true;
    }
    require(threw, "zero-length edge ctor must throw std::invalid_argument");
}

// ─────────────────────────────────────────────────────────────────────────────
// T2: narrow-phase dispatch (Collisions.cpp)
// ─────────────────────────────────────────────────────────────────────────────

void test_edge_circle_contact() {
    // Horizontal edge (-2,0)-(2,0) vs circle at (0,0.4) r=0.5.
    // Closest point on segment to circle centre is (0,0); dist=0.4; pen=0.5-0.4=0.1.
    auto edge   = make_edge_entity("ecc_edge",  FxVec2f{-2.f, 0.f}, FxVec2f{2.f, 0.f});
    auto circle = make_circle_entity("ecc_circ", 0.f, 0.4f, 0.5f);

    FxContact c = FxSolver::collision_check(edge, circle);
    require(c.is_valid(false),                                   "edge-circle contact must be valid");
    require(std::fabs(c.penetration_depth - 0.1f)    <= 1e-3f,  "edge-circle pen depth must be ~0.1");
    require(std::fabs(c.normal.x())                  <= 1e-3f,  "edge-circle normal.x must be ~0");
    require(std::fabs(std::fabs(c.normal.y()) - 1.f) <= 1e-3f,  "edge-circle normal.y must be ~±1");
    require(c.count >= 1,                                        "edge-circle contact count must be >= 1");
}

void test_edge_circle_separated() {
    // Circle at (0,1.0) r=0.5: surface at y=0.5, above the edge at y=0 — separated.
    auto edge   = make_edge_entity("ecs_edge",  FxVec2f{-2.f, 0.f}, FxVec2f{2.f, 0.f});
    auto circle = make_circle_entity("ecs_circ", 0.f, 1.0f, 0.5f);

    FxContact c = FxSolver::collision_check(edge, circle);
    require(!c.is_valid(false), "separated edge-circle must return invalid contact");
}

void test_edge_box_contact() {
    // 1×1 box centred at (0,0.45): bottom face at y=-0.05, sunk 0.05 into the edge at y=0.
    auto edge = make_edge_entity("ebc_edge", FxVec2f{-2.f, 0.f}, FxVec2f{2.f, 0.f});
    auto box  = make_box_entity("ebc_box",  0.f, 0.45f, 1.f, 1.f);

    FxContact c = FxSolver::collision_check(edge, box);
    require(c.is_valid(false),                                   "edge-box contact must be valid");
    require(std::fabs(c.normal.x())                  <= 1e-2f,  "edge-box normal.x must be ~0");
    require(std::fabs(std::fabs(c.normal.y()) - 1.f) <= 1e-2f,  "edge-box normal.y must be ~±1");
    require(std::fabs(c.penetration_depth - 0.05f)   <= 1e-2f,  "edge-box pen depth must be ~0.05");
    require(c.count >= 1,                                        "edge-box contact count must be >= 1");
}

void test_edge_box_offspan_no_contact() {
    // Box at (5,0.45) lies entirely past the segment span [-2,2] — no contact.
    auto edge = make_edge_entity("eos_edge", FxVec2f{-2.f, 0.f}, FxVec2f{2.f, 0.f});
    auto box  = make_box_entity("eos_box",  5.f, 0.45f, 1.f, 1.f);

    FxContact c = FxSolver::collision_check(edge, box);
    require(!c.is_valid(false), "box outside segment span must return invalid contact");
}

void test_edge_edge_no_contact() {
    // Two overlapping horizontal edges — no contact by design (D4).
    auto e1 = make_edge_entity("ee1", FxVec2f{-2.f, 0.f}, FxVec2f{2.f, 0.f});
    auto e2 = make_edge_entity("ee2", FxVec2f{-1.f, 0.f}, FxVec2f{1.f, 0.f});

    FxContact c = FxSolver::collision_check(e1, e2);
    require(!c.is_valid(false), "edge-edge must return invalid contact (D4)");
}

// ─────────────────────────────────────────────────────────────────────────────
// T3: YAML authoring (YamlUtils.cpp)
// ─────────────────────────────────────────────────────────────────────────────

void test_edge_yaml_load() {
    FxShape shape = FxYAML::buildShape(R"yaml(
geometry:
  edge: [[-2, 0], [2, 0]]
)yaml");
    require(shape.is_edge(), "YAML-parsed edge must have is_edge() == true");
}

void test_edge_yaml_bad_arity() {
    bool threw = false;
    try {
        FxYAML::buildShape(R"yaml(
geometry:
  edge: [[0, 0]]
)yaml");
    } catch (const std::runtime_error&) {
        threw = true;
    }
    require(threw, "single-point edge YAML must throw std::runtime_error");
}

} // namespace

void run_edge_tests() {
    test_edge_shape_properties();
    test_edge_aabb();
    test_edge_degenerate_throws();
    test_edge_circle_contact();
    test_edge_circle_separated();
    test_edge_box_contact();
    test_edge_box_offspan_no_contact();
    test_edge_edge_no_contact();
    test_edge_yaml_load();
    test_edge_yaml_bad_arity();
    std::cout << "Edge tests passed." << std::endl;
}
