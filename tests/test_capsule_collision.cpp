#include "Fx2D/Entity.h"
#include "Fx2D/Math.h"
#include "Fx2D/Solver.h"
#include "Fx2D/YamlUtils.h"

#include "test_harness.h"

#include <cmath>
#include <iostream>
#include <memory>
#include <stdexcept>

namespace {

bool approx(float a, float b, float eps = 1e-3f) {
    return std::fabs(a - b) <= eps;
}

// Build a capsule entity at (x, y) with segment length and skin radius.
std::shared_ptr<FxEntity> make_capsule(const std::string& name, float x, float y, float length,
                                       float radius, float theta = 0.0f) {
    auto e = std::make_shared<FxEntity>(name);
    e->set_mass(1.0f);
    e->set_collision_geometry(FxCollisionShape(length, radius));
    e->set_init_pose(FxVec3f{x, y, theta});
    e->set_inertia();
    e->step(FxVec2f{0.0f, 0.0f}, 0.0);
    return e;
}

std::shared_ptr<FxEntity> make_circle(const std::string& name, float x, float y, float r) {
    auto e = std::make_shared<FxEntity>(name);
    e->set_mass(1.0f);
    e->set_collision_geometry(FxCollisionShape(r));
    e->set_init_pose(FxVec3f{x, y, 0.0f});
    e->set_inertia();
    e->step(FxVec2f{0.0f, 0.0f}, 0.0);
    return e;
}

std::shared_ptr<FxEntity> make_rect(const std::string& name, float x, float y, float w, float h,
                                    float skin = 0.0f, float theta = 0.0f) {
    auto e = std::make_shared<FxEntity>(name);
    e->set_mass(1.0f);
    e->set_collision_geometry(FxCollisionShape(FxVec2f{w, h}, skin));
    e->set_init_pose(FxVec3f{x, y, theta});
    e->set_inertia();
    e->step(FxVec2f{0.0f, 0.0f}, 0.0);
    return e;
}

// Capsule storage uses 2 vertices along the x-axis (-L/2, 0) and (+L/2, 0).
void test_capsule_construction() {
    FxShape s(2.0f, 0.5f);
    require(s.is_capsule(), "FxShape(L,r) must be classified as capsule");
    require(!s.is_circle() && !s.is_polygon(), "capsule must not be circle or polygon");
    require(approx(s.skin_radius(), 0.5f), "capsule skin radius mismatch");
    require(approx(s.radius(), 1.5f), "capsule bounding radius = L/2 + r");
    require(approx(s.area(), FxPif * 0.25f + 2.0f * 0.5f * 2.0f), "capsule area = pi r^2 + 2rL");
}

// Two horizontal capsules end-on. Surfaces just touch when centers are L + 2r apart.
void test_capsule_vs_capsule_endon() {
    const float L = 2.0f, r = 0.5f;
    // Move them slightly closer so the skins overlap.
    auto a = make_capsule("cap_a", -1.4f, 0.0f, L, r); // right end at -0.4
    auto b = make_capsule("cap_b", 1.4f, 0.0f, L, r); // left end at 0.4 -> gap should be 0.8 - 2r =
                                                      // -0.2
    FxContact c = FxSolver::collision_check(a, b);
    require(c.is_valid(), "end-on capsules should collide");
    require(approx(c.penetration_depth, 0.2f, 1e-3f), "end-on penetration must be ~0.2");
    require(approx(c.normal.x(), 1.0f, 1e-2f), "normal must point along +x (A->B)");
    require(approx(c.normal.y(), 0.0f, 1e-2f), "normal y component must be ~0");
}

// Two horizontal capsules stacked: side-by-side overlap along y.
void test_capsule_vs_capsule_side_by_side() {
    const float L = 2.0f, r = 0.5f;
    auto a = make_capsule("cap_lo", 0.0f, 0.0f, L, r);
    auto b = make_capsule("cap_hi", 0.0f, 0.8f, L, r); // gap along y = 0.8, surface gap = -0.2
    FxContact c = FxSolver::collision_check(a, b);
    require(c.is_valid(), "side-by-side capsules should overlap");
    require(approx(c.penetration_depth, 0.2f, 1e-3f), "stacked capsule penetration ~0.2");
    require(c.normal.y() > 0.9f, "normal must point along +y (A->B)");
}

// Capsule and a far-away circle: no contact.
void test_capsule_vs_circle_separated() {
    auto cap = make_capsule("cap", 0.0f, 0.0f, 2.0f, 0.5f);
    auto circle = make_circle("ball", 10.0f, 0.0f, 0.5f);
    FxContact c = FxSolver::collision_check(cap, circle);
    require(!c.is_valid(), "separated capsule/circle must report no contact");
}

// Circle just touching the side of a horizontal capsule.
void test_capsule_vs_circle_side() {
    const float L = 4.0f, r = 0.5f;
    auto cap = make_capsule("cap", 0.0f, 0.0f, L, r);
    auto circle = make_circle("ball", 0.5f, 0.8f, 0.5f); // surface gap along y = 0.8 - r - 0.5 =
                                                         // -0.2
    FxContact c = FxSolver::collision_check(cap, circle);
    require(c.is_valid(), "circle should hit capsule side");
    require(approx(c.penetration_depth, 0.2f, 1e-3f), "side penetration ~0.2");
    require(c.normal.y() > 0.9f, "normal points from capsule toward circle (+y)");
}

// Capsule hitting a thin rectangle from above (rounded shape vs polygon).
void test_capsule_vs_rectangle() {
    auto cap = make_capsule("cap", 0.0f, 1.4f, 4.0f, 0.5f); // bottom of capsule at y=0.9
    auto rect = make_rect("floor", 0.0f, 0.0f, 10.0f, 2.0f); // top of rect at y=1.0
    // Overlap along y: capsule bottom at 0.9, rect top at 1.0 -> 0.1 penetration.
    FxContact c = FxSolver::collision_check(cap, rect);
    require(c.is_valid(), "capsule should touch rectangle");
    require(c.penetration_depth > 0.05f && c.penetration_depth < 0.15f,
            "capsule/rect penetration ~0.1");
    require(c.normal.y() < -0.9f, "normal must point from capsule (A) toward rect (B), i.e. -y");
}

// Rounded box: rectangle + skin. Rounded box vs rounded box at 45 degrees.
void test_rounded_box_construction() {
    FxShape s(FxVec2f{2.0f, 2.0f}, 0.25f);
    require(s.is_polygon(), "rounded box stays a polygon");
    require(approx(s.skin_radius(), 0.25f), "rounded box skin radius mismatch");
    require(s.area() > 4.0f, "rounded box area must exceed inner-rect area");
}

// Two rounded squares: place so skins just overlap horizontally.
void test_rounded_box_vs_rounded_box() {
    const float w = 2.0f, h = 2.0f, sk = 0.25f;
    auto a = make_rect("ra", 0.0f, 0.0f, w, h, sk);
    auto b = make_rect("rb", 2.4f, 0.0f, w, h, sk); // gap between raw rects = 0.4, skins overlap by
                                                    // 0.1
    FxContact c = FxSolver::collision_check(a, b);
    require(c.is_valid(), "rounded boxes with overlapping skins must collide");
    require(approx(c.penetration_depth, 0.1f, 1e-2f), "rounded-box penetration ~0.1");
    require(c.normal.x() > 0.9f, "normal points A->B along +x");
}

// YAML loader: capsule and `radius:` modifier produce the correct FxShape.
// Parses YAML::Nodes directly to avoid the buildShape(string) filesystem-path probe.
void test_yaml_loads_capsule_and_skin_radius() {
    // Capsule via sequence form.
    YAML::Node cfg_cap;
    cfg_cap["geometry"]["capsule"].push_back(2.5f);
    cfg_cap["geometry"]["capsule"].push_back(0.3f);
    FxShape s_cap = FxYAML::buildShape(cfg_cap);
    require(s_cap.is_capsule(), "YAML 'capsule:' must build a capsule");
    require(approx(s_cap.skin_radius(), 0.3f), "capsule skin radius from YAML mismatch");

    // Capsule via map form.
    YAML::Node cfg_cap2;
    cfg_cap2["geometry"]["capsule"]["length"] = 1.0f;
    cfg_cap2["geometry"]["capsule"]["radius"] = 0.2f;
    FxShape s_cap2 = FxYAML::buildShape(cfg_cap2);
    require(s_cap2.is_capsule(), "YAML capsule map form must build a capsule");
    require(approx(s_cap2.skin_radius(), 0.2f), "capsule (map) skin radius mismatch");

    // Rounded rectangle via `rectangle:` + `radius:`.
    YAML::Node cfg_rect;
    cfg_rect["geometry"]["rectangle"].push_back(2.0f);
    cfg_rect["geometry"]["rectangle"].push_back(2.0f);
    cfg_rect["geometry"]["radius"] = 0.25f;
    FxShape s_rect = FxYAML::buildShape(cfg_rect);
    require(s_rect.is_polygon(), "rounded rect stays a polygon");
    require(approx(s_rect.skin_radius(), 0.25f), "rounded rect skin radius mismatch");

    // Plain rectangle (no skin) remains backward-compatible.
    YAML::Node cfg_old;
    cfg_old["geometry"]["rectangle"].push_back(1.0f);
    cfg_old["geometry"]["rectangle"].push_back(1.0f);
    FxShape s_old = FxYAML::buildShape(cfg_old);
    require(s_old.is_polygon(), "plain rectangle still a polygon");
    require(approx(s_old.skin_radius(), 0.0f), "plain rectangle has zero skin");
}

// Sanity: zero-length capsule must equal a circle's contact behavior.
void test_zero_length_capsule_behaves_like_circle() {
    auto cap = make_capsule("cap", 0.0f, 0.0f, 0.0f, 0.5f);
    auto circ_ref = make_circle("ref", 0.0f, 0.0f, 0.5f);
    require(cap->collision_geometry()->is_capsule(), "stored as capsule");
    auto target = make_circle("hit", 0.8f, 0.0f, 0.5f);
    FxContact c1 = FxSolver::collision_check(cap, target);
    FxContact c2 = FxSolver::collision_check(circ_ref, target);
    require(c1.is_valid() && c2.is_valid(), "both must collide");
    require(approx(c1.penetration_depth, c2.penetration_depth, 1e-3f),
            "zero-length capsule and circle should produce same penetration depth");
}

} // namespace

void run_capsule_tests() {
    test_capsule_construction();
    test_capsule_vs_capsule_endon();
    test_capsule_vs_capsule_side_by_side();
    test_capsule_vs_circle_separated();
    test_capsule_vs_circle_side();
    test_capsule_vs_rectangle();
    test_rounded_box_construction();
    test_rounded_box_vs_rounded_box();
    test_zero_length_capsule_behaves_like_circle();
    test_yaml_loads_capsule_and_skin_radius();
    std::cout << "Capsule and rounded-shape tests passed." << std::endl;
}
