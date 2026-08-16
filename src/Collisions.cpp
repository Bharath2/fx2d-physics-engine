#include "Fx2D/Entity.h"
#include "Fx2D/Solver.h"
#include <iostream>
#include <limits>

namespace FxSolver {

// AABB overlap check
bool aabb_overlap_check(const FxEntity& entity1, const FxEntity& entity2) {
    auto aabb1 = entity1.bounding_box();
    auto aabb2 = entity2.bounding_box();
    return !(aabb1(2) < aabb2(0) || aabb2(2) < aabb1(0) || aabb1(3) < aabb2(1) ||
             aabb2(3) < aabb1(1));
}

// Overload for shared_ptr, delegates to object reference version
bool aabb_overlap_check(const std::shared_ptr<FxEntity>& entity1,
                        const std::shared_ptr<FxEntity>& entity2) {
    if (!entity1 || !entity2) return false;
    return aabb_overlap_check(*entity1, *entity2);
}

std::pair<FxVec2f, FxVec2f> clip_edge(const FxVec2f& p1, const FxVec2f& q1, const FxVec2f& p2,
                                      const FxVec2f& q2) {
    // Calculate edge direction and check for validity
    FxVec2f edge_dir = q2 - p2;
    float edge_length_sq = edge_dir.dot(edge_dir);
    if (edge_length_sq <= 1e-6f) {
        return std::make_pair(p1, p1); // Degenerate edge case
    }

    // Calculate edge normal and length
    FxVec2f edge_normal = edge_dir.perp();
    float edge_length = std::sqrt(edge_length_sq);
    FxVec2f edge_unit = edge_dir / edge_length;

    // Calculate signed distances from p1 and q1 to the line p2-q2
    float d1 = (p1 - p2).dot(edge_normal);
    float d2 = (q1 - p2).dot(edge_normal);

    // Clip q1 if p1 and q1 are on opposite sides of edge
    FxVec2f clipped_q1 = q1;
    if (d1 * d2 <= 0.0f) {
        float denom = d1 - d2;
        if (std::abs(denom) >= 1e-6f) {
            clipped_q1 = p1 + d1 * (q1 - p1) / denom;
        }
    }

    // Project points onto edge p2-q2
    float t1 = (p1 - p2).dot(edge_unit);
    float t2 = (clipped_q1 - p2).dot(edge_unit);

    // Clamp projections to edge
    t1 = std::clamp(t1, 0.0f, edge_length);
    t2 = std::clamp(t2, 0.0f, edge_length);

    // Calculate final projected points
    FxVec2f p1_projected = p2 + t1 * edge_unit;
    FxVec2f q1_projected = p2 + t2 * edge_unit;

    return std::make_pair(p1_projected, q1_projected);
}

// ----------------------------------------------------------------------
// Geometric helpers used by capsule reductions
// ----------------------------------------------------------------------

// Closest pair of points between two segments [a1,b1] and [a2,b2].
// Returns {point on seg1, point on seg2}. Handles parallel and degenerate cases.
static std::pair<FxVec2f, FxVec2f> seg_seg_closest(const FxVec2f& a1, const FxVec2f& b1,
                                                   const FxVec2f& a2, const FxVec2f& b2) {
    FxVec2f d1 = b1 - a1;
    FxVec2f d2 = b2 - a2;
    FxVec2f r = a1 - a2;
    float la = d1.dot(d1);
    float lb = d2.dot(d2);
    float f = d2.dot(r);
    float s, t;
    if (la <= 1e-12f && lb <= 1e-12f) return {a1, a2}; // both degenerate
    if (la <= 1e-12f) {
        s = 0.0f;
        t = std::clamp(f / lb, 0.0f, 1.0f);
    } else {
        float c = d1.dot(r);
        if (lb <= 1e-12f) {
            t = 0.0f;
            s = std::clamp(-c / la, 0.0f, 1.0f);
        } else {
            float b = d1.dot(d2);
            float denom = la * lb - b * b;
            s = (denom != 0.0f) ? std::clamp((b * f - c * lb) / denom, 0.0f, 1.0f) : 0.0f;
            t = (b * s + f) / lb;
            if (t < 0.0f) {
                t = 0.0f;
                s = std::clamp(-c / la, 0.0f, 1.0f);
            } else if (t > 1.0f) {
                t = 1.0f;
                s = std::clamp((b - c) / la, 0.0f, 1.0f);
            }
        }
    }
    return {a1 + s * d1, a2 + t * d2};
}

// Closest point on a capsule's central segment to a query world point.
static FxVec2f closest_on_capsule_segment(const FxShape* capsule, const FxVec2f& p) {
    const auto& v = capsule->vertices();
    return FxClosestOnSegment(v[0], v[1], p);
}

// Closest pair of points between a capsule's segment and a polygon's boundary.
// Returns {capsule-segment point, polygon-boundary point}.
static std::pair<FxVec2f, FxVec2f> closest_capsule_to_polygon(const FxShape* capsule,
                                                              const FxShape* polygon) {
    const auto& cv = capsule->vertices();
    const auto& pv = polygon->vertices();
    float best_d2 = FxInfinityf;
    FxVec2f best_p1{0.f, 0.f}, best_p2{0.f, 0.f};
    for (size_t i = 0, n = pv.size(); i < n; ++i) {
        const FxVec2f& s = pv[i];
        const FxVec2f& e = pv[(i + 1) % n];
        auto [p1, p2] = seg_seg_closest(cv[0], cv[1], s, e);
        float d2 = (p2 - p1).dot(p2 - p1);
        if (d2 < best_d2) {
            best_d2 = d2;
            best_p1 = p1;
            best_p2 = p2;
        }
    }
    return {best_p1, best_p2};
}

// Edge vs polygon: use the segment as the reference axis (generic capsule misses interiors).
static FxContact edge_polygon_contact(const FxShape* edge, const FxShape* poly) {
    const auto& ev = edge->vertices();
    const FxVec2f E0 = ev[0];
    FxVec2f eu = ev[1] - E0;
    const float L = eu.norm();
    if (L < 1e-6f) return FxContact(false);
    eu /= L;

    FxVec2f n = eu.perp();
    if ((poly->centroid() - E0).dot(n) < 0.0f) n = -n; // orient from edge toward polygon

    const auto& pv = poly->vertices();
    const size_t nP = pv.size();
    if (nP == 0) return FxContact(false);

    // The polygon's own skin extends its surface toward the edge.
    const float rB = poly->skin_radius();

    float deepest_sd = FxInfinityf;
    float span_min_t = FxInfinityf, span_max_t = -FxInfinityf;
    for (size_t i = 0; i < nP; ++i) {
        const FxVec2f d = pv[i] - E0;
        deepest_sd = std::min(deepest_sd, d.dot(n) - rB);
        const float t = d.dot(eu);
        span_min_t = std::min(span_min_t, t);
        span_max_t = std::max(span_max_t, t);
    }
    if (deepest_sd >= 0.0f) return FxContact(false);
    if (span_max_t < -rB || span_min_t > L + rB) return FxContact(false);

    // Keep the two deepest penetrating vertices that project onto the segment span.
    FxVec2f best[2] = {FxVec2f{0.0f, 0.0f}, FxVec2f{0.0f, 0.0f}};
    float bsd[2] = {0.0f, 0.0f};
    int nb = 0;
    for (size_t i = 0; i < nP; ++i) {
        const FxVec2f d = pv[i] - E0;
        const float sd = d.dot(n) - rB;
        if (sd >= 0.0f) continue;
        const float t = d.dot(eu);
        if (t < 0.0f || t > L) continue;
        const FxVec2f cpt = E0 + t * eu;
        if (nb < 2) {
            best[nb] = cpt;
            bsd[nb] = sd;
            ++nb;
        } else {
            const int shallowest = (bsd[0] > bsd[1]) ? 0 : 1;
            if (sd < bsd[shallowest]) {
                best[shallowest] = cpt;
                bsd[shallowest] = sd;
            }
        }
    }
    if (nb == 0) return FxContact(false);

    FxContact c(true);
    c.normal = n;
    c.penetration_depth = -std::min(bsd[0], (nb == 2) ? bsd[1] : bsd[0]);
    c.position[0] = best[0];
    c.count = 1;
    if (nb == 2 && (best[1] - best[0]).norm() >= 0.01f) {
        c.position[1] = best[1];
        c.count = 2;
    }
    return c;
}

// tests A's edge normals against B; early-exits on first sep axis, else tracks min-penetration
// axis. Returned `gap` is skin-inclusive (raw B_min_val - rA - rB), so gap > 0 means full
// separation.
static FxSatResult sat_query(const FxShape* A_shape, const FxShape* B_shape) {
    const auto& A_vertices = A_shape->vertices();
    const float skin_sum = A_shape->skin_radius() + B_shape->skin_radius();
    FxSatResult result;
    for (size_t i = 0, n = A_vertices.size(); i < n; ++i) {
        const FxVec2f& s = A_vertices[i];
        const FxVec2f& e = A_vertices[(i + 1) % n];
        FxVec2f dir = (e - s).normalized();
        FxVec2f axis = dir.perp();
        auto [B_min_idx, B_min_val] = B_shape->project_onto(axis, s).argmin();
        float gap_val = B_min_val - skin_sum;
        if (gap_val > 0.f) {
            result.has_sep = true;
            return result;
        } // separating axis
        if (gap_val > result.gap) {
            result.normal = axis;
            result.gap = gap_val;
            result.ref_edge_dir = dir;
            result.ref_edge_index = i;
            result.pen_vertex_index = B_min_idx;
        }
    }
    return result;
}

// Build a contact between a circle (centered at virtual position vc, radius vrA) and a polygon.
// vrA is the effective skin radius; the polygon's own skin is added internally.
static FxContact circle_vs_polygon_contact(const FxVec2f& vc, float vrA, const FxShape* B_polygon) {
    FxContact contact(true);
    const auto& Bv = B_polygon->vertices();
    if (Bv.empty()) {
        contact.set_valid(false);
        return contact;
    }
    const float rB = B_polygon->skin_radius();
    float min_dist = FxInfinityf;
    FxVec2f closest(0.0f, 0.0f);
    for (size_t i = 0, n = Bv.size(); i < n; ++i) {
        const FxVec2f& s = Bv[i];
        const FxVec2f& e = Bv[(i + 1) % n];
        FxVec2f dir = e - s;
        float edge_length = dir.dot(dir);
        if (edge_length < 1e-6f) continue;
        float t = std::clamp((vc - s).dot(dir) / edge_length, 0.f, 1.f);
        FxVec2f p = s + t * dir;
        float d = (vc - p).norm();
        if (d < min_dist) {
            min_dist = d;
            closest = p;
        }
    }
    if (min_dist == FxInfinityf) {
        contact.set_valid(false);
        return contact;
    }
    float penetration = vrA + rB - min_dist;
    if (penetration > 0.f) {
        FxVec2f n = (min_dist > 1e-6f) ? (closest - vc) / min_dist : FxVec2f{1.f, 0.f}; // from
                                                                                        // circle ->
                                                                                        // polygon
        contact.normal = n;
        contact.penetration_depth = penetration;
        // Contact lies on the polygon's skin surface facing the circle.
        contact.position[0] = closest - n * rB;
        contact.count = 1;
    } else contact.set_valid(false);
    return contact;
}

FxContact compute_contact_one_way(const FxShape* A_shape, const FxShape* B_shape) {
    auto contact = FxContact(true);
    contact.penetration_depth = 0.0f;

    // ----------- CAPSULE REDUCTIONS -----------
    // Reduce capsule-vs-X to "virtual circle at closest segment point vs X" using
    // the capsule's skin_radius as the effective circle radius.
    if (A_shape->is_capsule()) {
        const float rA = A_shape->skin_radius();
        // Two zero-thickness segments have no volume to resolve.
        if (A_shape->is_edge() && B_shape->is_edge()) {
            contact.set_valid(false);
            return contact;
        }
        if (B_shape->is_circle()) {
            FxVec2f vc_A = closest_on_capsule_segment(A_shape, B_shape->centroid());
            FxVec2f cB = B_shape->centroid();
            float rB = B_shape->skin_radius();
            FxVec2f d = cB - vc_A;
            float dist = d.norm();
            float pen = (rA + rB) - dist;
            if (pen > 0.0f) {
                FxVec2f n = (dist > 1e-6f) ? d / dist : FxVec2f{1.f, 0.f};
                contact.normal = n;
                contact.penetration_depth = pen;
                contact.position[0] = vc_A + n * (rA - 0.5f * pen);
                contact.count = 1;
            } else contact.set_valid(false);
            return contact;
        }
        if (B_shape->is_capsule()) {
            const auto& av = A_shape->vertices();
            const auto& bv = B_shape->vertices();
            auto [pA, pB] = seg_seg_closest(av[0], av[1], bv[0], bv[1]);
            float rB = B_shape->skin_radius();
            FxVec2f d = pB - pA;
            float dist = d.norm();
            float pen = (rA + rB) - dist;
            if (pen > 0.0f) {
                FxVec2f n = (dist > 1e-6f) ? d / dist : FxVec2f{1.f, 0.f};
                contact.normal = n;
                contact.penetration_depth = pen;
                contact.position[0] = pA + n * (rA - 0.5f * pen);
                contact.count = 1;
            } else contact.set_valid(false);
            return contact;
        }
        // Bare segment vs polygon needs the line-reference query, not the circle reduction.
        if (A_shape->is_edge()) return edge_polygon_contact(A_shape, B_shape);
        // Capsule vs polygon: closest seg-vs-polygon point, then circle-vs-polygon.
        auto [pCap, pPoly] = closest_capsule_to_polygon(A_shape, B_shape);
        return circle_vs_polygon_contact(pCap, rA, B_shape);
    }
    if (B_shape->is_capsule()) {
        // Symmetric: swap and flip normal so it stays A->B.
        contact = compute_contact_one_way(B_shape, A_shape);
        if (contact.is_valid(false)) contact.normal = -contact.normal;
        return contact;
    }

    // ----------- CIRCLE/POLYGON -----------

    // Circle vs Circle
    if (A_shape->is_circle() && B_shape->is_circle()) {
        FxVec2f cA = A_shape->centroid();
        FxVec2f cB = B_shape->centroid();
        FxVec2f d = cB - cA;
        float dist = d.norm();
        float rA = A_shape->skin_radius(), rB = B_shape->skin_radius();
        float penetration = (rA + rB) - dist;
        if (penetration > 0.f) {
            FxVec2f n = (dist > 1e-6f) ? d / dist : FxVec2f{1.f, 0.f};
            contact.normal = n; // from A -> B
            contact.penetration_depth = penetration;
            contact.position[0] = cA + n * (rA - 0.5f * penetration);
            contact.count = 1;
        } else contact.set_valid(false);
        return contact;
    }

    // Circle vs Polygon
    if (A_shape->is_circle() && B_shape->is_polygon()) {
        return circle_vs_polygon_contact(A_shape->centroid(), A_shape->skin_radius(), B_shape);
    }

    // Polygon vs Circle (reuse, flip normal so it stays A->B)
    if (A_shape->is_polygon() && B_shape->is_circle()) {
        contact = compute_contact_one_way(B_shape, A_shape);
        if (contact.is_valid(false)) {
            contact.normal = -contact.normal;
        } // now from polygon(A) -> circle(B)
        return contact;
    }

    // Polygon vs Polygon (skin-aware via sat_query — `sat.gap` already includes both skins)
    const auto& A_vertices = A_shape->vertices();
    const auto& B_vertices = B_shape->vertices();
    const float rB = B_shape->skin_radius();
    FxSatResult sat = sat_query(A_shape, B_shape);
    if (sat.has_sep) {
        contact.set_valid(false);
        return contact;
    }
    contact.normal = sat.normal;
    contact.penetration_depth = -sat.gap; // positive when overlapping (gap <= 0)

    if (!B_vertices.empty()) {
        const size_t B_N = B_vertices.size();
        const size_t ifwd = (sat.pen_vertex_index + 1) % B_N;
        const size_t ibwd = (sat.pen_vertex_index + B_N - 1) % B_N;
        const FxVec2f B_edge_start = B_vertices[sat.pen_vertex_index];
        const float dot_fwd =
            std::abs((B_vertices[ifwd] - B_edge_start).normalized().dot(sat.ref_edge_dir));
        const float dot_bwd =
            std::abs((B_edge_start - B_vertices[ibwd]).normalized().dot(sat.ref_edge_dir));
        const FxVec2f B_edge_end = dot_bwd > dot_fwd ? B_vertices[ibwd] : B_vertices[ifwd];
        const FxVec2f& A_edge_start = A_vertices[sat.ref_edge_index];
        const FxVec2f& A_edge_end = A_vertices[(sat.ref_edge_index + 1) % A_vertices.size()];
        const auto contact_points = clip_edge(B_edge_start, B_edge_end, A_edge_start, A_edge_end);
        // Shift onto B's skin surface (which faces A along -normal).
        const FxVec2f skin_shift = -rB * sat.normal;
        contact.position[0] = contact_points.first + skin_shift;
        contact.position[1] = contact_points.second + skin_shift;
        // Check if points are too close and resolve to one point if needed
        float dist = (contact.position[1] - contact.position[0]).norm();
        contact.count = (dist < 0.01f) ? 1 : 2;
    }
    return contact;
}

// Separating Axis Theorem collision check method
const FxContact collision_check(const std::shared_ptr<FxEntity>& entity1,
                                const std::shared_ptr<FxEntity>& entity2) {
    // Check if both entities have collision geometry
    if (!entity1 || !entity2) return FxContact(false);
    if (!entity1->collision_geometry() || !entity2->collision_geometry()) {
        return FxContact(false);
    }

    // Check if bounding boxes overlap first
    if (!aabb_overlap_check(*entity1, *entity2)) return FxContact(false);

    auto contact = FxContact(true);

    // the shapes are considered to be intersecting if they are not separated along any axis.
    const FxShape* A = entity1->collision_geometry().get();
    const FxShape* B = entity2->collision_geometry().get();

    // Circle and capsule paths produce a definitive single-direction contact; polygon-vs-polygon
    // runs both directions and biases toward A for jitter stability.
    if (A->is_circle() || A->is_capsule()) {
        contact = compute_contact_one_way(A, B);
    } else if (B->is_circle() || B->is_capsule()) {
        contact = compute_contact_one_way(B, A);
        if (contact.is_valid(false)) contact.normal = -contact.normal; // restore A->B convention
    } else {
        FxContact cAB = compute_contact_one_way(A, B);
        FxContact cBA = compute_contact_one_way(B, A);
        if (!cAB.is_valid(false) || !cBA.is_valid(false)) return FxContact(false);
        // bias toward cAB: prevents jitter from flipping the reference edge each frame
        float bias = 0.005f * cAB.penetration_depth + 1e-6f;
        contact = (cBA.penetration_depth < cAB.penetration_depth - bias) ? cBA : cAB;
    }

    // Ensure normal points from entity1 -> entity2
    if (contact.is_valid(false)) {
        FxVec2f delta = entity2->pose.xy() - entity1->pose.xy();
        if (delta.dot(contact.normal) < 0.0f) contact.normal = -contact.normal;
        contact.entity1 = entity1;
        contact.entity2 = entity2;
        contact.normal = contact.normal.normalized();
    }
    return contact;
}

// no-early-exit SAT: returns the axis of minimum signed separation for speculative contact.
// Returned gap is skin-inclusive (raw B_min_val - rA - rB).
static std::pair<FxVec2f, float> sat_gap_query(const FxShape* A_shape, const FxShape* B_shape) {
    const auto& A_vertices = A_shape->vertices();
    const float skin_sum = A_shape->skin_radius() + B_shape->skin_radius();
    FxVec2f best_normal{1.0f, 0.0f};
    float best_val = FxInfinityf;
    for (size_t i = 0, n = A_vertices.size(); i < n; ++i) {
        const FxVec2f& s = A_vertices[i];
        const FxVec2f& e = A_vertices[(i + 1) % n];
        FxVec2f dir = (e - s).normalized();
        FxVec2f axis = dir.perp();
        auto [B_min_idx, B_min_val] = B_shape->project_onto(axis, s).argmin();
        float gap_val = B_min_val - skin_sum;
        if (gap_val < best_val) {
            best_val = gap_val;
            best_normal = axis;
        }
    }
    return {best_normal, best_val};
}

// speculative contact for CCD: returns pre-contact (gap < 0) when bodies will collide this substep
FxContact speculative_contact_check(const std::shared_ptr<FxEntity>& entity1,
                                    const std::shared_ptr<FxEntity>& entity2, float substep_dt) {
    if (!entity1 || !entity2) return FxContact(false);
    if (!entity1->collision_geometry() || !entity2->collision_geometry()) return FxContact(false);
    if (substep_dt <= 0.0f) return FxContact(false);

    const FxShape* A = entity1->collision_geometry().get();
    const FxShape* B = entity2->collision_geometry().get();
    // Bare segments carry no skin, so the distance-minus-radii gap math degenerates for them.
    // Edges are static level geometry, where discrete contacts are sufficient.
    if (A->is_edge() || B->is_edge()) return FxContact(false);
    FxVec2f rel_vel = entity2->velocity.head<2>() - entity1->velocity.head<2>();

    FxVec2f normal;
    float gap;
    FxVec2f contact_anchor = entity1->pose.xy(); // overridden below for non-circle A

    // Reduce capsule to "virtual circle at closest segment pt to the other shape's reference"
    // so the speculative gap math stays a simple distance - radii subtraction.
    auto cap_reduce = [](const FxShape* cap,
                         const FxVec2f& other_ref) -> std::pair<FxVec2f, float> {
        return {closest_on_capsule_segment(cap, other_ref), cap->skin_radius()};
    };

    if ((A->is_circle() || A->is_capsule()) && (B->is_circle() || B->is_capsule())) {
        FxVec2f cA, cB;
        float rA, rB;
        if (A->is_capsule() && B->is_capsule()) {
            const auto& av = A->vertices();
            const auto& bv = B->vertices();
            auto [pA, pB] = seg_seg_closest(av[0], av[1], bv[0], bv[1]);
            cA = pA;
            cB = pB;
            rA = A->skin_radius();
            rB = B->skin_radius();
        } else if (A->is_capsule()) {
            cB = B->centroid();
            std::tie(cA, rA) = cap_reduce(A, cB);
            rB = B->skin_radius();
        } else if (B->is_capsule()) {
            cA = A->centroid();
            std::tie(cB, rB) = cap_reduce(B, cA);
            rA = A->skin_radius();
        } else {
            cA = A->centroid();
            cB = B->centroid();
            rA = A->skin_radius();
            rB = B->skin_radius();
        }
        FxVec2f delta = cB - cA;
        float dist = delta.norm();
        if (dist < 1e-6f) return FxContact(false);
        normal = delta / dist;
        gap = dist - rA - rB;
        contact_anchor = cA;
    } else if (A->is_circle() || A->is_capsule() || B->is_circle() || B->is_capsule()) {
        // Mixed: one is point/segment (rounded), the other is polygon.
        const FxShape* round_shape = (A->is_circle() || A->is_capsule()) ? A : B;
        const FxShape* polygon = (round_shape == A) ? B : A;
        FxVec2f vc;
        float rR;
        if (round_shape->is_capsule()) {
            auto [pCap, pPoly] = closest_capsule_to_polygon(round_shape, polygon);
            vc = pCap;
            rR = round_shape->skin_radius();
        } else {
            vc = round_shape->centroid();
            rR = round_shape->skin_radius();
        }
        // Closest point on the polygon boundary to vc.
        const auto& verts = polygon->vertices();
        float min_dist = FxInfinityf;
        FxVec2f closest{0.0f, 0.0f};
        for (size_t i = 0, n = verts.size(); i < n; ++i) {
            const FxVec2f& s = verts[i];
            const FxVec2f& e = verts[(i + 1) % n];
            FxVec2f dir = e - s;
            float len2 = dir.dot(dir);
            if (len2 < 1e-6f) continue;
            float t = std::clamp((vc - s).dot(dir) / len2, 0.f, 1.f);
            FxVec2f p = s + t * dir;
            float d = (vc - p).norm();
            if (d < min_dist) {
                min_dist = d;
                closest = p;
            }
        }
        if (min_dist == FxInfinityf) return FxContact(false);
        FxVec2f raw = (min_dist > 1e-6f) ? (closest - vc) / min_dist : FxVec2f{1.f, 0.f};
        normal = (round_shape == A) ? raw : -raw; // keep entity1 → entity2 convention
        gap = min_dist - rR - polygon->skin_radius();
        contact_anchor = vc;
    } else {
        // Polygon-polygon: run gap queries both ways, pick the axis with the smallest gap.
        // sat_gap_query already returns skin-inclusive gaps.
        auto [nAB, gAB] = sat_gap_query(A, B);
        auto [nBA, gBA] = sat_gap_query(B, A);
        if (gAB <= gBA) {
            normal = nAB;
            gap = gAB;
        } else {
            normal = -nBA;
            gap = gBA;
        } // flip B->A to A->B convention
        contact_anchor = entity1->pose.xy();
    }

    // Ensure normal points from entity1 → entity2
    FxVec2f c2c = entity2->pose.xy() - entity1->pose.xy();
    if (c2c.dot(normal) < 0.0f) normal = -normal;

    // Closing speed along the contact normal (positive = approaching)
    float v_closing = -rel_vel.dot(normal);
    if (v_closing <= 0.0f) return FxContact(false); // separating

    // Pre-contact depth: negative means the gap will close this substep
    float spec_depth = gap - v_closing * substep_dt;
    if (spec_depth >= 0.0f) return FxContact(false);

    FxContact c(true);
    c.count = 1;
    c.normal = normal;
    c.penetration_depth = spec_depth;
    // Anchor at the deepest feature of A; falls back to A's centroid for polygons.
    c.position[0] = contact_anchor + normal * A->skin_radius();
    c.entity1 = entity1;
    c.entity2 = entity2;
    return c;
}

} // namespace FxSolver
