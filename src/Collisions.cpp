#include "Fx2D/Entity.h"
#include "Fx2D/Solver.h"
#include <iostream>
#include <limits>

namespace FxSolver {

// AABB overlap check
bool aabb_overlap_check(const FxEntity& entity1, const FxEntity& entity2) {
    const auto& aabb1 = entity1.bounding_box();
    const auto& aabb2 = entity2.bounding_box();
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

// Segment [q0, q1] clipped to the span of reference segment [s, e]: the two ends of their
// overlap, and their heights above the reference line along n. False if the overlap is shorter
// than a manifold's two points may be, or the segments are not near parallel.
static bool clip_to_span(const FxVec2f& q0, const FxVec2f& q1, const FxVec2f& s, const FxVec2f& e,
                         const FxVec2f& n, FxVec2f out[2], float height[2]) {
    const FxVec2f u = e - s;
    const float span = u.norm();
    const FxVec2f q = q1 - q0;
    const float length = q.norm();
    if (span < 1e-6f || length < 1e-6f) return false;
    // Both segments square to the normal, within ~8.6 degrees, or the heights mean nothing.
    if (std::fabs(q.dot(n)) > 0.15f * length || std::fabs(u.dot(n)) > 0.15f * span) return false;
    const float a0 = (q0 - s).dot(u) / span, a1 = (q1 - s).dot(u) / span;
    float lo = std::clamp((0.0f - a0) / (a1 - a0), 0.0f, 1.0f);
    float hi = std::clamp((span - a0) / (a1 - a0), 0.0f, 1.0f);
    if (lo > hi) std::swap(lo, hi);
    if ((hi - lo) * length < 0.01f) return false;
    out[0] = q0 + lo * q;
    out[1] = q0 + hi * q;
    for (int i = 0; i < 2; ++i)
        height[i] = (out[i] - s).dot(n);
    return true;
}

// A capsule lying along a polygon's face, or along another capsule, rests on two points: one,
// the closest, would see it rock from end to end and never settle. `ref` carries the reference
// segment [s, e] with outward normal n; `inc` is the capsule's core; positions go on ref's skin.
static void rest_along(FxContact& c, const FxVec2f& s, const FxVec2f& e, const FxVec2f& n,
                       float ref_skin, const FxVec2f& inc0, const FxVec2f& inc1, float skins) {
    FxVec2f points[2];
    float height[2];
    if (!clip_to_span(inc0, inc1, s, e, n, points, height)) return;
    for (int i = 0; i < 2; ++i) {
        c.position[i] = points[i] - n * (height[i] - ref_skin);
        c.point_depth[i] = skins - height[i];
    }
    c.penetration_depth = std::max(c.point_depth[0], c.point_depth[1]);
    c.count = 2;
}

// The polygon face a contact with normal `out` (pointing away from the polygon) presses on, if
// it is near square to it; otherwise the contact is at a corner and keeps its single point.
static void rest_capsule_on_face(FxContact& c, const FxShape* capsule, const FxShape* poly,
                                 const FxVec2f& out) {
    const auto& pv = poly->vertices();
    const auto& cv = capsule->vertices();
    for (size_t i = 0, n = pv.size(); i < n; ++i) {
        const FxVec2f& s = pv[i];
        const FxVec2f& e = pv[(i + 1) % n];
        FxVec2f face_normal = (e - s).perp().normalized();
        if (face_normal.dot(s - poly->centroid()) < 0.0f) face_normal = -face_normal;
        if (face_normal.dot(out) < 0.99f) continue;
        rest_along(c, s, e, face_normal, poly->skin_radius(), cv[0], cv[1],
                   capsule->skin_radius() + poly->skin_radius());
        return;
    }
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
        c.point_depth[0] = -bsd[0];
        c.point_depth[1] = -bsd[1];
        c.count = 2;
    }
    return c;
}

// One SAT axis: the outward normal of A's edge i, and how far B reaches along it. Shared so the
// two SAT sweeps below differ only in what they keep, not in how an axis is computed.
struct FxSatAxis {
    FxVec2f dir; // along the edge, normalised
    FxVec2f axis; // outward normal
    float gap; // signed separation, both skins already subtracted
    size_t b_vertex; // B's deepest vertex on this axis
};

static FxSatAxis sat_axis(const FxVec2fArray& A_vertices, const FxShape* B_shape, size_t i,
                          size_t n, float skin_sum) {
    const FxVec2f& s = A_vertices[i];
    const FxVec2f& e = A_vertices[(i + 1) % n];
    FxSatAxis out;
    out.dir = (e - s).normalized();
    out.axis = out.dir.perp();
    const auto [b_min_idx, b_min_val] = B_shape->min_projection(out.axis, s);
    out.gap = b_min_val - skin_sum;
    out.b_vertex = b_min_idx;
    return out;
}

// tests A's edge normals against B; early-exits on first sep axis, else tracks min-penetration
// axis. Returned `gap` is skin-inclusive (raw B_min_val - rA - rB), so gap > 0 means full
// separation.
static FxSatResult sat_query(const FxShape* A_shape, const FxShape* B_shape) {
    const auto& A_vertices = A_shape->vertices();
    const float skin_sum = A_shape->skin_radius() + B_shape->skin_radius();
    FxSatResult result;
    for (size_t i = 0, n = A_vertices.size(); i < n; ++i) {
        const FxSatAxis a = sat_axis(A_vertices, B_shape, i, n, skin_sum);
        if (a.gap > 0.f) {
            result.has_sep = true;
            return result;
        } // separating axis
        // Keeps the largest gap: the axis of least penetration is the one to resolve along.
        if (a.gap > result.gap) {
            result.normal = a.axis;
            result.gap = a.gap;
            result.ref_edge_dir = a.dir;
            result.ref_edge_index = i;
            result.pen_vertex_index = a.b_vertex;
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

// Build the contact for a polygon pair whose SAT result is known: find B's incident edge, clip
// it against A's reference edge, place the points on B's skin surface. Split out from the SAT
// so it runs once -- only the winning direction's points are used, and this half is the costly one.
static FxContact polygon_contact_from_sat(const FxShape* A_shape, const FxShape* B_shape,
                                          const FxSatResult& sat) {
    FxContact contact(true);
    contact.normal = sat.normal;
    contact.penetration_depth = -sat.gap; // positive when overlapping (gap <= 0)

    const auto& A_vertices = A_shape->vertices();
    const auto& B_vertices = B_shape->vertices();
    if (B_vertices.empty()) return contact;

    const float rB = B_shape->skin_radius();
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
    // The points lie on A's reference edge; each one's depth is how far B's incident edge sits
    // below it there, skins included, as sat.gap measures. Linear along that edge.
    const float skin_sum = A_shape->skin_radius() + rB;
    const FxVec2f& u = sat.ref_edge_dir;
    const float t_start = (B_edge_start - A_edge_start).dot(u);
    const float t_end = (B_edge_end - A_edge_start).dot(u);
    const float d_start = skin_sum - (B_edge_start - A_edge_start).dot(sat.normal);
    const float d_end = skin_sum - (B_edge_end - A_edge_start).dot(sat.normal);
    auto depth_at = [&](const FxVec2f& q) {
        if (std::fabs(t_end - t_start) < 1e-6f) return contact.penetration_depth;
        const float s = ((q - A_edge_start).dot(u) - t_start) / (t_end - t_start);
        return d_start + s * (d_end - d_start);
    };
    contact.point_depth[0] = depth_at(contact_points.first);
    contact.point_depth[1] = depth_at(contact_points.second);
    // Check if points are too close and resolve to one point if needed
    float dist = (contact.position[1] - contact.position[0]).norm();
    contact.count = (dist < 0.01f) ? 1 : 2;
    return contact;
}

FxContact compute_contact_one_way(const FxShape* A_shape, const FxShape* B_shape) {
    auto contact = FxContact(true);
    contact.penetration_depth = 0.0f;

    // ----------- CHAIN -----------
    // One-sided (front = left of the authored direction). Joint contacts are clamped into the
    // arc the adjacent faces admit, or an endpoint normal could eject the body through it.
    if (A_shape->is_chain() || B_shape->is_chain()) {
        const bool a_is_chain = A_shape->is_chain();
        const FxShape* chain = a_is_chain ? A_shape : B_shape;
        const FxShape* other = a_is_chain ? B_shape : A_shape;
        if (other->is_chain() || other->is_edge()) {
            contact.set_valid(false);
            return contact;
        }

        const auto cv = chain->vertices();
        const std::size_t segments = chain->segment_count();

        auto front_of = [&](std::size_t i) {
            const FxVec2f d = cv[i + 1] - cv[i];
            const float len = d.norm();
            return (len > 1e-8f) ? FxVec2f{-d.y() / len, d.x() / len} : FxVec2f{0.f, 1.f};
        };

        if (other->is_circle() || other->is_capsule()) {
            const float r = other->skin_radius();
            const auto ov = other->vertices();

            float best_depth = -FxInfinityf;
            FxVec2f best_n{0.f, 1.f};
            FxVec2f best_p{0.f, 0.f};
            for (std::size_t i = 0; i < segments; ++i) {
                // Virtual circle centre: the capsule's is its core point nearest this segment.
                const FxVec2f c = other->is_circle() ?
                                      other->centroid() :
                                      seg_seg_closest(ov[0], ov[1], cv[i], cv[i + 1]).first;
                const FxVec2f f = front_of(i);
                if ((c - cv[i]).dot(f) < 0.0f) continue; // behind this face

                const FxVec2f p = FxClosestOnSegment(cv[i], cv[i + 1], c);
                const FxVec2f d = c - p;
                const float dist = d.norm();
                if (dist >= r) continue;
                FxVec2f n = (dist > 1e-6f) ? d / dist : f;
                float depth = r - dist;

                // Contact at an interior vertex: only a convex joint has a real corner arc,
                // and the normal must lie inside it; anything else clamps to the nearer face.
                const bool at_start = i > 0 && (p - cv[i]).norm() < 1e-5f;
                const bool at_end = i + 1 < segments && (p - cv[i + 1]).norm() < 1e-5f;
                if (at_start || at_end) {
                    const std::size_t j = at_start ? i : i + 1;
                    const FxVec2f fa = front_of(j - 1);
                    const FxVec2f fb = front_of(j);
                    const float turn = fa.cross(fb); // < 0 convex (crest), > 0 concave (valley)
                    const bool inside =
                        turn < -1e-6f && fa.cross(n) <= 1e-6f && n.cross(fb) <= 1e-6f;
                    if (!inside) {
                        n = (n.dot(fa) >= n.dot(fb)) ? fa : fb;
                        depth = r - (c - cv[j]).dot(n);
                        if (depth <= 0.0f) continue;
                    }
                }
                if (depth > best_depth) {
                    best_depth = depth;
                    best_n = n;
                    best_p = p;
                }
            }
            if (best_depth <= 0.0f) {
                contact.set_valid(false);
                return contact;
            }
            contact.count = 1;
            contact.penetration_depth = best_depth;
            contact.normal = a_is_chain ? best_n : -best_n; // keep the A -> B convention
            contact.position[0] = best_p;
            return contact;
        }

        // Polygons go segment by segment through the edge path, whose reference query already
        // reports face normals, and the deepest contact wins.
        const FxVec2f centre = other->centroid();
        FxContact best = FxContact(false);
        for (std::size_t i = 0; i < segments; ++i) {
            if ((centre - cv[i]).dot(front_of(i)) < 0.0f) continue;
            const FxShape seg = chain->segment(i);
            FxContact c = a_is_chain ? compute_contact_one_way(&seg, other) :
                                       compute_contact_one_way(other, &seg);
            if (!c.is_valid(false) || c.count == 0) continue;
            if (!best.is_valid(false) || c.penetration_depth > best.penetration_depth) best = c;
        }
        return best;
    }

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
                rest_along(contact, av[0], av[1], n, rA, bv[0], bv[1], rA + rB);
            } else contact.set_valid(false);
            return contact;
        }
        // Bare segment vs polygon needs the line-reference query, not the circle reduction.
        if (A_shape->is_edge()) return edge_polygon_contact(A_shape, B_shape);
        // Capsule vs polygon: closest seg-vs-polygon point, then circle-vs-polygon.
        auto [pCap, pPoly] = closest_capsule_to_polygon(A_shape, B_shape);
        FxContact c = circle_vs_polygon_contact(pCap, rA, B_shape);
        if (c.is_valid(false)) rest_capsule_on_face(c, A_shape, B_shape, -c.normal);
        return c;
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

    // Polygon vs Polygon (skin-aware via sat_query -- `sat.gap` already includes both skins)
    FxSatResult sat = sat_query(A_shape, B_shape);
    if (sat.has_sep) {
        contact.set_valid(false);
        return contact;
    }
    return polygon_contact_from_sat(A_shape, B_shape, sat);
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
        // Both directions are needed to find the smaller penetration, but only the winner's
        // points are used, so the SAT runs twice and the clip once. A separating axis from
        // either side settles it before the second SAT starts.
        const bool polygons = A->is_polygon() && B->is_polygon();
        FxSatResult satAB, satBA;
        if (polygons) {
            satAB = sat_query(A, B);
            if (satAB.has_sep) return FxContact(false);
            satBA = sat_query(B, A);
            if (satBA.has_sep) return FxContact(false);
        }

        const float depth_ab = polygons ? -satAB.gap : 0.0f;
        const float depth_ba = polygons ? -satBA.gap : 0.0f;
        // bias toward A->B: prevents jitter from flipping the reference edge each frame
        const float bias = 0.005f * depth_ab + 1e-6f;
        bool picked_ba = polygons && (depth_ba < depth_ab - bias);

        if (polygons) {
            contact = picked_ba ? polygon_contact_from_sat(B, A, satBA) :
                                  polygon_contact_from_sat(A, B, satAB);
            if (!contact.is_valid(false)) return FxContact(false);
        } else {
            // Chains and other non-polygon pairs still go through the general path.
            FxContact cAB = compute_contact_one_way(A, B);
            if (!cAB.is_valid(false)) return FxContact(false);
            FxContact cBA = compute_contact_one_way(B, A);
            if (!cBA.is_valid(false)) return FxContact(false);
            const float b2 = 0.005f * cAB.penetration_depth + 1e-6f;
            picked_ba = cBA.penetration_depth < cAB.penetration_depth - b2;
            contact = picked_ba ? cBA : cAB;
        }
        // one_way(B, A) reports B->A; chains skip the centre-to-centre fixup below, so restore
        // the convention here instead.
        if (picked_ba && (A->is_chain() || B->is_chain())) contact.normal = -contact.normal;
    }

    // Ensure normal points from entity1 -> entity2. Skipped for chains and edges: their entity
    // pose is wherever their points were authored from, so a centre-to-centre flip is arbitrary
    // and can invert correct normals. Their branches orient their own.
    if (contact.is_valid(false)) {
        const bool own_normal = A->is_chain() || B->is_chain() || A->is_edge() || B->is_edge();
        if (!own_normal) {
            FxVec2f delta = entity2->pose.xy() - entity1->pose.xy();
            if (delta.dot(contact.normal) < 0.0f) contact.normal = -contact.normal;
        }
        contact.entity1 = entity1.get();
        contact.entity2 = entity2.get();
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
        const FxSatAxis a = sat_axis(A_vertices, B_shape, i, n, skin_sum);
        // Keeps the smallest gap, and never exits early: a speculative contact needs the
        // closest approach across every axis, including ones that already separate.
        if (a.gap < best_val) {
            best_val = a.gap;
            best_normal = a.axis;
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
    if (A->is_edge() || B->is_edge() || A->is_chain() || B->is_chain()) return FxContact(false);
    FxVec2f rel_vel = entity2->velocity.xy() - entity1->velocity.xy();

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
    c.entity1 = entity1.get();
    c.entity2 = entity2.get();
    return c;
}

// How far a held manifold may drift before the narrow phase is asked again: either body turning
// more than kHoldTurn radians, or a point sliding more than kHoldSlide, since detection. A held
// contact is released once every point is more than kHoldRelease clear.
static constexpr float kHoldTurn = 0.02f;
static constexpr float kHoldSlide = 0.01f;
static constexpr float kHoldRelease = 0.01f;

// A circle looks the same at every angle, so a point on it is held relative to its centre and
// does not turn with the body; a rolling circle keeps its contact. Other shapes' points turn.
static bool turns_with_body(const FxEntity& e) {
    const auto& shape = e.collision_geometry();
    return !(shape && shape->is_circle());
}
static FxVec2f to_held_frame(const FxEntity& e, const FxVec2f& world_point) {
    return turns_with_body(e) ? e.to_entity_frame(world_point) : world_point - e.pose.xy();
}
static FxVec2f from_held_frame(const FxEntity& e, const FxVec2f& held_point) {
    return turns_with_body(e) ? e.to_world_frame(held_point) : e.pose.xy() + held_point;
}

void hold_contact(FxHeldContact& held, const FxContact& contact) {
    held.contact = contact;
    for (size_t i = 0; i < contact.count; ++i) {
        held.local1[i] = to_held_frame(*contact.entity1, contact.position[i]);
        held.local2[i] = to_held_frame(*contact.entity2, contact.position[i]);
    }
    for (size_t i = 0; i < contact.count; ++i) {
        const float d = contact.point_depth[i];
        held.detect_depth[i] = std::isfinite(d) ? d : contact.penetration_depth;
    }
    held.detect_theta1 = contact.entity1->pose.theta();
    held.detect_theta2 = contact.entity2->pose.theta();
    held.held = true;
}

bool remeasure_contact(FxHeldContact& held) {
    FxContact& c = held.contact;
    const FxEntity& A = *c.entity1;
    const FxEntity& B = *c.entity2;
    auto turned = [](const FxEntity& e, float theta0) {
        return turns_with_body(e) && std::fabs(FxAngleWrap(e.pose.theta() - theta0)) > kHoldTurn;
    };
    if (turned(A, held.detect_theta1) || turned(B, held.detect_theta2)) return false;

    // Each point's two anchors coincided at detection, so how far they have since moved apart
    // along the normal is how much shallower that point is. The normal stays as detected.
    const FxVec2f tangent = c.normal.perp();
    float deepest = -FxInfinityf;
    for (size_t i = 0; i < c.count; ++i) {
        const FxVec2f on_a = from_held_frame(A, held.local1[i]);
        const FxVec2f on_b = from_held_frame(B, held.local2[i]);
        const FxVec2f apart = on_b - on_a;
        if (std::fabs(apart.dot(tangent)) > kHoldSlide) return false;
        c.point_depth[i] = held.detect_depth[i] - apart.dot(c.normal);
        deepest = std::max(deepest, c.point_depth[i]);
        c.position[i] = on_b;
    }
    if (deepest < -kHoldRelease) return false;
    c.penetration_depth = deepest;
    return true;
}

} // namespace FxSolver
