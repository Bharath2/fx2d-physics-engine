#include "Fx2D/Solver.h"
#include "Fx2D/Entity.h"
#include <limits>
#include <iostream>

namespace FxSolver {
    
    // AABB overlap check
    bool aabb_overlap_check(const FxEntity& entity1, const FxEntity& entity2) {
        auto aabb1 = entity1.bounding_box();
        auto aabb2 = entity2.bounding_box();
        // check if they are overlapping
        return !(aabb1(2) < aabb2(0) || aabb2(2) < aabb1(0) || // this.maxX < other.minX or other.maxX < this.minX
                 aabb1(3) < aabb2(1) || aabb2(3) < aabb1(1));  // this.maxY < other.minY or  other.maxY < this.minY
    }

    // Overload for shared_ptr, delegates to object reference version
    bool aabb_overlap_check(const std::shared_ptr<FxEntity>& entity1, const std::shared_ptr<FxEntity>& entity2) {
        if (!entity1 || !entity2) return false;
        return aabb_overlap_check(*entity1, *entity2);
    }


    std::pair<FxVec2f, FxVec2f> clip_edge(const FxVec2f& p1, const FxVec2f& q1,
                                          const FxVec2f& p2, const FxVec2f& q2) {
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

    // Closest point on segment [A,B] to point P.
    static FxVec2f closest_pt_on_seg(const FxVec2f& P, const FxVec2f& A, const FxVec2f& B) {
        FxVec2f AB = B - A;
        float len2 = AB.dot(AB);
        if (len2 < 1e-6f) return A;
        float t = std::clamp((P - A).dot(AB) / len2, 0.f, 1.f);
        return A + t * AB;
    }

    // Closest point pair between segments [A0,A1] and [B0,B1].
    static std::pair<FxVec2f, FxVec2f> closest_pts_seg_seg(
        const FxVec2f& A0, const FxVec2f& A1,
        const FxVec2f& B0, const FxVec2f& B1)
    {
        FxVec2f d1 = A1 - A0, d2 = B1 - B0, r = A0 - B0;
        float a = d1.dot(d1), e = d2.dot(d2), f = d2.dot(r);
        float s, t;
        if (a < 1e-6f && e < 1e-6f) return {A0, B0};
        if (a < 1e-6f) {
            s = 0.f; t = std::clamp(f / e, 0.f, 1.f);
        } else {
            float c = d1.dot(r);
            if (e < 1e-6f) {
                t = 0.f; s = std::clamp(-c / a, 0.f, 1.f);
            } else {
                float b = d1.dot(d2);
                float denom = a * e - b * b;
                s = (std::abs(denom) > 1e-6f) ? std::clamp((b * f - c * e) / denom, 0.f, 1.f) : 0.f;
                t = (b * s + f) / e;
                if (t < 0.f) { t = 0.f; s = std::clamp(-c / a, 0.f, 1.f); }
                else if (t > 1.f) { t = 1.f; s = std::clamp((b - c) / a, 0.f, 1.f); }
            }
        }
        return {A0 + s * d1, B0 + t * d2};
    }

    // Edge (or capsule) vs circle contact. Normal: segment surface → circle center.
    static FxContact capsule_circle_contact(const FxShape* cap, const FxShape* circ) {
        const auto& verts = cap->vertices();
        FxVec2f Q = closest_pt_on_seg(circ->centroid(), verts[0], verts[1]);
        FxVec2f d = circ->centroid() - Q;
        float dist = d.norm();
        float pen  = cap->radius() + circ->radius() - dist;
        FxContact c(true);
        if (pen > 0.f) {
            FxVec2f n = (dist > 1e-6f) ? d / dist : FxVec2f{0.f, 1.f};
            c.normal = n;
            c.penetration_depth = pen;
            c.position[0] = Q + n * cap->radius();
            c.count = 1;
        } else { c.set_valid(false); }
        return c;
    }

    // Capsule (or edge) vs capsule (or edge) contact. Normal: segment-A surface → segment-B.
    static FxContact capsule_capsule_contact(const FxShape* capA, const FxShape* capB) {
        const auto& va = capA->vertices();
        const auto& vb = capB->vertices();
        auto [pA, pB] = closest_pts_seg_seg(va[0], va[1], vb[0], vb[1]);
        FxVec2f d = pB - pA;
        float dist = d.norm();
        float pen  = capA->radius() + capB->radius() - dist;
        FxContact c(true);
        if (pen > 0.f) {
            FxVec2f n = (dist > 1e-6f) ? d / dist : FxVec2f{0.f, 1.f};
            c.normal = n;
            c.penetration_depth = pen;
            c.position[0] = pA + n * capA->radius();
            c.count = 1;
        } else { c.set_valid(false); }
        return c;
    }

    // Capsule vs polygon contact: minimum distance between core segment and polygon boundary.
    static FxContact capsule_polygon_contact(const FxShape* cap, const FxShape* poly) {
        const auto& cv = cap->vertices();
        const auto& pv = poly->vertices();
        size_t nP = pv.size();
        if (nP < 2) return FxContact(false);
        float   rCap     = cap->radius();
        float   min_dist = FxInfinityf;
        FxVec2f best_cap_pt{0.f, 0.f}, best_poly_pt{0.f, 0.f};
        for (size_t i = 0; i < nP; ++i) {
            auto [pc, pp] = closest_pts_seg_seg(cv[0], cv[1], pv[i], pv[(i+1)%nP]);
            float d = (pc - pp).norm();
            if (d < min_dist) { min_dist = d; best_cap_pt = pc; best_poly_pt = pp; }
        }
        float pen = rCap - min_dist;
        FxContact c(true);
        if (pen > 0.f) {
            FxVec2f dv = best_poly_pt - best_cap_pt;
            FxVec2f n  = (min_dist > 1e-6f) ? dv / min_dist : FxVec2f{0.f, 1.f};
            c.normal = n;
            c.penetration_depth = pen;
            c.position[0] = best_cap_pt + n * rCap;
            c.count = 1;
        } else { c.set_valid(false); }
        return c;
    }

    // Dedicated edge↔polygon contact (capsule_polygon_contact early-bails at rCap=0 — D2/D10).
    // Normal points from the edge toward the polygon; collision_check re-orients to entity1→entity2.
    static FxContact edge_polygon_contact(const FxShape* edge, const FxShape* poly) {
        const auto& ev = edge->vertices();
        FxVec2f E0 = ev[0], E1 = ev[1];
        FxVec2f eu = E1 - E0;
        float L = eu.norm();
        if (L < 1e-6f) return FxContact(false);
        eu /= L;

        FxVec2f n = eu.perp();
        if ((poly->centroid() - E0).dot(n) < 0.f) n = -n;  // orient toward polygon

        const auto& pv = poly->vertices();
        size_t nP = pv.size();
        if (nP == 0) return FxContact(false);

        float deepest_sd = FxInfinityf;
        float poly_min_t = FxInfinityf, poly_max_t = -FxInfinityf;
        for (size_t i = 0; i < nP; ++i) {
            float sd = (pv[i] - E0).dot(n);
            if (sd < deepest_sd) deepest_sd = sd;
            float t = (pv[i] - E0).dot(eu);
            if (t < poly_min_t) poly_min_t = t;
            if (t > poly_max_t) poly_max_t = t;
        }
        if (deepest_sd >= 0.f) return FxContact(false);
        if (poly_max_t < 0.f || poly_min_t > L) return FxContact(false);

        // Collect up to 2 deepest penetrating vertices within segment span [0, L]
        FxVec2f best[2] = {FxVec2f{0.f, 0.f}, FxVec2f{0.f, 0.f}};
        float   bsd[2]  = {0.f, 0.f};
        int     nb = 0;
        for (size_t i = 0; i < nP; ++i) {
            float sd = (pv[i] - E0).dot(n);
            if (sd >= 0.f) continue;
            float t = (pv[i] - E0).dot(eu);
            if (t < 0.f || t > L) continue;
            FxVec2f cpt = E0 + t * eu;
            if (nb < 2) {
                best[nb] = cpt; bsd[nb] = sd; ++nb;
            } else {
                int sh = (bsd[0] > bsd[1]) ? 0 : 1;  // index of shallowest stored
                if (sd < bsd[sh]) { best[sh] = cpt; bsd[sh] = sd; }
            }
        }
        if (nb == 0) return FxContact(false);

        FxContact c(true);
        c.normal = n;
        c.penetration_depth = -bsd[0];
        c.position[0] = best[0];
        if (nb == 2) {
            float dist = (best[1] - best[0]).norm();
            if (dist >= 0.01f) { c.position[1] = best[1]; c.count = 2; }
            else               { c.count = 1; }
        } else {
            c.count = 1;
        }
        return c;
    }

    // tests A's edge normals against B; early-exits on first sep axis, else tracks min-penetration axis
    static FxSatResult sat_query(const FxShape* A_shape, const FxShape* B_shape) {
        const auto &A_vertices = A_shape->vertices();
        FxSatResult result;
        for (size_t i=0, n = A_vertices.size(); i < n; ++i) {
            const FxVec2f &s = A_vertices[i];
            const FxVec2f &e = A_vertices[(i+1)%n];
            FxVec2f dir = (e - s).normalized();
            FxVec2f axis = dir.perp();
            auto [B_min_idx, B_min_val] = B_shape->project_onto(axis, s).argmin();
            if (B_min_val > 0.f) { result.has_sep = true; return result; } // separating axis
            if (B_min_val > result.gap) {
                result.normal           = axis;
                result.gap              = B_min_val;
                result.ref_edge_dir     = dir;
                result.ref_edge_index   = i;
                result.pen_vertex_index = B_min_idx;
            }
        }
        return result;
    }

    FxContact compute_contact_one_way(const FxShape* A_shape, const FxShape* B_shape) {
        auto contact = FxContact(true);
        contact.penetration_depth = 0.0f;

        // Segment shapes (capsule/edge) are handled by dedicated helpers in collision_check.
        if (A_shape->has_core_segment() || B_shape->has_core_segment()) {
            contact.set_valid(false); return contact;
        }

        // Circle vs Circle
        if (A_shape->is_circle() && B_shape->is_circle()) {
            FxVec2f cA = A_shape->centroid(); 
            FxVec2f cB = B_shape->centroid(); 
            FxVec2f d = cB - cA;
            float dist = d.norm();
            float rA = A_shape->radius(), rB = B_shape->radius();
            float penetration = (rA + rB) - dist;
            if (penetration > 0.f) {
                FxVec2f n = (dist > 1e-6f) ? d / dist : FxVec2f{1.f,0.f};
                contact.normal = n;                      // from A -> B
                contact.penetration_depth = penetration;
                contact.position[0] = cA + n * (rA - 0.5f * penetration);
                contact.count = 1;
            } else contact.set_valid(false);
            return contact;
        }
    
        // Circle vs Polygon
        if (A_shape->is_circle() && !B_shape->is_circle()) {
            const auto& B_vertices = B_shape->vertices();
            if (B_vertices.empty()) { 
                contact.set_valid(false); return contact; 
            }
            FxVec2f cA = A_shape->centroid();
            float rA = A_shape->radius();
            float min_dist = FxInfinityf; 
            FxVec2f closest(0.0f, 0.0f);
            for (size_t i=0, n = B_vertices.size(); i < n; ++i) {
                const FxVec2f &s = B_vertices[i]; 
                const FxVec2f &e = B_vertices[(i+1)%n];
                FxVec2f dir = e - s; 
                float edge_length = dir.dot(dir); 
                if (edge_length < 1e-6f) continue;
                float t = std::clamp((cA - s).dot(dir) / edge_length, 0.f, 1.f);
                FxVec2f p = s + t * dir; 
                float d = (cA - p).norm();
                if (d < min_dist) { 
                    min_dist = d; 
                    closest = p; }
            }
            if (min_dist == FxInfinityf) { 
                contact.set_valid(false); return contact; 
            }
            float penetration = rA - min_dist;
            if (penetration > 0.f) {
                FxVec2f n = (min_dist > 1e-6f) ? (closest - cA) / min_dist : FxVec2f{1.f,0.f}; // from circle -> polygon
                contact.normal = n;
                contact.penetration_depth = penetration;
                contact.position[0] = closest;
                contact.count = 1;
            } else contact.set_valid(false);
            return contact;
        }
    
        // Polygon vs Circle (reuse, flip normal so it stays A->B)
        if (!A_shape->is_circle() && B_shape->is_circle()) {
            contact = compute_contact_one_way(B_shape, A_shape);
            if (contact.is_valid(false)) { contact.normal = -contact.normal; } // now from polygon(A) -> circle(B)
            return contact;
        }
    
        // Polygon vs Polygon
        const auto &A_vertices = A_shape->vertices(); 
        const auto &B_vertices = B_shape->vertices();
        FxSatResult sat = sat_query(A_shape, B_shape);
        if (sat.has_sep) { contact.set_valid(false); return contact; } // separating axis found
        contact.normal            = sat.normal;
        contact.penetration_depth = std::abs(sat.gap);
    
        if (!B_vertices.empty()) {
            const size_t B_N = B_vertices.size();
            const size_t ifwd = (sat.pen_vertex_index + 1)%B_N;
            const size_t ibwd = (sat.pen_vertex_index + B_N - 1)%B_N;
            const FxVec2f B_edge_start = B_vertices[sat.pen_vertex_index];
            const float dot_fwd = std::abs((B_vertices[ifwd] - B_edge_start).normalized().dot(sat.ref_edge_dir));
            const float dot_bwd = std::abs((B_edge_start - B_vertices[ibwd]).normalized().dot(sat.ref_edge_dir));
            const FxVec2f B_edge_end = dot_bwd > dot_fwd ? B_vertices[ibwd] : B_vertices[ifwd];
            const FxVec2f &A_edge_start = A_vertices[sat.ref_edge_index]; 
            const FxVec2f &A_edge_end = A_vertices[(sat.ref_edge_index+1)%A_vertices.size()];
            const auto contact_points = clip_edge(B_edge_start, B_edge_end, A_edge_start, A_edge_end);
            contact.position[0] = contact_points.first;
            contact.position[1] = contact_points.second;
            // Check if points are too close and resolve to one point if needed
            float dist = (contact.position[1] - contact.position[0]).norm();
            contact.count = (dist < 0.01f) ? 1 : 2;
        }
        return contact;
    }

    // Separating Axis Theorem collision check method
    const FxContact collision_check(const std::shared_ptr<FxEntity>& entity1, const std::shared_ptr<FxEntity>& entity2) {
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
        
        // Segment-shape dispatch (capsule or edge via has_core_segment())
        auto seg = [](const FxShape* s) { return s->has_core_segment(); };
        if (seg(A) || seg(B)) {
            if (seg(A) && seg(B)) {
                if (A->is_edge() && B->is_edge()) {
                    // lean: no edge-edge contact — ceiling: dynamic edge-edge — upgrade: when edge-edge physics needed
                    return FxContact(false);
                }
                contact = capsule_capsule_contact(A, B);
            } else {
                const FxShape* seg_s = seg(A) ? A : B;
                const FxShape* other = seg(A) ? B : A;
                if (other->is_circle()) {
                    contact = capsule_circle_contact(seg_s, other);
                } else if (seg_s->is_edge()) {
                    contact = edge_polygon_contact(seg_s, other);
                } else {
                    contact = capsule_polygon_contact(seg_s, other);
                }
            }
        } else if (A->is_circle()) {
            contact = compute_contact_one_way(A, B);
        } else if (B->is_circle()) {
            contact = compute_contact_one_way(B, A);
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
            contact.normal  = contact.normal.normalized();
        }
        return contact;
    }

    // no-early-exit SAT: returns the axis of minimum signed separation for speculative contact
    static std::pair<FxVec2f, float> sat_gap_query(const FxShape* A_shape, const FxShape* B_shape) {
        const auto &A_vertices = A_shape->vertices();
        FxVec2f best_normal{1.0f, 0.0f};
        float   best_val = FxInfinityf;
        for (size_t i=0, n = A_vertices.size(); i < n; ++i) {
            const FxVec2f &s = A_vertices[i];
            const FxVec2f &e = A_vertices[(i+1)%n];
            FxVec2f axis = (e - s).normalized().perp();
            auto [B_min_idx, B_min_val] = B_shape->project_onto(axis, s).argmin();
            if (B_min_val < best_val) { best_val = B_min_val; best_normal = axis; }
        }
        return {best_normal, best_val};
    }

    // speculative contact for CCD: returns pre-contact (gap < 0) when bodies will collide this substep
    FxContact speculative_contact_check(const std::shared_ptr<FxEntity>& entity1,
                                        const std::shared_ptr<FxEntity>& entity2,
                                        float substep_dt) {
        if (!entity1 || !entity2) return FxContact(false);
        if (!entity1->collision_geometry() || !entity2->collision_geometry()) return FxContact(false);
        if (substep_dt <= 0.0f) return FxContact(false);

        const FxShape* A = entity1->collision_geometry().get();
        const FxShape* B = entity2->collision_geometry().get();
        // Edges are static; CCD not needed for segment shapes (D5).
        if (A->has_core_segment() || B->has_core_segment()) return FxContact(false);
        FxVec2f rel_vel = entity2->velocity.head<2>() - entity1->velocity.head<2>();

        FxVec2f normal;
        float   gap;

        if (A->is_circle() && B->is_circle()) {
            // Exact axis for circle-circle
            FxVec2f delta = entity2->pose.xy() - entity1->pose.xy();
            float dist = delta.norm();
            if (dist < 1e-6f) return FxContact(false);
            normal = delta / dist;
            gap    = dist - A->radius() - B->radius();
        } else if (A->is_circle() || B->is_circle()) {
            // Closest point on the polygon edge to the circle center
            const FxShape* circle  = A->is_circle() ? A : B;
            const FxShape* polygon = A->is_circle() ? B : A;
            FxVec2f cC = circle->centroid();
            const auto& verts = polygon->vertices();
            float   min_dist = FxInfinityf;
            FxVec2f closest{};
            for (size_t i = 0, n = verts.size(); i < n; ++i) {
                const FxVec2f& s = verts[i];
                const FxVec2f& e = verts[(i + 1) % n];
                FxVec2f dir = e - s;
                float   len2 = dir.dot(dir);
                if (len2 < 1e-6f) continue;
                float t = std::clamp((cC - s).dot(dir) / len2, 0.f, 1.f);
                FxVec2f p = s + t * dir;
                float d = (cC - p).norm();
                if (d < min_dist) { min_dist = d; closest = p; }
            }
            if (min_dist == FxInfinityf) return FxContact(false);
            // Normal from circle surface → polygon (direction from circle center to closest point)
            FxVec2f raw = (min_dist > 1e-6f) ? (closest - cC) / min_dist : FxVec2f{1.f, 0.f};
            normal = A->is_circle() ? raw : -raw;  // keep entity1 → entity2 convention
            gap    = min_dist - circle->radius();
        } else {
            // Polygon-polygon: run gap queries both ways, pick the axis with the smallest gap
            auto [nAB, gAB] = sat_gap_query(A, B);
            auto [nBA, gBA] = sat_gap_query(B, A);
            if (gAB <= gBA) { normal = nAB;  gap = gAB; }
            else            { normal = -nBA; gap = gBA; } // flip B->A to A->B convention
        }

        // Ensure normal points from entity1 → entity2
        FxVec2f c2c = entity2->pose.xy() - entity1->pose.xy();
        if (c2c.dot(normal) < 0.0f) normal = -normal;

        // Closing speed along the contact normal (positive = approaching)
        float v_closing = -rel_vel.dot(normal);
        if (v_closing <= 0.0f) return FxContact(false);  // separating

        // Pre-contact depth: negative means the gap will close this substep
        float spec_depth = gap - v_closing * substep_dt;
        if (spec_depth >= 0.0f) return FxContact(false);

        FxContact c(true);
        c.count             = 1;
        c.normal            = normal;
        c.penetration_depth = spec_depth;
        c.position[0]       = entity1->pose.xy() + normal * A->radius();
        c.entity1           = entity1;
        c.entity2           = entity2;
        return c;
    }

} // namespace FxSolver