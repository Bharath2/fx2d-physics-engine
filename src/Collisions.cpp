#include "Solver.h"
#include "Entity.h"
#include <limits>
#include <iostream>

namespace FxSolver {

    // Utility function to find intersection between two line segments
    std::unique_ptr<FxVec2f> getSegmentIntersection(
        const FxVec2f& p1, const FxVec2f& q1,
        const FxVec2f& p2, const FxVec2f& q2) {
        float epsilon = 1e-4;
        // Direction vectors
        FxVec2f dir1 = q1 - p1, dir2 = q2 - p2;
        float denom = dir1.cross(dir2);
        if (std::abs(denom) < epsilon) {  // Parallel or colinear
            return nullptr;
        }
        FxVec2f diff = p2 - p1;
        float t = diff.cross(dir2) / denom;
        float u = diff.cross(dir1) / denom;
        // Check if intersection lies on both segments
        if (t >= 0.0f && t <= 1.1f && u >= 0.0f && u <= 1.0f) {
            return std::make_unique<FxVec2f>(p1 + t * dir1);
        }
        return nullptr;
    }

    FxVec2f _projected_midpoint(const FxVec2f& p1, const FxVec2f& q1,
                                const FxVec2f& p2, const FxVec2f& q2) {
        FxVec2f d2 = q2 - p2;
        float len2_squared = d2.dot(d2);
        if (len2_squared <= 1e-12f) return p2;  // Degenerate second segment
        
        // Project p1 onto p2-q2
        FxVec2f v1 = p1 - p2;
        float t1 = v1.dot(d2) / len2_squared;
        t1 = std::clamp(t1, 0.0f, 1.0f);
        FxVec2f proj_p1 = p2 + t1 * d2;
        
        // Project q1 onto p2-q2
        FxVec2f v2 = q1 - p2;
        float t2 = v2.dot(d2) / len2_squared;
        t2 = std::clamp(t2, 0.0f, 1.0f);
        FxVec2f proj_q1 = p2 + t2 * d2;
        
        // Return midpoint of the two projections
        return (proj_p1 + proj_q1) * 0.5f;
    }

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

    FxContact compute_contact_one_way(const FxShape* m_shape, const FxShape* o_shape) {
        auto contact = FxContact(true); 
        contact.penetration_depth = 0.0f;
    
        // Circle vs Circle
        if (m_shape->is_circle() && o_shape->is_circle()) {
            FxVec2f c1 = m_shape->centroid(); 
            FxVec2f c2 = o_shape->centroid(); 
            FxVec2f d = c2 - c1;
            float dist = d.norm();
            float r1 = m_shape->radius(), r2 = o_shape->radius();
            float penetration = (r1 + r2) - dist;
            if (penetration > 0.f) {
                FxVec2f n = (dist > 1e-8f) ? d / dist : FxVec2f{1.f,0.f};
                contact.normal = n;                      // from m -> o
                contact.penetration_depth = penetration;
                contact.position = c1 + n * (r1 - 0.5f * penetration);
            } else contact.is_valid = false;
            return contact;
        }
    
        // Circle vs Polygon
        if (m_shape->is_circle() && !o_shape->is_circle()) {
            const auto& o_vertices = o_shape->vertices();
            if (o_vertices.empty()) { contact.is_valid = false; return contact; }
            FxVec2f c = m_shape->centroid();
            float r = m_shape->radius(), min_dist = FxInfinityf; 
            FxVec2f closest(0.0f, 0.0f);
            for (size_t i=0, n=o_vertices.size(); i<n; ++i) {
                const FxVec2f &s = o_vertices[i], &e = o_vertices[(i+1)%n];
                FxVec2f dir = e - s; 
                float edge_length = dir.dot(dir); 
                if (edge_length < 1e-8f) continue;
                float t = std::clamp((c - s).dot(dir) / edge_length, 0.f, 1.f);
                FxVec2f p = s + t * dir; 
                float d = (c - p).norm();
                if (d < min_dist) { 
                    min_dist = d; 
                    closest = p; }
            }
            if (min_dist == FxInfinityf) { contact.is_valid = false; return contact; }
            float penetration = r - min_dist;
            if (penetration > 0.f) {
                FxVec2f n = (min_dist > 1e-8f) ? (closest - c) / min_dist : FxVec2f{1.f,0.f}; // from circle -> polygon
                contact.normal = n;
                contact.penetration_depth = penetration;
                contact.position = closest;
            } else contact.is_valid = false;
            return contact;
        }
    
        // Polygon vs Circle (reuse, flip normal so it stays m->o)
        if (!m_shape->is_circle() && o_shape->is_circle()) {
            contact = compute_contact_one_way(o_shape, m_shape);
            if (contact.is_valid) { contact.normal = -contact.normal; } // now from polygon(m) -> circle(o)
            return contact;
        }
    
        // Polygon vs Polygon
        const auto &m_vertices = m_shape->vertices(); 
        const auto &o_vertices = o_shape->vertices();
        std::size_t m_ref_edge_index=0, o_pen_vertex_index=0; 
        FxVec2f m_ref_edge_dir{0, 0};
        float best_penetration = FxInfinityf;
        for (size_t i=0, n=m_vertices.size(); i<n; ++i) {
            const FxVec2f &s = m_vertices[i];
            const FxVec2f &e = m_vertices[(i+1)%n];
            FxVec2f dir = (e - s).normalized();
            FxVec2f axis = dir.perp();
            auto o_proj = o_shape->project_onto(axis, s);
            auto [omin_idx, omin_val] = o_proj.argmin();
            if (omin_val > 0.f) { // separating axis
                contact.is_valid = false; 
                return contact; 
            } 
            float pen = std::abs(omin_val);
            if (pen < best_penetration) {
                contact.normal = axis; best_penetration = pen; 
                m_ref_edge_dir = dir; m_ref_edge_index = i; 
                o_pen_vertex_index = omin_idx; 
            }
        }
        contact.penetration_depth = best_penetration;
    
        if (!o_vertices.empty()) {
            const size_t oN = o_vertices.size();
            FxVec2f v = o_vertices[o_pen_vertex_index];
            FxVec2f fwd = o_vertices[(o_pen_vertex_index+1)%oN] - v;
            FxVec2f bwd = v - o_vertices[(o_pen_vertex_index+oN-1)%oN];
            float dot_fwd = std::abs(fwd.normalized().dot(m_ref_edge_dir));
            float dot_bwd = std::abs(bwd.normalized().dot(m_ref_edge_dir));
            const FxVec2f &m_edge_start = m_vertices[m_ref_edge_index]; 
            const FxVec2f &m_edge_end = m_vertices[(m_ref_edge_index+1)%m_vertices.size()];
            float dot_ = dot_fwd;
            FxVec2f o_edge_start = v, o_seg_end = v + fwd;
            if (dot_bwd > dot_fwd) {
                dot_ = dot_bwd;
                o_edge_start = v - bwd; o_seg_end = v;
            }
            constexpr float eps = 5e-4f;
            if (dot_ > 1.f - eps) {
                contact.position = _projected_midpoint(o_edge_start, o_seg_end, m_edge_start, m_edge_end);
            } else {
                auto intersection = getSegmentIntersection(o_edge_start, o_seg_end, m_edge_start, m_edge_end);
                contact.position = intersection ? *intersection : v;
            }
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
        
        // // Returns contact normal and penetration depth  
        auto contact = FxContact(true);
        
        // the shapes are considered to be intersecting if they are not separated along any axis.
        const FxShape* m_shape = entity1->collision_geometry().get();
        const FxShape* o_shape = entity2->collision_geometry().get();
        // For circles, contact point is along the line joining centers
        if (m_shape->is_circle()) { 
            contact = compute_contact_one_way(m_shape, o_shape);
        } else if (o_shape->is_circle()) {
            contact = compute_contact_one_way(o_shape, m_shape);
        } else {
            auto contact1 = compute_contact_one_way(m_shape, o_shape);
            auto contact2 = compute_contact_one_way(o_shape, m_shape);
            // if either direction finds no collision
            if (!contact1.is_valid || !contact2.is_valid) {
                contact.is_valid = false; 
                return contact;
            }
            // minimum penetration is the actual contact
            contact = (contact1.penetration_depth <= contact2.penetration_depth) ? contact1 : contact2;
        }
        // Ensure normal points from entity1 to entity2
        FxVec2f delta = entity2->pose.xy() - entity1->pose.xy();
        if (delta.dot(contact.normal) < 0.0f) {
            contact.normal = -contact.normal;
        }

        // Store entity pointers
        contact.entity1 = entity1;
        contact.entity2 = entity2;
        
        return contact;
    }

} // namespace FxCollision 