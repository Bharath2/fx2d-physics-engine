// Spatial queries: ray casts, overlap and shape queries.
//
// Ray tests are done here rather than routed through the contact solver, because a ray is a
// degenerate shape the narrow phase deliberately does not handle (zero-thickness segments carry
// no volume to resolve). Overlap tests go the other way and reuse FxSolver::collision_check
// verbatim, so a query and a simulated contact can never disagree about what is touching.

#include "Fx2D/Scene.h"

#include <algorithm>
#include <cmath>

namespace {

constexpr float kEps = 1e-8f;

// Nearest positive root of |origin + t*dir - centre| = radius, or -1 for a miss.
// dir must be unit length. A ray starting inside reports 0.
float ray_circle(const FxVec2f& origin, const FxVec2f& dir, const FxVec2f& centre, float radius) {
    const FxVec2f m = origin - centre;
    const float b = m.dot(dir);
    const float c = m.dot(m) - radius * radius;
    if (c <= 0.0f) return 0.0f; // started inside
    const float disc = b * b - c;
    if (disc < 0.0f) return -1.0f;
    const float root = std::sqrt(disc);
    const float t = -b - root;
    return (t >= 0.0f) ? t : -1.0f; // the far root is behind us if the near one is
}

// Distance along the ray to segment [a, b], or -1. dir must be unit length.
float ray_segment(const FxVec2f& origin, const FxVec2f& dir, const FxVec2f& a, const FxVec2f& b) {
    const FxVec2f s = b - a;
    const float denom = dir.cross(s);
    if (std::fabs(denom) < kEps) return -1.0f; // parallel
    const FxVec2f ao = a - origin;
    const float t = ao.cross(s) / denom;
    const float u = ao.cross(dir) / denom;
    if (t < 0.0f || u < 0.0f || u > 1.0f) return -1.0f;
    return t;
}

// Twice the signed area of the vertex loop: positive for counter-clockwise winding.
float signed_area(const FxVec2fArray& verts) {
    float total = 0.0f;
    const size_t n = verts.size();
    for (size_t i = 0; i < n; ++i) {
        const FxVec2f& a = verts[i];
        const FxVec2f& b = verts[(i + 1) % n];
        total += a.cross(b);
    }
    return total;
}

// Is the point within the shape, skin included?
bool point_inside(const FxShape& shape, const FxVec2f& p) {
    const float skin = shape.skin_radius();
    const FxVec2fArray verts = shape.vertices();
    const size_t n = verts.size();

    if (n == 0) return (p - shape.centroid()).norm() <= skin; // circle

    if (n == 2) { // capsule or edge: within `skin` of the segment
        const FxVec2f a = verts[0], b = verts[1];
        const FxVec2f ab = b - a;
        const float len2 = ab.dot(ab);
        const float u = (len2 > kEps) ? std::clamp((p - a).dot(ab) / len2, 0.0f, 1.0f) : 0.0f;
        return (p - (a + ab * u)).norm() <= skin;
    }

    // Convex polygon: inside when the point sits on the same side of every edge. Accepting
    // either sign keeps this winding-agnostic. The skin is handled by allowing the point to sit
    // slightly outside each edge, which is exact away from the corners and conservative at them.
    bool all_left = true, all_right = true;
    for (size_t i = 0; i < n; ++i) {
        const FxVec2f a = verts[i];
        const FxVec2f b = verts[(i + 1) % n];
        const FxVec2f edge = b - a;
        const float len = edge.norm();
        if (len < kEps) continue;
        const float side = edge.cross(p - a) / len; // signed distance from the edge line
        if (side < -skin) all_left = false;
        if (side > skin) all_right = false;
    }
    return all_left || all_right;
}

// Ray against one shape, already positioned in the world. Returns distance or -1, and fills the
// surface normal.
//
// Every shape is vertices + skin radius, so the boundary is the vertex core offset outward by
// the skin: each edge becomes a segment pushed out along its outward normal, and each vertex
// becomes an arc of radius `skin`. Testing the offset edges and the vertex circles and keeping
// the nearest hit therefore handles circles, capsules, edges, polygons and rounded polygons
// through one path. With skin 0 it degenerates to plain ray-versus-edge, which is exactly right
// for a sharp polygon.
float ray_shape(const FxShape& shape, const FxVec2f& origin, const FxVec2f& dir,
                FxVec2f& out_normal) {
    const float skin = shape.skin_radius();
    const FxVec2fArray verts = shape.vertices();

    if (verts.size() == 0) { // circle
        const FxVec2f centre = shape.centroid();
        const float t = ray_circle(origin, dir, centre, skin);
        if (t < 0.0f) return -1.0f;
        const FxVec2f p = origin + dir * t;
        FxVec2f n = p - centre;
        const float len = n.norm();
        out_normal = (len > kEps) ? (n / len) : -dir;
        return t;
    }

    float best = -1.0f;
    FxVec2f best_normal{0.0f, 0.0f};
    const size_t n = verts.size();

    // A ray that begins inside the shape reports the shape immediately, so picking works when
    // the cursor is over a body rather than only when it is outside looking in.
    if (point_inside(shape, origin)) {
        out_normal = -dir;
        return 0.0f;
    }

    // Which way is out depends on the winding, and the two shape constructors do not agree, so
    // measure it rather than assume. A 2-vertex capsule has no enclosed area: its edge list is
    // v0->v1 and v1->v0, whose right perps already point to opposite sides, so it needs no flip.
    const float outward_sign = (n >= 3 && signed_area(verts) < 0.0f) ? -1.0f : 1.0f;

    for (size_t i = 0; i < n; ++i) {
        const FxVec2f a = verts[i];
        const FxVec2f b = verts[(i + 1) % n];
        const FxVec2f edge = b - a;
        const float len = edge.norm();
        if (len < kEps) continue;

        const FxVec2f outward{outward_sign * edge.y() / len, -outward_sign * edge.x() / len};
        const FxVec2f oa = a + outward * skin;
        const FxVec2f ob = b + outward * skin;

        const float t = ray_segment(origin, dir, oa, ob);
        if (t >= 0.0f && (best < 0.0f || t < best)) {
            best = t;
            best_normal = outward;
        }
    }

    if (skin > kEps) {
        for (size_t i = 0; i < n; ++i) {
            const float t = ray_circle(origin, dir, verts[i], skin);
            if (t >= 0.0f && (best < 0.0f || t < best)) {
                best = t;
                FxVec2f nrm = (origin + dir * t) - verts[i];
                const float len = nrm.norm();
                best_normal = (len > kEps) ? (nrm / len) : -dir;
                best = t;
            }
        }
    }

    if (best < 0.0f) return -1.0f;
    out_normal = best_normal;
    return best;
}

// Cheap rejection: does the ray come within the shape's bounding circle inside max_distance?
//
// Deliberately derived from the shape itself (centroid and skin-inclusive bounding radius)
// rather than FxEntity::bounding_box(), which is a cache refreshed only by step() and reset().
// A query must answer correctly for a body that was just moved by hand, and before the scene
// has ever stepped.
bool ray_reaches(const FxVec2f& origin, const FxVec2f& dir, float max_distance,
                 const FxShape& shape) {
    const FxVec2f centre = shape.centroid();
    const float radius = shape.radius();
    const float along = (centre - origin).dot(dir);
    if (along > max_distance + radius) return false; // beyond the end of the ray
    const FxVec2f nearest_point = origin + dir * std::max(along, 0.0f);
    return (centre - nearest_point).norm() <= radius;
}

bool queryable(const std::shared_ptr<FxEntity>& e) {
    return e && e->enabled && e->collision_geometry();
}

// A throwaway body carrying the query shape, so overlap tests can go through the very same
// narrow phase the simulation uses instead of a parallel implementation that could disagree.
std::shared_ptr<FxEntity> make_probe(const FxShape& shape, const FxVec3f& pose) {
    auto probe = std::make_shared<FxEntity>("fx_query_probe");
    probe->set_init_pose(pose);
    probe->pose = pose;
    probe->prev_pose = pose;
    probe->set_collision_geometry(FxCollisionShape(shape));
    probe->collision_geometry()->set_world_pose(pose);
    return probe;
}

} // namespace

void FxScene::raycast_all(const FxVec2f& origin, const FxVec2f& direction, float max_distance,
                          std::vector<FxRayHit>& out_hits) const {
    out_hits.clear();
    const float dir_len = direction.norm();
    if (dir_len < kEps || max_distance <= 0.0f) return;
    const FxVec2f dir = direction / dir_len;

    for (const auto& entity : m_entities.items()) {
        if (!queryable(entity)) continue;
        const FxShape& shape = *entity->collision_geometry();
        if (!ray_reaches(origin, dir, max_distance, shape)) continue;

        FxVec2f normal{0.0f, 0.0f};
        const float t = ray_shape(shape, origin, dir, normal);
        if (t < 0.0f || t > max_distance) continue;

        FxRayHit hit;
        hit.entity = entity;
        hit.distance = t;
        hit.point = origin + dir * t;
        // Always report the face the ray arrived at, even for a grazing or inside-out hit.
        hit.normal = (normal.dot(dir) > 0.0f) ? -normal : normal;
        out_hits.push_back(hit);
    }

    std::sort(out_hits.begin(), out_hits.end(),
              [](const FxRayHit& a, const FxRayHit& b) { return a.distance < b.distance; });
}

bool FxScene::raycast(const FxVec2f& origin, const FxVec2f& direction, float max_distance,
                      FxRayHit& out_hit) const {
    std::vector<FxRayHit> hits;
    raycast_all(origin, direction, max_distance, hits);
    if (hits.empty()) return false;
    out_hit = hits.front();
    return true;
}

void FxScene::overlap_shape(const FxShape& shape, const FxVec3f& pose,
                            std::vector<std::shared_ptr<FxEntity>>& out) const {
    out.clear();
    auto probe = make_probe(shape, pose);

    for (const auto& entity : m_entities.items()) {
        if (!queryable(entity)) continue;
        // No bounding-box pre-filter here for the same reason as the ray path: it would read a
        // cache that step() maintains, and a query must be right between steps too.
        const FxShape& entity_shape = *entity->collision_geometry();
        if (FxSolver::collision_check(probe, entity).is_valid()) {
            out.push_back(entity);
            continue;
        }
        // The narrow phase is built to separate overlapping bodies, so it reports nothing when
        // one shape is wholly inside the other — there is no penetration axis, and coincident
        // centres give it no normal to work with. That is fine for solving contacts and wrong
        // for a query, where full containment is the most emphatic kind of overlap there is.
        if (point_inside(*probe->collision_geometry(), entity_shape.centroid()) ||
            point_inside(entity_shape, probe->collision_geometry()->centroid())) {
            out.push_back(entity);
        }
    }
}

void FxScene::overlap_circle(const FxVec2f& centre, float radius,
                             std::vector<std::shared_ptr<FxEntity>>& out) const {
    out.clear();
    if (radius <= 0.0f) return;
    overlap_shape(FxShape(radius), FxVec3f{centre.x(), centre.y(), 0.0f}, out);
}

void FxScene::overlap_box(const FxVec2f& centre, const FxVec2f& extents,
                          std::vector<std::shared_ptr<FxEntity>>& out) const {
    out.clear();
    if (extents.x() <= 0.0f || extents.y() <= 0.0f) return;
    overlap_shape(FxShape(extents), FxVec3f{centre.x(), centre.y(), 0.0f}, out);
}

void FxScene::overlap_point(const FxVec2f& point,
                            std::vector<std::shared_ptr<FxEntity>>& out) const {
    // A point is not a shape the narrow phase accepts, so probe with the smallest circle the
    // shape constructor will build. The radius is far below any sane scene scale.
    overlap_circle(point, 1e-4f, out);
}

std::shared_ptr<FxEntity> FxScene::entity_at_point(const FxVec2f& point) const {
    std::vector<std::shared_ptr<FxEntity>> hits;
    overlap_point(point, hits);
    return hits.empty() ? nullptr : hits.front();
}
