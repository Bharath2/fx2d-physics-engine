#pragma once

#include "Fx2D/Math.h"

#include <memory>

// Geometry built on the vector and array types in Math.h: bounding boxes, the unified shape,
// and the primitive queries that operate on them.

// Closest point on segment [a,b] to p.
static inline FxVec2f FxClosestOnSegment(const FxVec2f& a, const FxVec2f& b, const FxVec2f& p) {
    FxVec2f ab = b - a;
    float len2 = ab.dot(ab);
    if (len2 < 1e-12f) return a;
    float t = std::clamp((p - a).dot(ab) / len2, 0.0f, 1.0f);
    return a + t * ab;
}

// Axis-aligned bounding box in 2D world coordinates
struct FxAABB {
    float minX = 0.0f, minY = 0.0f, maxX = 0.0f, maxY = 0.0f;
    FxAABB() = default;
    FxAABB(float mnX, float mnY, float mxX, float mxY) :
        minX(mnX), minY(mnY), maxX(mxX), maxY(mxY) {}
    static FxAABB combine(const FxAABB& a, const FxAABB& b) {
        return {std::min(a.minX, b.minX), std::min(a.minY, b.minY), std::max(a.maxX, b.maxX),
                std::max(a.maxY, b.maxY)};
    }
    FxAABB fatten(float margin) const {
        return {minX - margin, minY - margin, maxX + margin, maxY + margin};
    }
    float perimeter() const { return (maxX - minX) + (maxY - minY); }
    bool overlaps(const FxAABB& o) const {
        return maxX >= o.minX && o.maxX >= minX && maxY >= o.minY && o.maxY >= minY;
    }
    bool contains(const FxAABB& inner) const {
        return minX <= inner.minX && minY <= inner.minY && maxX >= inner.maxX && maxY >= inner.maxY;
    }
    bool is_valid() const { return maxX > minX && maxY > minY; }
};

// Custom 2x2 float matrix with .a(), .b(), .c(), .d() getters and corresponding setters.
// Unified shape: vertices (0/2/N) + skin_radius (circle/capsule/edge/polygon/rounded).
enum class FxShapeType { Circle, Capsule, Polygon, Chain };

struct FxShape {
  protected:
    FxShapeType m_shape_type; // Circle, Capsule, Polygon, or Chain
    float m_radius; // bounding radius from centroid (skin-inclusive)
    float m_skin_radius = 0.0f; // Minkowski-sum skin (rounding) radius
    FxVec2fArray m_vertices; // local vertices: 0 (circle), 2 (capsule), or >=3 (polygon)
    FxVec3f m_offset_pose{0.0f, 0.0f, 0.0f}; // initial offset pose in world coordinates
    FxVec3f m_world_pose{0.0f, 0.0f, 0.0f}; // current pose in the world
    FxVec2f m_centroid{0.0f, 0.0f}; //
    FxVec2fArray m_world_vertices;

    // 1) Compute the bounding radius from (0,0)
    static float calc_radius(const FxVec2fArray& verts) {
        float maxSq = 0.0f;
        for (size_t i = 0; i < verts.size(); ++i) {
            const FxVec2f& v = verts[i];
            float d2 = v.x() * v.x() + v.y() * v.y();
            if (d2 > maxSq) maxSq = d2;
        }
        return std::sqrt(maxSq);
    }

    // 3) Convexity: all cross‐products have same sign
    static bool is_convex(const FxVec2fArray& verts) {
        size_t n = verts.size();
        bool gotPos = false, gotNeg = false;
        for (size_t i = 0; i < n; ++i) {
            const FxVec2f& A = verts[i];
            const FxVec2f& B = verts[(i + 1) % n];
            const FxVec2f& C = verts[(i + 2) % n];
            float cross = (B.x() - A.x()) * (C.y() - B.y()) - (B.y() - A.y()) * (C.x() - B.x());
            if (cross > 0) gotPos = true;
            else if (cross < 0) gotNeg = true;
            if (gotPos && gotNeg) return false;
        }
        return true;
    }

  public:
    // default ctor
    FxShape() : m_shape_type(FxShapeType::Circle), m_radius(0.5f), m_skin_radius(0.5f) {}

    // –– Circle ctor: unified as a 0-vertex shape with skin_radius = radius
    FxShape(float radius) {
        if (radius <= 1e-6f) throw std::invalid_argument("FxShape: radius must be > 0");
        m_shape_type = FxShapeType::Circle;
        m_radius = radius;
        m_skin_radius = radius;
    }

    // –– Capsule ctor: segment of given length (along x in local frame) with end-cap radius.
    //    length == 0 collapses to a circle of the same radius. radius == 0 yields a bare segment.
    FxShape(float length, float radius) {
        if (radius < 0.0f) throw std::invalid_argument("FxShape: capsule radius must be >= 0");
        if (length < 0.0f) throw std::invalid_argument("FxShape: capsule length must be >= 0");
        if (length <= 1e-6f && radius <= 1e-6f)
            throw std::invalid_argument("FxShape: degenerate capsule (zero length and radius)");
        const float hl = length * 0.5f;
        m_shape_type = FxShapeType::Capsule;
        m_vertices = {{-hl, 0.0f}, {hl, 0.0f}};
        m_skin_radius = radius;
        m_radius = hl + radius;
        m_world_vertices = m_vertices;
    }

    // –– Edge ctor: zero-thickness segment between two local-frame endpoints.
    //    Stored as a capsule with skin_radius = 0; endpoints are kept as given so the
    //    body origin stays where the scene author placed it.
    FxShape(const FxVec2f& a, const FxVec2f& b) {
        if ((b - a).norm() <= 1e-6f)
            throw std::invalid_argument("FxShape: edge endpoints must be distinct");
        m_shape_type = FxShapeType::Capsule;
        m_vertices = {a, b};
        m_skin_radius = 0.0f;
        m_radius = calc_radius(m_vertices);
        m_world_vertices = m_vertices;
    }

    // Chain: an open polyline of >= 2 segments, for static level geometry a convex polygon
    // approximates badly. Like an edge it has no interior, so no area and no inertia, and like
    // an edge its vertices are kept exactly as authored rather than recentred.
    struct ChainTag {};
    FxShape(const FxVec2fArray& points, ChainTag) {
        if (points.size() < 3)
            throw std::invalid_argument("FxShape: a chain needs at least 3 points");
        for (std::size_t i = 1; i < points.size(); ++i) {
            if ((points[i] - points[i - 1]).norm() <= 1e-6f)
                throw std::invalid_argument("FxShape: chain points must be distinct");
        }
        m_shape_type = FxShapeType::Chain;
        m_vertices = points;
        m_skin_radius = 0.0f;
        m_radius = calc_radius(m_vertices);
        m_world_vertices = m_vertices;
    }

    // –– Polygon from arbitrary vertices, with optional uniform skin (rounding) radius
    FxShape(const FxVec2fArray& vertices, float skin_radius = 0.0f) {
        constexpr float minArea = 1e-6f;
        if (vertices.size() < 3) throw std::invalid_argument("FxShape: less than 3 vertices");
        if (skin_radius < 0.0f) throw std::invalid_argument("FxShape: skin radius must be >= 0");
        float area = polygon_area(vertices);
        if (std::fabs(area) <= minArea) throw std::invalid_argument("FxShape: area ≤ 2e-6");
        if (!is_convex(vertices)) throw std::invalid_argument("FxShape: not convex");
        m_shape_type = FxShapeType::Polygon;
        // centroid will be pushed to {0.0f, 0.0f}
        FxVec2fArray verts = vertices;
        if (area > 0.0f) { // saved in CCW order only
            std::reverse(verts.begin(), verts.end());
        }
        m_vertices = verts - verts.mean();
        m_skin_radius = skin_radius;
        m_radius = calc_radius(m_vertices) + skin_radius;
        m_world_vertices = m_vertices;
    }

    // –– Rectangle centered at origin, width=size.x(), height=size.y(); optional rounded corners
    FxShape(const FxVec2f& size, float skin_radius = 0.0f) {
        if (size.x() <= 0.0f || size.y() <= 0.0f)
            throw std::invalid_argument("FxShape: dimensions must be > 0");
        if (skin_radius < 0.0f) throw std::invalid_argument("FxShape: skin radius must be >= 0");
        float hx = size.x() * 0.5f;
        float hy = size.y() * 0.5f;
        // Check for valid area
        if (hx * hy <= 1e-6f) throw std::runtime_error("FxShape: degenerate rectangle");
        // build CCW rectangle around (0, 0)
        m_vertices = {{-hx, -hy}, {-hx, hy}, {hx, hy}, {hx, -hy}};
        m_shape_type = FxShapeType::Polygon;
        m_skin_radius = skin_radius;
        m_radius = std::sqrt(hx * hx + hy * hy) + skin_radius;
        m_world_vertices = m_vertices;
    }

    // Shoelace signed area of a vertex loop: >0 counter-clockwise, <0 clockwise.
    static float polygon_area(const FxVec2fArray& verts) {
        double sum = 0.0;
        const size_t n = verts.size();
        for (size_t i = 0; i < n; ++i) {
            const FxVec2f& a = verts[i];
            const FxVec2f& b = verts[(i + 1) % n];
            sum += double(a.x()) * b.y() - double(b.x()) * a.y();
        }
        return float(0.5 * sum);
    }

    // getters for shape properties
    FxShapeType shape_type() const { return m_shape_type; }
    float radius() const { return m_radius; }
    float skin_radius() const { return m_skin_radius; }
    // By reference: returning by value copied every vertex into a fresh aligned allocation,
    // invisibly, because every caller writes `const auto& v = shape->vertices()`.
    const FxVec2fArray& vertices() const { return m_world_vertices; }
    FxVec2fArray __vertices() const {
        return m_vertices;
    } // native coordinates of vertices with centroid as (0,0)
    FxVec2f centroid() const { return m_centroid; }

    // methods to check shape type
    bool is_circle() const { return m_shape_type == FxShapeType::Circle; }

    bool is_capsule() const { return m_shape_type == FxShapeType::Capsule; }

    bool is_polygon() const { return m_shape_type == FxShapeType::Polygon; }

    // Builds an open polyline collider from >= 3 points.
    static FxShape make_chain(const FxVec2fArray& points) { return FxShape(points, ChainTag{}); }

    bool is_chain() const { return m_shape_type == FxShapeType::Chain; }

    // Segments in a chain; 0 for every other shape.
    std::size_t segment_count() const { return is_chain() ? m_world_vertices.size() - 1 : 0; }

    // Segment i of a chain as a standalone edge, positioned in the world.
    FxShape segment(std::size_t i) const {
        return FxShape(m_world_vertices[i], m_world_vertices[i + 1]);
    }

    // A zero-skin capsule is a bare segment: zero area, zero inertia, static level geometry.
    bool is_edge() const { return m_shape_type == FxShapeType::Capsule && m_skin_radius <= 1e-6f; }

    // Is the world point inside this shape, skin included? Convex only, which every FxShape is.
    bool contains(const FxVec2f& p) const {
        if (is_chain()) return false;
        const size_t n = m_world_vertices.size();
        if (n == 0) return (p - m_centroid).norm() <= m_skin_radius;
        if (n == 2) {
            const FxVec2f closest = FxClosestOnSegment(m_world_vertices[0], m_world_vertices[1], p);
            return (p - closest).norm() <= m_skin_radius;
        }
        // Same side of every edge. Accepting either sign keeps it winding-agnostic; the skin is
        // exact away from the corners and conservative at them.
        bool all_left = true, all_right = true;
        for (size_t i = 0; i < n; ++i) {
            const FxVec2f& a = m_world_vertices[i];
            const FxVec2f& b = m_world_vertices[(i + 1) % n];
            const FxVec2f edge = b - a;
            const float len = edge.norm();
            if (len < 1e-8f) continue;
            const float side = edge.cross(p - a) / len;
            if (side < -m_skin_radius) all_left = false;
            if (side > m_skin_radius) all_right = false;
        }
        return all_left || all_right;
    }

    // Get area of the shape (handles circle, capsule, and polygon — skin radius included)
    float area() const {
        if (is_chain()) return 0.0f;
        if (is_circle()) {
            return FxPif * m_skin_radius * m_skin_radius;
        }
        if (is_capsule()) {
            // Minkowski sum of segment (length L) with disc (radius r):
            // area = pi r^2 (two end caps form one full disc) + 2 r L (central rectangle)
            const float L = (m_vertices[1] - m_vertices[0]).norm();
            return FxPif * m_skin_radius * m_skin_radius + 2.0f * m_skin_radius * L;
        }
        // Polygon: raw polygon area + (skin contribution if rounded)
        const float core = std::abs(polygon_area(m_world_vertices));
        if (m_skin_radius <= 0.0f) return core;
        // Skin contribution = perimeter * r + pi r^2 (full disc from summing exterior corner
        // angles)
        float perim = 0.0f;
        const auto& V = m_vertices;
        for (std::size_t i = 0, n = V.size(); i < n; ++i) {
            perim += (V[(i + 1) % n] - V[i]).norm();
        }
        return core + perim * m_skin_radius + FxPif * m_skin_radius * m_skin_radius;
    }

    // Calculate moment of inertia for given mass (uniform density)
    float calc_inertia(float mass) const {
        if (is_chain()) return 0.0f;
        if (is_circle()) {
            return 0.5f * mass * m_skin_radius * m_skin_radius;
        }
        if (is_capsule()) {
            // Uniform-density capsule: rectangle (L x 2r) + full disc (radius r) split between
            // caps.
            const float L = (m_vertices[1] - m_vertices[0]).norm();
            const float r = m_skin_radius;
            const float A_rect = 2.0f * r * L;
            const float A_caps = FxPif * r * r;
            const float A_tot = A_rect + A_caps;
            if (A_tot < 1e-6f) return 0.0f;
            const float m_rect = mass * (A_rect / A_tot);
            const float m_caps = mass * (A_caps / A_tot);
            // Rectangle about its centroid (capsule center): I = m * (L^2 + (2r)^2) / 12
            const float I_rect = m_rect * (L * L + 4.0f * r * r) / 12.0f;
            // Two half-discs offset by L/2 from capsule center; parallel-axis theorem.
            const float I_caps = 0.5f * m_caps * r * r + m_caps * (L * 0.5f) * (L * 0.5f);
            return I_rect + I_caps;
        }
        // Polygon (with optional skin):
        const std::size_t n = m_vertices.size();
        float signed_twice_area = 0.0f;
        float accum = 0.0f;
        for (std::size_t i = 0; i < n; ++i) {
            const FxVec2f& a = m_vertices[i];
            const FxVec2f& b = m_vertices[(i + 1) % n];
            const float cross = a.x() * b.y() - b.x() * a.y();
            signed_twice_area += cross;
            const float x2 = a.x() * a.x() + a.x() * b.x() + b.x() * b.x();
            const float y2 = a.y() * a.y() + a.y() * b.y() + b.y() * b.y();
            accum += cross * (x2 + y2);
        }
        float core_area = std::abs(signed_twice_area * 0.5f);
        if (core_area < 1e-6f) return 0.0f;

        if (m_skin_radius <= 0.0f) {
            const float density = mass / core_area;
            return (density / 12.0f) * std::abs(accum);
        }
        // Rounded polygon: keep the bare polygon inertia and add a uniform skin ring approximation.
        // The skin contributes mass roughly at the average vertex radius + skin_radius.
        const float total_area = area();
        const float density = mass / total_area;
        const float I_core = (density / 12.0f) * std::abs(accum);
        // Skin mass approximated as a ring at the bounding radius.
        const float m_skin = mass - density * core_area;
        const float r_eff_sq = m_radius * m_radius - m_skin_radius * m_skin_radius * 0.5f;
        return I_core + m_skin * std::max(0.0f, r_eff_sq);
    }

    // offset pose setter and getter
    void set_offset_pose(const FxVec3f& o_pose) { m_offset_pose = o_pose; }
    FxVec3f offset_pose() const { return m_offset_pose; }

    // Returns current axis aligned bounding box of the shape and sets world pose
    FxArray<float> set_world_pose(const FxVec3f& world_pose) {
        FxArray<float> bb(4);
        set_world_pose(world_pose, bb);
        return bb;
    }

    // Allocation-free form, for the caller that runs per entity per substep: the form above
    // builds three temporary FxArrays. `out_aabb` must already hold four elements and is
    // written as {minX, minY, maxX, maxY}.
    void set_world_pose(const FxVec3f& world_pose, FxArray<float>& out_aabb) {
        m_world_pose = world_pose;
        m_centroid = world_pose.xy() + m_offset_pose.xy();
        if (is_circle()) {
            const float pX = m_centroid.x();
            const float pY = m_centroid.y();
            const float r = m_skin_radius;
            out_aabb[0] = pX - r;
            out_aabb[1] = pY - r;
            out_aabb[2] = pX + r;
            out_aabb[3] = pY + r;
            return;
        }

        // Capsule and polygon: rotate local vertices into the world frame, then inflate the
        // AABB by the skin. Same arithmetic and same order as FxVec2f::rotate_inplace_rad
        // followed by a translate, so results are unchanged to the bit.
        const std::size_t n = m_vertices.size();
        if (m_world_vertices.size() != n) m_world_vertices = FxVec2fArray(n);
        if (n == 0) return;

        const float theta = world_pose.theta() + m_offset_pose.theta();
        const float cos_t = std::cos(theta), sin_t = std::sin(theta);
        const float cx = m_centroid.x(), cy = m_centroid.y();

        float min_x = FxInfinityf, min_y = FxInfinityf;
        float max_x = -FxInfinityf, max_y = -FxInfinityf;
        for (std::size_t i = 0; i < n; ++i) {
            const float xi = m_vertices[i].x(), yi = m_vertices[i].y();
            const float x = xi * cos_t - yi * sin_t + cx;
            const float y = xi * sin_t + yi * cos_t + cy;
            m_world_vertices[i] = FxVec2f(x, y);
            if (x < min_x) min_x = x;
            if (x > max_x) max_x = x;
            if (y < min_y) min_y = y;
            if (y > max_y) max_y = y;
        }

        out_aabb[0] = min_x - m_skin_radius;
        out_aabb[1] = min_y - m_skin_radius;
        out_aabb[2] = max_x + m_skin_radius;
        out_aabb[3] = max_y + m_skin_radius;
    }

    // Getter for the current world pose of the shape
    FxVec3f world_pose() const { return m_world_pose; }

    // Set the position (xy) of the shape in world coordinates (preserving rotation)
    void set_position(const FxVec2f& pos) {
        m_world_pose.set_xy(pos);
        set_world_pose(m_world_pose);
    }

    // Set the rotation (theta) of the shape in world coordinates (preserving position)
    void set_rotation(float theta) {
        m_world_pose.set_theta(theta);
        set_world_pose(m_world_pose);
    }

    // Move the shape by a delta in world coordinates
    void move(const FxVec2f& delta) {
        m_world_pose.set_xy(m_world_pose.xy() + delta);
        set_world_pose(m_world_pose);
    }

    // Rotate the shape by a delta angle (in radians)
    void rotate(float delta_theta) {
        m_world_pose.set_theta(m_world_pose.theta() + delta_theta);
        set_world_pose(m_world_pose);
    }

    // Smallest projection of (vertex - origin) onto axis, with the index that achieved it. No
    // skin applied; SAT subtracts both skin radii explicitly. A running minimum, deliberately:
    // the array-building spelling put ~11% of a box-stack step in malloc and free.
    std::pair<std::size_t, float> min_projection(const FxVec2f& axis, const FxVec2f& origin) const {
        if (is_circle()) {
            // A circle projects as a single point; the caller applies its radius via the skin.
            return {0, (m_centroid - origin).dot(axis)};
        }
        const std::size_t n = m_world_vertices.size();
        if (n == 0) return {0, FxInfinityf};

        // Scalars rather than (vertex - origin).dot(axis): the same arithmetic, but Eigen's
        // expression templates showed up across the profile for a loop this hot. Same operands
        // in the same order, so with FP contraction pinned off it is bit-identical.
        const float ox = origin.x(), oy = origin.y();
        const float ax = axis.x(), ay = axis.y();
        std::size_t best_index = 0;
        float best = FxInfinityf;
        for (std::size_t i = 0; i < n; ++i) {
            const float px = m_world_vertices[i].x(), py = m_world_vertices[i].y();
            const float projection = (px - ox) * ax + (py - oy) * ay;
            if (projection < best) {
                best = projection;
                best_index = i;
            }
        }
        return {best_index, best};
    }

    // get the closest vertex of the shape from a point (returns surface point, skin-inclusive)
    FxVec2f get_closest_vertex(const FxVec2f& point) const {
        if (is_circle()) {
            FxVec2f v = point - m_centroid; // vector from center to query point
            FxVec2f dir;
            if (v.dot(v) < 1e-6f) dir = FxVec2f(1.0f, 0.0f); // arbitrary unit vector
            else dir = v.normalized(); // safe to normalize
            return m_centroid + dir * m_skin_radius;
        }
        if (is_capsule()) {
            // Closest point on the capsule's central segment, then pushed out by skin radius.
            const FxVec2f& a = m_world_vertices[0];
            const FxVec2f& b = m_world_vertices[1];
            FxVec2f ab = b - a;
            float len2 = ab.dot(ab);
            FxVec2f q =
                (len2 < 1e-6f) ? a : a + std::clamp((point - a).dot(ab) / len2, 0.0f, 1.0f) * ab;
            FxVec2f v = point - q;
            float vlen = v.norm();
            FxVec2f dir = (vlen > 1e-6f) ? v / vlen : FxVec2f(1.0f, 0.0f);
            return q + dir * m_skin_radius;
        }
        // Polygon
        auto shifted = (m_world_vertices - point);
        auto dist = (shifted).dot(shifted);
        auto [min_ind, min_value] = dist.argmin();
        return m_world_vertices[min_ind];
    }
};

class FxEntity;

// What a ray struck.
struct FxRayHit {
    std::shared_ptr<FxEntity> entity = nullptr;
    FxVec2f point{0.0f, 0.0f}; // world-space point of impact
    FxVec2f normal{0.0f, 0.0f}; // outward surface normal there, facing back along the ray
    float distance = 0.0f; // travel along the ray from its origin

    bool hit() const { return entity != nullptr; }
};
