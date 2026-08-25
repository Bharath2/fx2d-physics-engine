# Math Utilities — `Fx2D/Math.h`

All math types and utilities are defined in `include/Fx2D/Math.h`, built on top of [Eigen](https://eigen.tuxfamily.org/). Include via:

```cpp
#include "Fx2D/Math.h"
```

---

## Constants

| Constant | Type | Value |
|---|---|---|
| `FxPif` | `float` | π |
| `FxPid` | `double` | π |
| `FxInfinityf` | `float` | +∞ |
| `FxInfinityd` | `double` | +∞ |

```cpp
float FxAngleWrap(float angle);  // wraps angle to [-π, π]
```

---

## Vector Types

All vector types extend their corresponding Eigen base, so all Eigen operations (`.norm()`, `.dot()`, `.normalized()`, etc.) are available.

### `FxVec2f` / `FxVec2d`

2D float / double vectors.

```cpp
FxVec2f v(1.0f, 2.0f);
FxVec2f u(3.0f);          // fills both components: (3, 3)

v.x();  v.y();             // getters
v.set_x(1.0f);             // setters

v.cross(u);                // scalar cross product: v.x*u.y - v.y*u.x
v.perp();                  // CCW perpendicular: (-y, x)
v.perpCW();                // CW perpendicular:  ( y,-x)

v.rotate(45.0f);           // returns rotated copy (degrees)
v.rotate_rad(FxPif/4);     // returns rotated copy (radians)
v.rotate_inplace(45.0f);   // mutates v (degrees)
v.rotate_inplace_rad(...); // mutates v (radians)
```

Scalar arithmetic operators (`+`, `-`, `*`, `/`) work with any numeric type:
```cpp
FxVec2f result = v * 2;
FxVec2f result = 3.0f + v;
```

### `FxVec3f` / `FxVec3d`

3D float / double vectors. When used as a 2D pose, `z()` / `theta()` hold the orientation angle.

```cpp
FxVec3f p(1.0f, 2.0f, 0.5f);
p.x(); p.y(); p.z();
p.theta();               // alias for z(), used for orientation

p.xy();                  // returns FxVec2f view of the first two components
p.get_xy();              // returns FxVec2f copy
p.set_xy(FxVec2f(1,2));  // sets x and y
```

### `FxVec4f`

4D float vector. Used for RGBA colors in some APIs.

```cpp
FxVec4f c(255, 128, 0, 255);
c.x(); c.y(); c.z(); c.a();
```

### `FxVec2ui`

2D unsigned integer vector. Used for scene dimensions.

```cpp
FxVec2ui size(1920, 1080);
size.x(); size.y();
```

### `FxVec4ui8`

4D `uint8_t` vector. Used for RGBA colors.

```cpp
FxVec4ui8 color(255, 128, 0, 255);  // R, G, B, A
color.x(); color.y(); color.z(); color.a();
```

---

## Matrix Types

### `FxMat2f` — 2×2 float matrix

Used for 2D rotation matrices.

```cpp
FxMat2f R;
R.a(); R.b();   // row 0: (0,0), (0,1)
R.c(); R.d();   // row 1: (1,0), (1,1)

FxMat2f Rinv = R.inv_rotation();  // transpose (inverse of rotation matrix)
```

### `FxMat3f` — 3×3 float matrix

Used for 2D homogeneous transformation matrices of the form:

$$M = \begin{bmatrix} R & t \\ 0 & 1 \end{bmatrix}$$

```cpp
FxMat3f M;
M.Rot();                  // returns the 2×2 rotation block as FxMat2f
M.t();                    // returns the translation as FxVec2f
M.set_Rot(R);
M.set_t(trans);
FxMat3f Minv = M.inv_transform();  // computes M⁻¹ analytically
```

---

## FxArray

A NumPy-style 1D array template with 32-byte aligned allocation for SIMD performance. Supports `float`, `double`, integer types, and `FxVec2f` / `FxVec3f` / `FxVec4f`.

```cpp
FxArray<float> a(100);                       // size-100 array (NOT zero-initialized — raw allocation)
FxArray<float> b = {1.0f, 2.0f, 3.0f};      // from initializer list
FxArray<float> c(std::vector<float>{...});   // from std::vector

a[i];           // unchecked access (fast, for hot loops)
a.at(i);        // bounds-checked access (throws std::out_of_range)
a.size();
a.empty();
```

Supports STL range-for:
```cpp
for (float val : a) { ... }
```

### Arithmetic Operators

All standard operators work element-wise or with a scalar:
```cpp
a + b;   a - b;   a * b;   a / b;   // element-wise (same size required)
a + 2.0f;  3 * a;  a / 2;           // scalar broadcast
a += b;  a -= 2.0f;                 // in-place variants
-a;                                 // element-wise negate
```

### Aggregates (numeric types only)

```cpp
a.min();                          // minimum value
a.max();                          // maximum value
a.argmin();                       // {index, min_value}
a.argmax();                       // {index, max_value}
a.mean();                         // T, uses T's + and /
a.meanf();                        // float mean (double accumulator)
a.stddev();                       // float population standard deviation
```

### Type Conversion

```cpp
FxArray<double> d = a.as<double>();
```

---

## FxVec2fArray

`FxVec2fArray` is a type alias for `FxArray<FxVec2f>` — an aligned array of 2D float vectors.

```cpp
using FxVec2fArray = FxArray<FxVec2f>;
```

Extra operations available on `FxVec2fArray`:

```cpp
FxVec2fArray pts = { FxVec2f(1,0), FxVec2f(0,1) };

pts.rotate_inplace(45.0f);       // rotate all vectors in-place (degrees)
pts.rotate_inplace_rad(FxPif/4); // rotate all vectors in-place (radians)
pts.rotate_rad(theta);           // returns rotated copy
pts.perp_inplace();              // CCW perpendicular of each vector, in-place
pts.perp();                      // returns perpendicular copy

// dot product
FxArray<float> d = pts.dot(FxVec2f(1,0));  // dot each vector with a fixed vector
FxArray<float> d = pts.dot(other_pts);     // element-wise dot with another FxVec2fArray
```

---

## FxShape

`FxShape` is the single collision/visual geometry type. Every shape — circle, capsule,
edge, polygon, rounded polygon — is stored the same way: **a vertex list plus a skin
radius**, where the skin is a Minkowski sum of the vertex core with a disc of that radius.

```cpp
enum class FxShapeType { Circle, Capsule, Polygon };
```

| Shape | Type | Vertices | Skin radius |
|---|---|---|---|
| Circle | `Circle` | 0 | the radius |
| Capsule | `Capsule` | 2 (segment endpoints) | end-cap radius |
| Edge | `Capsule` | 2 (segment endpoints) | 0 |
| Polygon | `Polygon` | ≥ 3 | 0 (sharp) or > 0 (rounded corners) |

An **edge is not a separate type** — it is a capsule whose skin radius is zero, which is
what `is_edge()` tests. Because circle, capsule, and rounded polygon all share the skin
mechanism, contact normals, AABBs, area, and inertia follow one consistent code path.

### Constructors

```cpp
FxShape();                                          // circle, radius 0.5
FxShape(float radius);                              // circle
FxShape(float length, float radius);                // capsule along local +x
FxShape(const FxVec2f& a, const FxVec2f& b);        // edge (zero-skin capsule)
FxShape(const FxVec2fArray& vertices, float skin_radius = 0.0f);  // polygon
FxShape(const FxVec2f& size, float skin_radius = 0.0f);           // rectangle -> polygon
```

The overloads are distinguished only by argument count and type, so watch the two-float
case: `FxShape(0.5f)` is a circle, `FxShape(2.0f, 0.3f)` is a capsule. Likewise
`FxShape(a, b)` with two `FxVec2f` is an edge, while `FxShape(size, 0.1f)` is a rectangle.

All constructors validate their input and throw `std::invalid_argument` on degenerate
geometry: a non-positive circle radius, a capsule with both length and radius at zero, a
zero-length edge, or a polygon that has fewer than 3 vertices, negligible area, or is
non-convex.

Polygons and rectangles are **recentred on their centroid** at construction, so the stored
vertices are relative to the centre of area. Edges are the exception: their endpoints are
kept exactly as authored, which is what makes them usable as level geometry placed by
absolute coordinates.

### Queries

```cpp
FxShapeType shape_type() const;
bool is_circle() const;
bool is_capsule() const;
bool is_polygon() const;
bool is_edge() const;      // capsule with skin_radius <= 1e-6

float radius() const;       // bounding radius from the centroid, skin included
float skin_radius() const;
FxVec2fArray vertices() const;    // world-space vertices
FxVec2fArray __vertices() const;  // local vertices, centroid at the origin
FxVec2f centroid() const;         // world-space centroid

float area() const;                     // skin included
float calc_inertia(float mass) const;   // about the centroid
```

`area()` and `calc_inertia()` account for the skin: a capsule is a rectangle plus a full
disc, and a rounded polygon is its core plus a perimeter band plus a disc. A zero-skin
capsule (an edge) therefore has **zero area and zero inertia**, which is why edges should
be paired with `mass: 0` and used as static geometry.

For rounded polygons `calc_inertia()` is an approximation: the core is taken at reduced
density and the skin mass is treated as a ring. Sharp polygons use the exact shoelace
second moment.

### Placement

```cpp
void set_offset_pose(const FxVec3f& pose);   // shape pose relative to the entity
FxVec3f offset_pose() const;
FxArray<float> set_world_pose(const FxVec3f& world_pose);  // returns the AABB
```

`set_world_pose()` transforms the local vertices into world space and returns the
axis-aligned bounding box, inflated by the skin radius on all sides. `FxEntity::step()`
calls it every substep, so `vertices()` and `centroid()` always reflect the current pose.

> **Inertia is computed from the *visual* shape.** `FxEntity::set_inertia()` reads
> `visual_geometry()`, not the collision shape, and returns zero if no visual shape is
> attached or if mass is zero. Attach the visual geometry **before** calling it.

See [Scene YAML](./scene-yaml) for the YAML spelling of every shape, and
[the collision pipeline](../concepts/collisions) for how each pair is tested.
