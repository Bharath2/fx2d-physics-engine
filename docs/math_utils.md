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
