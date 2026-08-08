#pragma once 
#include <Eigen/Core>
#include <cstdint>
#include <vector>
#include <initializer_list>
#include <algorithm> 
#include <iostream>
#include <string>
#include <memory>
#include <stdexcept>
#include <type_traits>
#include <concepts> 
#include <numbers>

// Constant representing positive infinity
static constexpr float FxInfinityf = std::numeric_limits<float>::infinity();
static constexpr double FxInfinityd = std::numeric_limits<double>::infinity();

// Constant representing pi 
static constexpr float FxPif = std::numbers::pi_v<float>;
static constexpr double FxPid = std::numbers::pi_v<double>;

// Helper function for angle wrapping
static inline float FxAngleWrap(float angle) {
    angle = std::fmod(angle + FxPif, 2.0f * FxPif);
    if (angle < 0.0f) angle += 2.0f * FxPif;
    return angle - FxPif;
}


// concept to capture float, double, long double, int etc..
template<typename T>
concept Numeric = std::integral<T> || std::floating_point<T>; 

// Custom 2D float vector with .x() getter and .set_x() setter.
class FxVec2f : public Eigen::Vector2f {
public:
    // Inherit constructors
    using Eigen::Vector2f::Vector2f;

    // Constructor from single float (fills both components)
    explicit FxVec2f(float a) : Eigen::Vector2f(a, a) {}

    // Getter for x and y.
    float& x() { return (*this)(0); }
    float& y() { return (*this)(1); }

    // Const getters.
    float x() const { return (*this)(0); }
    float y() const { return (*this)(1); }

    // Setter for x and y.
    void set_x(float val) { (*this)(0) = val; }
    void set_y(float val) { (*this)(1) = val; }

    //Radian‐based rotate (suffix "_rad")
    FxVec2f& rotate_inplace_rad(float theta) noexcept {
        const float c = std::cos(theta), s = std::sin(theta);
        float xi = x(), yi = y();
        set_x(xi * c - yi * s);
        set_y(xi * s + yi * c);
        return *this;
    }

    // Degree‐based rotate (default "rotate_inplace" uses degrees)
    FxVec2f& rotate_inplace(float degrees) noexcept {
        constexpr float FX_DEG2RAD = FxPif / 180.0f;
        return rotate_inplace_rad(degrees * FX_DEG2RAD);
    }

    // — non-mutating rotation: returns a rotated copy
    FxVec2f rotate(float theta) const  noexcept {
        return FxVec2f(*this).rotate_inplace(theta);
    }
    FxVec2f rotate_rad(float theta) const  noexcept {
        return FxVec2f(*this).rotate_inplace_rad(theta);
    }

    // Cross product with another 2D vector (returns scalar)
    float cross(const FxVec2f& other) const {
        return x() * other.y() - y() * other.x();
    }

    // Perpendicular vectors
    FxVec2f perp() const {
        return FxVec2f(-y(), x()); // CCW perpendicular
    }
    
    FxVec2f perpCW() const {
        return FxVec2f(y(), -x()); // CW perpendicular
    }
};

// Custom 2D double vector with .x() getter and .set_x() setter.
class FxVec2d : public Eigen::Vector2d {
public:
    // Inherit constructors
    using Eigen::Vector2d::Vector2d;

    // Constructor from single double (fills both components)
    explicit FxVec2d(double a) : Eigen::Vector2d(a, a) {}

    // Getter for x and y.
    double& x() { return (*this)(0); }
    double& y() { return (*this)(1); }

    // Const getters.
    double x() const { return (*this)(0); }
    double y() const { return (*this)(1); }

    // Setter for x and y.
    void set_x(double val) { (*this)(0) = val; }
    void set_y(double val) { (*this)(1) = val; }

    //Radian‐based rotate (suffix "_rad")
    FxVec2d& rotate_inplace_rad(double theta) noexcept {
        const double c = std::cos(theta), s = std::sin(theta);
        double xi = x(), yi = y();
        set_x(xi * c - yi * s);
        set_y(xi * s + yi * c);
        return *this;
    }

    // Degree‐based rotate (default "rotate_inplace" uses degrees)
    FxVec2d& rotate_inplace(double degrees) noexcept {
        constexpr double FX_DEG2RAD = FxPid / 180.0;
        return rotate_inplace_rad(degrees * FX_DEG2RAD);
    }

    // — non-mutating rotation: returns a rotated copy
    FxVec2d rotate(double theta) const  noexcept {
        return FxVec2d(*this).rotate_inplace(theta);
    }
    FxVec2d rotate_rad(double theta) const  noexcept {
        return FxVec2d(*this).rotate_inplace_rad(theta);
    }

    // Cross product with another 2D vector (returns scalar)
    double cross(const FxVec2d& other) const {
        return x() * other.y() - y() * other.x();
    }

    // Perpendicular vectors
    FxVec2d perp() const {
        return FxVec2d(-y(), x()); // CCW perpendicular
    }
    
    FxVec2d perpCW() const {
        return FxVec2d(y(), -x()); // CW perpendicular
    }
};

using FxVec2fMap = Eigen::Map<Eigen::Vector2f>;

// Custom 3D float vector with .x(), .y(), .z() getters and corresponding setters.
class FxVec3f : public Eigen::Vector3f {
public:
    using Eigen::Vector3f::Vector3f;

    // Constructor from single float (fills all components)
    explicit FxVec3f(float a) : Eigen::Vector3f(a, a, a) {}

    // Getters.
    float& x() { return (*this)(0); }
    float& y() { return (*this)(1); }
    float& z() { return (*this)(2); }
    // to use the third value as orientation 
    float& theta() { return (*this)(2); }

     // Const getters.
    float x() const { return (*this)(0); }
    float y() const { return (*this)(1); }
    float z() const { return (*this)(2); }
    float theta() const { return (*this)(2); }

    // Setters.
    void set_x(float val) { (*this)(0) = val; }
    void set_y(float val) { (*this)(1) = val; }
    void set_z(float val) { (*this)(2) = val; }
    // when used as orientation
    void set_theta(float val) { (*this)(2) = val; }

    FxVec2fMap xy() { return FxVec2fMap(this->data()); }
    FxVec2f get_xy() const { return FxVec2f(this->data()); }
    FxVec2f xy() const { return this->head<2>(); }
    void set_xy(const FxVec2f& v2) { this->head<2>() = v2; }
};

using FxVec2dMap = Eigen::Map<Eigen::Vector2d>;

// Custom 3D double vector with .x(), .y(), .z() getters and corresponding setters.
class FxVec3d : public Eigen::Vector3d {
public:
    using Eigen::Vector3d::Vector3d;

    // Constructor from single double (fills all components)
    explicit FxVec3d(double a) : Eigen::Vector3d(a, a, a) {}

    // Getters.
    double& x() { return (*this)(0); }
    double& y() { return (*this)(1); }
    double& z() { return (*this)(2); }
    // to use the third value as orientation 
    double& theta() { return (*this)(2); }

     // Const getters.
    double x() const { return (*this)(0); }
    double y() const { return (*this)(1); }
    double z() const { return (*this)(2); }
    double theta() const { return (*this)(2); }

    // Setters.
    void set_x(double val) { (*this)(0) = val; }
    void set_y(double val) { (*this)(1) = val; }
    void set_z(double val) { (*this)(2) = val; }
    // when used as orientation
    void set_theta(double val) { (*this)(2) = val; }

    FxVec2dMap xy() { return FxVec2dMap(this->data()); }
    FxVec2d get_xy() const { return FxVec2d(this->data()); }
    FxVec2d xy() const { return this->head<2>(); }
    void set_xy(const FxVec2d& v2) { this->head<2>() = v2; }
};


// Custom 4D float vector with .x(), .y(), .z(), .a() getters and corresponding setters.
class FxVec4f : public Eigen::Vector4f {
public:
    using Eigen::Vector4f::Vector4f;

    // Constructor from single float (fills all components)
    explicit FxVec4f(float a) : Eigen::Vector4f(a, a, a, a) {}

    // Getters.
    float& x() { return (*this)(0); }
    float& y() { return (*this)(1); }
    float& z() { return (*this)(2); }
    float& a() { return (*this)(3); }

    // Const getters.
    float x() const { return (*this)(0); }
    float y() const { return (*this)(1); }
    float z() const { return (*this)(2); }
    float a() const { return (*this)(3); }

    // Setters.
    void set_x(float val) { (*this)(0) = val; }
    void set_y(float val) { (*this)(1) = val; }
    void set_z(float val) { (*this)(2) = val; }
    void set_a(float val) { (*this)(3) = val; }
};

// FxVec2f scalar operations with generic Numeric s
template<Numeric S>
inline FxVec2f operator*(FxVec2f const& v, S s) { return FxVec2f(v.array() * static_cast<float>(s)); }
template<Numeric S>
inline FxVec2f operator*(S s, FxVec2f const& v) { return v * s; }
template<Numeric S>
inline FxVec2f operator/(FxVec2f const& v, S s) { return FxVec2f(v.array() / static_cast<float>(s)); }
template<Numeric S>
inline FxVec2f operator+(FxVec2f const& v, S s) { return FxVec2f(v.array() + static_cast<float>(s)); }
template<Numeric S>
inline FxVec2f operator+(S s, FxVec2f const& v) { return v + s; }
template<Numeric S>
inline FxVec2f operator-(FxVec2f const& v, S s) { return FxVec2f(v.array() - static_cast<float>(s)); }
template<Numeric S>
inline FxVec2f operator-(S s, FxVec2f const& v) { return FxVec2f((FxVec2f::Scalar(static_cast<float>(s)) * FxVec2f::Ones()).array() - v.array()); }

// FxVec3f scalar operations with generic Numeric s
template<Numeric S>
inline FxVec3f operator*(FxVec3f const& v, S s) { return FxVec3f(v.array() * static_cast<float>(s)); }
template<Numeric S>
inline FxVec3f operator*(S s, FxVec3f const& v) { return v * s; }
template<Numeric S>
inline FxVec3f operator/(FxVec3f const& v, S s) { return FxVec3f(v.array() / static_cast<float>(s)); }
template<Numeric S>
inline FxVec3f operator+(FxVec3f const& v, S s) { return FxVec3f(v.array() + static_cast<float>(s)); }
template<Numeric S>
inline FxVec3f operator+(S s, FxVec3f const& v) { return v + s; }
template<Numeric S>
inline FxVec3f operator-(FxVec3f const& v, S s) { return FxVec3f(v.array() - static_cast<float>(s)); }
template<Numeric S>
inline FxVec3f operator-(S s, FxVec3f const& v) { return FxVec3f((FxVec3f::Scalar(static_cast<float>(s)) * FxVec3f::Ones()).array() - v.array()); }

// FxVec2d scalar operations with generic Numeric s
template<Numeric S>
inline FxVec2d operator*(FxVec2d const& v, S s) { return FxVec2d(v.array() * static_cast<double>(s)); }
template<Numeric S>
inline FxVec2d operator*(S s, FxVec2d const& v) { return v * s; }
template<Numeric S>
inline FxVec2d operator/(FxVec2d const& v, S s) { return FxVec2d(v.array() / static_cast<double>(s)); }
template<Numeric S>
inline FxVec2d operator+(FxVec2d const& v, S s) { return FxVec2d(v.array() + static_cast<double>(s)); }
template<Numeric S>
inline FxVec2d operator+(S s, FxVec2d const& v) { return v + s; }
template<Numeric S>
inline FxVec2d operator-(FxVec2d const& v, S s) { return FxVec2d(v.array() - static_cast<double>(s)); }
template<Numeric S>
inline FxVec2d operator-(S s, FxVec2d const& v) { return FxVec2d((FxVec2d::Scalar(static_cast<double>(s)) * FxVec2d::Ones()).array() - v.array()); }

// FxVec3d scalar operations with generic Numeric s
template<Numeric S>
inline FxVec3d operator*(FxVec3d const& v, S s) { return FxVec3d(v.array() * static_cast<double>(s)); }
template<Numeric S>
inline FxVec3d operator*(S s, FxVec3d const& v) { return v * s; }
template<Numeric S>
inline FxVec3d operator/(FxVec3d const& v, S s) { return FxVec3d(v.array() / static_cast<double>(s)); }
template<Numeric S>
inline FxVec3d operator+(FxVec3d const& v, S s) { return FxVec3d(v.array() + static_cast<double>(s)); }
template<Numeric S>
inline FxVec3d operator+(S s, FxVec3d const& v) { return v + s; }
template<Numeric S>
inline FxVec3d operator-(FxVec3d const& v, S s) { return FxVec3d(v.array() - static_cast<double>(s)); }
template<Numeric S>
inline FxVec3d operator-(S s, FxVec3d const& v) { return FxVec3d((FxVec3d::Scalar(static_cast<double>(s)) * FxVec3d::Ones()).array() - v.array()); }

// FxVec4f scalar operations with generic Numeric s
template<Numeric S>
inline FxVec4f operator*(FxVec4f const& v, S s) { return FxVec4f(v.array() * static_cast<float>(s)); }
template<Numeric S>
inline FxVec4f operator*(S s, FxVec4f const& v) { return v * s; }
template<Numeric S>
inline FxVec4f operator/(FxVec4f const& v, S s) { return FxVec4f(v.array() / static_cast<float>(s)); }
template<Numeric S>
inline FxVec4f operator+(FxVec4f const& v, S s) { return FxVec4f(v.array() + static_cast<float>(s)); }
template<Numeric S>
inline FxVec4f operator+(S s, FxVec4f const& v) { return v + s; }
template<Numeric S>
inline FxVec4f operator-(FxVec4f const& v, S s) { return FxVec4f(v.array() - static_cast<float>(s)); }
template<Numeric S>
inline FxVec4f operator-(S s, FxVec4f const& v) { return FxVec4f((FxVec4f::Scalar(static_cast<float>(s)) * FxVec4f::Ones()).array() - v.array()); }

// FxVec2f in-place scalar operations
template<Numeric S>
inline FxVec2f& operator+=(FxVec2f& v, S s) {
    v.array() += static_cast<float>(s);
    return v;
}
template<Numeric S>
inline FxVec2f& operator-=(FxVec2f& v, S s) {
    v.array() -= static_cast<float>(s);
    return v;
}
template<Numeric S>
inline FxVec2f& operator*=(FxVec2f& v, S s) {
    v.array() *= static_cast<float>(s);
    return v;
}
template<Numeric S>
inline FxVec2f& operator/=(FxVec2f& v, S s) {
    v.array() /= static_cast<float>(s);
    return v;
}

// FxVec3f in-place scalar operations
template<Numeric S>
inline FxVec3f& operator+=(FxVec3f& v, S s) {
    v.array() += static_cast<float>(s);
    return v;
}
template<Numeric S>
inline FxVec3f& operator-=(FxVec3f& v, S s) {
    v.array() -= static_cast<float>(s);
    return v;
}
template<Numeric S>
inline FxVec3f& operator*=(FxVec3f& v, S s) {
    v.array() *= static_cast<float>(s);
    return v;
}
template<Numeric S>
inline FxVec3f& operator/=(FxVec3f& v, S s) {
    v.array() /= static_cast<float>(s);
    return v;
}

// FxVec4f in-place scalar operations
template<Numeric S>
inline FxVec4f& operator+=(FxVec4f& v, S s) {
    v.array() += static_cast<float>(s);
    return v;
}
template<Numeric S>
inline FxVec4f& operator-=(FxVec4f& v, S s) {
    v.array() -= static_cast<float>(s);
    return v;
}
template<Numeric S>
inline FxVec4f& operator*=(FxVec4f& v, S s) {
    v.array() *= static_cast<float>(s);
    return v;
}
template<Numeric S>
inline FxVec4f& operator/=(FxVec4f& v, S s) {
    v.array() /= static_cast<float>(s);
    return v;
}

// free non-member operators
inline FxVec2f& operator*=(FxVec2f& lhs, const FxVec2f& rhs) {
    lhs.array() *= rhs.array();
    return lhs;
}
inline FxVec2f& operator/=(FxVec2f& lhs, const FxVec2f& rhs) {
    lhs.array() /= rhs.array();
    return lhs;
}
inline FxVec2f& operator+=(FxVec2f& lhs, const FxVec2f& rhs) {
    lhs.array() += rhs.array();
    return lhs;
}
inline FxVec2f& operator-=(FxVec2f& lhs, const FxVec2f& rhs) {
    lhs.array() -= rhs.array();
    return lhs;
}
// FxVec2f scalar/vector division: s / v
template<Numeric S>
inline FxVec2f operator/(S s, FxVec2f const& v) {
    return FxVec2f((FxVec2f::Scalar(static_cast<float>(s)) * FxVec2f::Ones()).array() / v.array());
}

// FxVec3f scalar/vector division: s / v
template<Numeric S>
inline FxVec3f operator/(S s, FxVec3f const& v) {
    return FxVec3f((FxVec3f::Scalar(static_cast<float>(s)) * FxVec3f::Ones()).array() / v.array());
}

// FxVec4f scalar/vector division: s / v
template<Numeric S>
inline FxVec4f operator/(S s, FxVec4f const& v) {
    return FxVec4f((FxVec4f::Scalar(static_cast<float>(s)) * FxVec4f::Ones()).array() / v.array());
}

// Custom 2D unsigned int vector with .x() getter and .set_x() setter.
class FxVec2ui : public Eigen::Matrix<unsigned int, 2, 1> {
public:
    using Base = Eigen::Matrix<unsigned int, 2, 1>;
    using Base::Base;

    unsigned int& x() { return (*this)(0); }
    unsigned int& y() { return (*this)(1); }

    // Const getters.
    unsigned int x() const { return (*this)(0); }
    unsigned int y() const { return (*this)(1); }

    void set_x(unsigned int val) { (*this)(0) = val; }
    void set_y(unsigned int val) { (*this)(1) = val; }
};


// And a 4D 8-bit unsigned integer vector.
class FxVec4ui8 : public Eigen::Matrix<uint8_t, 4, 1> {
public:
    using Base = Eigen::Matrix<uint8_t, 4, 1>;
    using Base::Base;

    uint8_t& x() { return (*this)(0); }
    uint8_t& y() { return (*this)(1); }
    uint8_t& z() { return (*this)(2); }
    uint8_t& a() { return (*this)(3); }

     // Const getters.
    uint8_t x() const { return (*this)(0); }
    uint8_t y() const { return (*this)(1); }
    uint8_t z() const { return (*this)(2); }
    uint8_t a() const { return (*this)(3); }

    void set_x(uint8_t val) { (*this)(0) = val; }
    void set_y(uint8_t val) { (*this)(1) = val; }
    void set_z(uint8_t val) { (*this)(2) = val; }
    void set_a(uint8_t val) { (*this)(3) = val; }
};


// Axis-aligned bounding box in 2D world coordinates
struct FxAABB {
    float minX = 0.0f, minY = 0.0f, maxX = 0.0f, maxY = 0.0f;
    FxAABB() = default;
    FxAABB(float mnX, float mnY, float mxX, float mxY)
        : minX(mnX), minY(mnY), maxX(mxX), maxY(mxY) {}
    static FxAABB combine(const FxAABB& a, const FxAABB& b) {
        return { std::min(a.minX,b.minX), std::min(a.minY,b.minY),
                 std::max(a.maxX,b.maxX), std::max(a.maxY,b.maxY) };
    }
    FxAABB fatten(float margin) const { return {minX-margin,minY-margin,maxX+margin,maxY+margin}; }
    float perimeter() const { return (maxX-minX)+(maxY-minY); }
    bool overlaps(const FxAABB& o) const {
        return maxX>=o.minX && o.maxX>=minX && maxY>=o.minY && o.maxY>=minY;
    }
    bool contains(const FxAABB& inner) const {
        return minX<=inner.minX && minY<=inner.minY && maxX>=inner.maxX && maxY>=inner.maxY;
    }
    bool is_valid() const { return maxX > minX && maxY > minY; }
};


// Custom 2x2 float matrix with .a(), .b(), .c(), .d() getters and corresponding setters.
class FxMat2f : public Eigen::Matrix2f {
public:
    using Eigen::Matrix2f::Matrix2f;

    float& a() { return (*this)(0, 0); }
    float& b() { return (*this)(0, 1); }
    float& c() { return (*this)(1, 0); }
    float& d() { return (*this)(1, 1); }

     // Const getters.
    float a() const { return (*this)(0, 0); }
    float b() const { return (*this)(0, 1); }
    float c() const { return (*this)(1, 0); }
    float d() const { return (*this)(1, 1); }

    void set_a(float val) { (*this)(0, 0) = val; }
    void set_b(float val) { (*this)(0, 1) = val; }
    void set_c(float val) { (*this)(1, 0) = val; }
    void set_d(float val) { (*this)(1, 1) = val; }

    //The inverse (transpose) of the rotation matrix.
    FxMat2f inv_rotation() const {
        return this->transpose();
    }

};


class FxMat3f : public Eigen::Matrix3f {
public:
    using Eigen::Matrix3f::Matrix3f;

    // Accessors for each element of the 3x3 matrix
    // Row 0
    float& a() { return (*this)(0, 0); }
    float& b() { return (*this)(0, 1); }
    float& c() { return (*this)(0, 2); }
    // Row 1
    float& d() { return (*this)(1, 0); }
    float& e() { return (*this)(1, 1); }
    float& f() { return (*this)(1, 2); }
    // Row 2
    float& g() { return (*this)(2, 0); }
    float& h() { return (*this)(2, 1); }
    float& i() { return (*this)(2, 2); }

    // Const getters.
    float a() const { return (*this)(0, 0); }
    float b() const { return (*this)(0, 1); }
    float c() const { return (*this)(0, 2); }
    float d() const { return (*this)(1, 0); }
    float e() const { return (*this)(1, 1); }
    float f() const { return (*this)(1, 2); }
    float g() const { return (*this)(2, 0); }
    float h() const { return (*this)(2, 1); }
    float i() const { return (*this)(2, 2); }
    
    // Setters for each element of the 3x3 matrix
    // Row 0
    void set_a(float val) { (*this)(0, 0) = val; }
    void set_b(float val) { (*this)(0, 1) = val; }
    void set_c(float val) { (*this)(0, 2) = val; }
    // Row 1
    void set_d(float val) { (*this)(1, 0) = val; }
    void set_e(float val) { (*this)(1, 1) = val; }
    void set_f(float val) { (*this)(1, 2) = val; }
    // Row 2
    void set_g(float val) { (*this)(2, 0) = val; }
    void set_h(float val) { (*this)(2, 1) = val; }
    void set_i(float val) { (*this)(2, 2) = val; }

    // For a homogeneous transformation matrix M = [ R  t ]
    //                                             [ 0  1 ]
    // its inverse is: M⁻¹ = [ Rᵀ  -Rᵀt ]
    //                       [  0    1  ]
    // Extract the 2x2 rotation matrix from the upper-left block as an FxMat2f
    FxMat2f Rot() const { return this->block<2, 2>(0, 0).eval(); }
    // Extract the translation vector (first two elements of the third column) as an FxVec2f
    FxVec2f t() const { return FxVec2f((*this)(0, 2), (*this)(1, 2)); }

    // set the rotation part from an FxMat2f
    void set_Rot(const FxMat2f &R) {
         (*this)(0, 0) = R(0, 0);
         (*this)(0, 1) = R(0, 1);
         (*this)(1, 0) = R(1, 0);
         (*this)(1, 1) = R(1, 1);
    }

    //set the translation part from an FxVec2f
    void set_t(const FxVec2f &trans) {
         (*this)(0, 2) = trans.x();  // Assuming FxVec2f has x() method.
         (*this)(1, 2) = trans.y();  // And a y() method.
    }

    // Compute and return the inverse transformation.
    FxMat3f inv_transform() const {
        FxMat3f inv;  // This will store the inverse.

        // Compute the rotation transpose (which is the inverse of a rotation).
        inv(0, 0) = (*this)(0, 0);
        inv(0, 1) = (*this)(1, 0);
        inv(1, 0) = (*this)(0, 1);
        inv(1, 1) = (*this)(1, 1);

        // Compute the new translation vector as -Rᵀt.
        float t0 = (*this)(0, 2); // original translation x value.
        float t1 = (*this)(1, 2); // original translation y value.
        inv(0, 2) = - (inv(0, 0) * t0 + inv(0, 1) * t1);
        inv(1, 2) = - (inv(1, 0) * t0 + inv(1, 1) * t1);

        // Set the homogeneous row.
        inv(2, 0) = 0;
        inv(2, 1) = 0;
        inv(2, 2) = 1;

        return inv;
    }
};

//────────────────────────────────────────────────────────────────────────────
// FxArray: numpy style array
//────────────────────────────────────────────────────────────────────────────

// 1) define the concepts
template<typename T>
concept NumericOrFxVec =
       std::integral<T>               // all integer types
    || std::floating_point<T>          // float, double, long double
    || std::same_as<T, FxVec2f>
    || std::same_as<T, FxVec3f>
    || std::same_as<T, FxVec4f>;
    // || std::same_as<T, FxVecXf>;

template<typename T>
concept FxVecT =
       std::same_as<T, FxVec2f>
    || std::same_as<T, FxVec3f>
    || std::same_as<T, FxVec4f>;     


template<typename U, typename T>
concept ConvertibleOrNumeric = std::convertible_to<U, T> || Numeric<U>;

// Custom deleter for aligned arrays
template<class T>
struct FxArrayAlignedDelete {
    std::size_t align;
    void operator()(T* p) const noexcept {
        ::operator delete[](p, std::align_val_t(align));
    }
};

// Helper function to create aligned arrays
template<typename T>
static std::unique_ptr<T[], FxArrayAlignedDelete<T>>
FxArray_make_aligned(std::size_t n, std::size_t align = 32) {
    if (n == 0) return {nullptr, FxArrayAlignedDelete<T>{align}};
    // allocation (ctors run)
    T* p = static_cast<T*>(::operator new[](n * sizeof(T), std::align_val_t(align)));
    return std::unique_ptr<T[], FxArrayAlignedDelete<T>>(p, FxArrayAlignedDelete<T>{align});
}

// define the FxArray class template
template<NumericOrFxVec T>
class FxArray {
  private:
    static constexpr std::size_t kAlign = 32;                 // compile-time alignment
    std::size_t m_size = 0;
    std::unique_ptr<T[], FxArrayAlignedDelete<T>> m_arr;

  protected:
    // throws if index is out of bounds
    template<std::integral I>
    std::size_t checkIndex(I idx) const {
        auto i = static_cast<long long>(idx);         // signed capture for error text
        auto u = static_cast<std::size_t>(i);         // cast for bounds test
        if (i < 0 || u >= m_size) {
            throw std::out_of_range(
                "FxArray::[] index " + std::to_string(i)
           + " out of range for [0," + std::to_string(m_size) + ")");
        }
        return u;
    }

    // throws if empty
    void throw_if_empty(char const* what) const {
        if (m_size == 0)
        throw std::runtime_error(std::string("FxArray::") + what + " on empty array");
    } 

    // throws if size mismatch
    void throw_if_size_mismatch(char const* what, size_t o_size) const {
        if (m_size != o_size)
        throw std::invalid_argument(std::string("FxArray::operator") + what + " size mismatch");
    } 

    // single scan to find (index, value) of best element
    template<typename Compare>
    std::pair<std::size_t, T>  best_pair(Compare cmp, const char* name) const {
        throw_if_empty(name);
        T const* __restrict p = aligned_data();
        std::size_t bestIdx = 0;
        T bestVal = p[0];
        for (std::size_t i = 1; i < m_size; ++i) {
            if (cmp(p[i], bestVal)) {
                bestVal = p[i];
                bestIdx = i;
            }
        }
        return {bestIdx, bestVal};
    }

  public:
    // 1) default (empty)
    FxArray() : m_arr(nullptr, FxArrayAlignedDelete<T>{kAlign}) {}
    // 2) n sized ctor (all zeros)
    explicit FxArray(std::size_t n)
      : m_size(n)
      , m_arr(FxArray_make_aligned<T>(n, kAlign))
    {}

    // 3) Dedicated init_list ctor — for braced lists
    FxArray(std::initializer_list<T> init)
      : FxArray(init.size())
    { std::copy(init.begin(), init.end(), aligned_data()); }

    // 4) one ctor for std::vector
    FxArray(std::vector<T> const& v)
      : FxArray(v.size())
    { std::copy(v.begin(), v.end(), aligned_data()); }

    // 5) one overload for c style array
    template<std::size_t N>
    FxArray(T const (&arr)[N])
      : FxArray(N)
    { std::copy_n(arr, N, aligned_data()); }

    // deep‐copy copy‐ctor using copy_n
    FxArray(FxArray const& o)
      : FxArray(o.m_size)
    { std::copy_n(o.aligned_data(), m_size, aligned_data()); }

    // move-assignment (O(1) swap of pointers and size)
    FxArray(FxArray&&) noexcept = default;

    // Default destructor is sufficient now
    ~FxArray() = default;

    // = move-assignment
    FxArray& operator=(FxArray const& o) {
      if (&o == this) return *this;
      FxArray tmp(o);
      swap(tmp);
      return *this;
    }

    // assign from std::vector<T>
    FxArray& operator=(std::vector<T> const& v) {
        // reuse your vector‐ctor + swap
        FxArray tmp(v);
        swap(tmp);
        return *this;
    }

    // assign from initializer_list<T>
    FxArray& operator=(std::initializer_list<T> init) {
        FxArray tmp(init);
        swap(tmp);
        return *this;
    }

    FxArray& operator=(FxArray&&) noexcept = default;

    // Aligned raw-data helpers 
    T*       aligned_data()       noexcept { return std::assume_aligned<kAlign>(m_arr.get()); }
    T const* aligned_data() const noexcept { return std::assume_aligned<kAlign>(m_arr.get()); }


    // 1) UNCHECKED, inlined, noexcept operator[] for hot loops
    T&       operator[](size_t i)       noexcept { return aligned_data()[i]; }
    T const& operator[](size_t i) const noexcept { return aligned_data()[i]; }

    template<std::integral I>
    T& operator()(I i) noexcept { return aligned_data()[static_cast<size_t>(i)]; }

    template<std::integral I>
    T const& operator()(I i) const noexcept { return aligned_data()[static_cast<size_t>(i)]; }
    
    // 2) BOUNDS-CHECKED at(), still accepts *any* integral index
    template<std::integral I>
    T& at(I idx) { return aligned_data()[checkIndex(idx)]; }
    template<std::integral I>
    T const& at(I idx) const { return aligned_data()[checkIndex(idx)]; }

    // Iterators use aligned_data (helps vectorizers in range-for)
    T*       begin()       noexcept { return aligned_data(); }
    T const* begin() const noexcept { return aligned_data(); }
    T*       end()         noexcept { return aligned_data() + m_size; }
    T const* end()   const noexcept { return aligned_data() + m_size; }

    // size & raw data
    size_t size()  const noexcept { return m_size; }
    bool   empty() const noexcept { return m_size == 0; }
    T*       data()       noexcept      { return m_arr.get(); }
    T const* data() const noexcept      { return m_arr.get(); }

    void swap(FxArray& o) noexcept {
      std::swap(m_size,  o.m_size);
      std::swap(m_arr,  o.m_arr);
    }

    template<typename U> requires (Numeric<T> && Numeric<U>)
    FxArray<U> as() const {
        FxArray<U> result(m_size);
        T const* __restrict src = aligned_data();
        U* __restrict dst = result.aligned_data();
        for (std::size_t i = 0; i < m_size; ++i)
            dst[i] = static_cast<U>(src[i]);
        return result;
    }

    template<typename Compare>
    T min(Compare cmp) const {
        return best_pair(cmp, "min").second;
    }

    template<typename Compare>
    T max(Compare cmp) const {
        return best_pair([&](const T& a, const T& b){ return cmp(b,a); }, "max").second;
    }

    // default operator< versions
    T min() const { return min(std::less<T>{}); }
    T max() const { return max(std::less<T>{}); }

    // ---------- index-only -> reuse best_pair ----------
    template<typename Compare>
    std::pair<std::size_t, T> argmin(Compare cmp) const {
        return best_pair(cmp, "argmin");
    }

    template<typename Compare>
    std::pair<std::size_t, T> argmax(Compare cmp) const {
        return best_pair([&](const T& a, const T& b){ return cmp(b,a); }, "argmax");
    }

    // default operator< versions
    std::pair<std::size_t, T> argmin() const { return argmin(std::less<T>{}); }
    std::pair<std::size_t, T> argmax() const { return argmax(std::less<T>{}); }

    // --- mean: requires T() + T+= U + T /= scalar ---
    T mean() const {
        throw_if_empty("mean");
        T const* __restrict p = aligned_data();
        T sum = p[0];                     // initialize with first element
        for (std::size_t i = 1; i < m_size; ++i)
            sum += p[i];
        return sum / m_size;
    }

    // mean as float (casts each element to double for accuracy, returns float)
    float meanf() const requires Numeric<T> {
        throw_if_empty("meanf");
        T const* __restrict p = aligned_data();
        double sum = 0.0;
        for (std::size_t i = 0; i < m_size; ++i)
            sum += static_cast<double>(p[i]);
        return static_cast<float>(sum / m_size);
    }

    //population standard deviation as float
    float stddev() const requires Numeric<T> {
        throw_if_empty("stddev");
        double m = meanf();
        T const* __restrict p = aligned_data();
        double acc = 0.0;
        for (std::size_t i = 0; i < m_size; ++i) {
            double d = static_cast<double>(p[i]) - m;
            acc += d * d;
        }
        return static_cast<float>( std::sqrt(acc / m_size) );
    }

    // --- unary minus (element‐wise negate) ---
    FxArray operator-() const {
        FxArray result(m_size);
        T const* __restrict src = aligned_data();
        T* __restrict dst = result.aligned_data();
        for (std::size_t i = 0; i < m_size; ++i)
            dst[i] = -src[i];
        return result;
    }

    // --- in-place with a single T ---
    template<typename U> requires ConvertibleOrNumeric<U, T>
    FxArray& operator+=(U const& v) {
        T* __restrict p = aligned_data();
        const T vv = static_cast<T>(v);
        for (std::size_t i = 0; i < m_size; ++i) p[i] += vv;
        return *this;
    }

    template<typename U> requires ConvertibleOrNumeric<U, T>
    FxArray& operator-=(U const& v) {
        T* __restrict p = aligned_data();
        const T vv = static_cast<T>(v);
        for (std::size_t i = 0; i < m_size; ++i) p[i] -= vv;
        return *this;
    }

    template<typename U> requires ConvertibleOrNumeric<U, T>
    FxArray& operator*=(U const& v) {
        T* __restrict p = aligned_data();
        const T vv = static_cast<T>(v);
        for (std::size_t i = 0; i < m_size; ++i) p[i] *= vv;
        return *this;
    }

    template<typename U> requires ConvertibleOrNumeric<U, T>
    FxArray& operator/=(U const& v) {
        T* __restrict p = aligned_data();
        const T vv = static_cast<T>(v);
        for (std::size_t i = 0; i < m_size; ++i) p[i] /= vv;
        return *this;
    }

    // --- in-place element‐wise with another FxArray ---
    template<typename U> requires ConvertibleOrNumeric<U, T>
    FxArray& operator+=(FxArray<U> const& o) {
        throw_if_size_mismatch("+=", o.size());
        T* __restrict p = aligned_data();
        U const* __restrict op = o.aligned_data();
        for (std::size_t i = 0; i < m_size; ++i)
            p[i] += op[i];
        return *this;
    }

    template<typename U> requires ConvertibleOrNumeric<U, T>
    FxArray& operator-=(FxArray<U> const& o) {
        throw_if_size_mismatch("-=", o.size());
        T* __restrict p = aligned_data();
        U const* __restrict op = o.aligned_data();
        for (std::size_t i = 0; i < m_size; ++i)
            p[i] -= op[i];
        return *this;
    }

    template<typename U> requires ConvertibleOrNumeric<U, T>
    FxArray& operator*=(FxArray<U> const& o) {
        throw_if_size_mismatch("*=", o.size());
        T* __restrict p = aligned_data();
        U const* __restrict op = o.aligned_data();
        for (std::size_t i = 0; i < m_size; ++i)
            p[i] *= op[i];
        return *this;
    }

    template<typename U> requires ConvertibleOrNumeric<U, T>
    FxArray& operator/=(FxArray<U> const& o) {
        throw_if_size_mismatch("/=", o.size());
        T* __restrict p = aligned_data();
        U const* __restrict op = o.aligned_data();
        for (std::size_t i = 0; i < m_size; ++i)
            p[i] /= op[i];
        return *this;
    }

    // dot with one FxVec2f → returns FxArray<float>
    FxArray<float> dot(T const& v) const requires FxVecT<T>{
        FxArray<float> out(this->size());
        T const* __restrict p = aligned_data();
        float* __restrict result = out.aligned_data();
        for (size_t i = 0; i < this->size(); ++i)
            result[i] = p[i].dot(v);
        return out;
    }

    // element-wise dot with another FxVec2fArray
    FxArray<float> dot(FxArray<T> const& o) const requires FxVecT<T>{
        this->throw_if_size_mismatch("dot", o.size());
        FxArray<float> out(this->size());
        T const* __restrict p = aligned_data();
        T const* __restrict op = o.aligned_data();
        float* __restrict result = out.aligned_data();
        for (size_t i = 0; i < this->size(); ++i)
            result[i] = p[i].dot(op[i]);
        return out;
    }

    // 1) In-place rotate by radians
    FxArray& rotate_inplace_rad(float theta_rad) noexcept requires std::same_as<T, FxVec2f> {
        const float c = std::cos(theta_rad);
        const float s = std::sin(theta_rad);
        T* __restrict p = aligned_data();
        for (std::size_t i = 0; i < m_size; ++i) {
            float xi = p[i].x(), yi = p[i].y();
            p[i].x() = xi * c - yi * s;
            p[i].y() = xi * s + yi * c;
        }
        return *this;
    }

    // 2) In-place rotate by degrees
    FxArray& rotate_inplace(float degrees) noexcept requires std::same_as<T, FxVec2f> {
        constexpr float FX_DEG2RAD = FxPif / 180.0f;
        return rotate_inplace_rad(degrees * FX_DEG2RAD);
    }

    // 3) In-place rotate by degrees
    FxArray& perp_inplace() noexcept requires std::same_as<T, FxVec2f> {
        T* __restrict p = aligned_data();
        for (std::size_t i = 0; i < m_size; ++i) p[i] = p[i].perp();
        return *this;
    }

    // 4) Non-mutating perp → new array
    FxArray perp() const requires std::same_as<T, FxVec2f> {
        FxArray tmp = *this;
        tmp.perp_inplace();
        return tmp;
    }

    // 5) Non-mutating rotate by radians → new array
    FxArray rotate_rad(float theta_rad) const requires std::same_as<T, FxVec2f> {
        FxArray tmp = *this;
        tmp.rotate_inplace_rad(theta_rad);
        return tmp;
    }

    // 6) Non-mutating rotate by degrees → new array
    FxArray rotate(float degrees) const requires std::same_as<T, FxVec2f> {
        FxArray tmp = *this;
        tmp.rotate_inplace(degrees);
        return tmp;
    }

    FxArray<float> bounds() const requires std::same_as<T, FxVec2f>{
        this->throw_if_empty("extrema");
        T const* __restrict p = aligned_data();
        const auto& v0 = p[0];
        float minx = v0.x(), maxx = v0.x();
        float miny = v0.y(), maxy = v0.y();
        for (size_t i = 1, n = this->size(); i < n; ++i) {
            const auto& v = p[i];
            float x = v.x(), y = v.y();
            if (x < minx)      minx = x;
            else if (x > maxx) maxx = x;
            if (y < miny)      miny = y;
            else if (y > maxy) maxy = y;
        }
        return { minx, miny, maxx, maxy };
    }

};

template<NumericOrFxVec T>
void swap(FxArray<T>& a, FxArray<T>& b) noexcept {
  a.swap(b);
}


//────────────────────────────────────────────────────────────────────────────
// FxVec2fArray: fixed‐size array of FxVec2f
//────────────────────────────────────────────────────────────────────────────
using FxVec2fArray = FxArray<FxVec2f>;

// Scalar–Array
template<typename T, typename U> requires ConvertibleOrNumeric<U, T>
inline FxArray<T> operator+(FxArray<T> a, U const& scalar) { return a += scalar; }

template<typename T, typename U> requires ConvertibleOrNumeric<U, T>
inline FxArray<T> operator-(FxArray<T> a, U const& scalar) { return a -= scalar; }

template<typename T, typename U> requires ConvertibleOrNumeric<U, T>
inline FxArray<T> operator*(FxArray<T> a, U const& scalar) { return a *= scalar; }

template<typename T, typename U> requires ConvertibleOrNumeric<U, T>
inline FxArray<T> operator/(FxArray<T> a, U const& scalar) { return a /= scalar; }

// Scalar on the left
template<typename T, typename U> requires ConvertibleOrNumeric<U, T>
inline FxArray<T> operator+(U const& scalar, FxArray<T> a) { return a += scalar; }

template<typename T, typename U> requires ConvertibleOrNumeric<U, T>
inline FxArray<T> operator*(U const& scalar, FxArray<T> a) { return a *= scalar; }

template<typename T, typename U> requires ConvertibleOrNumeric<U, T>
inline FxArray<T> operator-(U const& scalar, FxArray<T> a) { return -a + scalar; }

// Scalar–Array division: scalar / array
template<typename T, Numeric U> 
inline FxArray<T> operator/(U const& scalar, FxArray<T> const& a) {
  FxArray<T> result(a.size());
  T const* __restrict src = a.aligned_data();
  T* __restrict dst = result.aligned_data();
  for (size_t i = 0; i < result.size(); ++i)
    dst[i] = scalar / src[i];
  return result;
}


template<typename T>
inline FxArray<T> operator/(T const& scalar, FxArray<T> const& a) {
  FxArray<T> result(a.size());
  T const* __restrict src = a.aligned_data();
  T* __restrict dst = result.aligned_data();
  for (size_t i = 0; i < result.size(); ++i)
    dst[i] = scalar / src[i];
  return result;
}

// Array–Array
template<typename T>
inline FxArray<T> operator+(FxArray<T> a, const FxArray<T>& b) { return a += b; }

template<typename T>
inline FxArray<T> operator-(FxArray<T> a, const FxArray<T>& b) { return a -= b; }

template<typename T>
inline FxArray<T> operator*(FxArray<T> a, const FxArray<T>& b) { return a *= b; }

template<typename T>
inline FxArray<T> operator/(FxArray<T> a, const FxArray<T>& b) { return a /= b; }

template<Numeric T>
inline std::ostream& operator<<(std::ostream& os, FxArray<T> const& a) {
    os << "FxArray { ";
    for (size_t i = 0; i < a.size(); ++i) os << a[i] << ", ";
    os << "}";
    return os;
}

inline std::ostream& operator<<(std::ostream& os, FxVec2fArray const& a) {
    os << "FxVec2fArray { ";
    for(size_t i = 0; i < a.size(); ++i)
        os << "("<<a[i].x()<<" "<<a[i].y()<<"), ";
    os <<"} ";
    return os;
}

inline std::ostream& operator<<(std::ostream& os, FxVec2f const& a) {
    os << "FxVec2f { ";
    os <<a.x()<<" "<<a.y();
    os <<" }";
    return os;
}

inline std::ostream& operator<<(std::ostream& os, FxVec3f const& a) {
    os << "FxVec3f { ";
    os <<a.x()<<" "<<a.y()<<" "<<a.z();
    os <<" }";
    return os;
}

//---------------------------------------------
// Shape Definition
//
// All shapes share a unified storage model:
//   * m_vertices   — local core feature points (empty for circle, 2 for capsule, N>=3 for polygon)
//   * m_skin_radius — Minkowski-sum "skin" radius added uniformly around the core feature
//
// Circle           = no vertices,    skin_radius = r
// Capsule          = 2 endpoints,    skin_radius = r  (zero skin => bare line segment)
// Polygon          = N>=3 vertices,  skin_radius = 0
// Rounded polygon  = N>=3 vertices,  skin_radius > 0 (covers rounded rectangles)
//---------------------------------------------
enum class FxShapeType {
    Circle,
    Capsule,
    Polygon
};

struct FxShape {
  protected:
    FxShapeType    m_shape_type;                     // Circle, Capsule, or Polygon
    float          m_radius;                         // bounding radius from centroid (skin-inclusive)
    float          m_skin_radius = 0.0f;             // Minkowski-sum skin (rounding) radius
    FxVec2fArray   m_vertices;                       // local vertices: 0 (circle), 2 (capsule), or >=3 (polygon)
    FxVec3f        m_offset_pose {0.0f, 0.0f, 0.0f}; // initial offset pose in world coordinates
    FxVec3f        m_world_pose {0.0f, 0.0f, 0.0f};  // current pose in the world
    FxVec2f        m_centroid {0.0f, 0.0f};          //
    FxVec2fArray   m_world_vertices;

    // 1) Compute the bounding radius from (0,0)
    static float calc_radius(const FxVec2fArray& verts) {
        float maxSq = 0.0f;
        for (size_t i = 0; i < verts.size(); ++i) {
            const FxVec2f& v = verts[i];
            float d2 = v.x()*v.x() + v.y()*v.y();
            if (d2 > maxSq) maxSq = d2;
        }
        return std::sqrt(maxSq);
    }

    // 2) Shoelace‐formula signed area (returns signed area, caller checks validity)
    static float polygon_area(const FxVec2fArray& verts) {
        double sum = 0.0;
        const size_t n = verts.size();
        for (size_t i = 0; i < n; ++i) {
            const FxVec2f& a = verts[i];
            const FxVec2f& b = verts[(i + 1) % n];
            sum += double(a.x()) * b.y() - double(b.x()) * a.y();
        }
        return float(0.5 * sum); // >0 ⇒ CCW, <0 ⇒ CW
    }

    // 3) Convexity: all cross‐products have same sign
    static bool is_convex(const FxVec2fArray& verts) {
        size_t n = verts.size();
        bool gotPos = false, gotNeg = false;
        for (size_t i = 0; i < n; ++i) {
            const FxVec2f& A = verts[i];
            const FxVec2f& B = verts[(i+1)%n];
            const FxVec2f& C = verts[(i+2)%n];
            float cross =
                (B.x()-A.x())*(C.y()-B.y()) -
                (B.y()-A.y())*(C.x()-B.x());
            if      (cross > 0) gotPos = true;
            else if (cross < 0) gotNeg = true;
            if (gotPos && gotNeg) return false;
        }
        return true;
    }

  public:
    // default ctor
    FxShape() : m_shape_type(FxShapeType::Circle), m_radius(0.5f), m_skin_radius(0.5f) {}

    //–– Circle ctor: unified as a 0-vertex shape with skin_radius = radius
    FxShape(float radius) {
        if (radius <= 1e-6f)
            throw std::invalid_argument("FxShape: radius must be > 0");
        m_shape_type  = FxShapeType::Circle;
        m_radius      = radius;
        m_skin_radius = radius;
    }

    //–– Capsule ctor: segment of given length (along x in local frame) with end-cap radius.
    //   length == 0 collapses to a circle of the same radius. radius == 0 yields a bare segment.
    FxShape(float length, float radius) {
        if (radius < 0.0f)
            throw std::invalid_argument("FxShape: capsule radius must be >= 0");
        if (length < 0.0f)
            throw std::invalid_argument("FxShape: capsule length must be >= 0");
        if (length <= 1e-6f && radius <= 1e-6f)
            throw std::invalid_argument("FxShape: degenerate capsule (zero length and radius)");
        const float hl = length * 0.5f;
        m_shape_type     = FxShapeType::Capsule;
        m_vertices       = { { -hl, 0.0f }, {  hl, 0.0f } };
        m_skin_radius    = radius;
        m_radius         = hl + radius;
        m_world_vertices = m_vertices;
    }

    //–– Polygon from arbitrary vertices, with optional uniform skin (rounding) radius
    FxShape(const FxVec2fArray& vertices, float skin_radius = 0.0f) {
        constexpr float minArea = 1e-6f;
        if (vertices.size() < 3)
            throw std::invalid_argument("FxShape: less than 3 vertices");
        if (skin_radius < 0.0f)
            throw std::invalid_argument("FxShape: skin radius must be >= 0");
        float area = polygon_area(vertices);
        if (std::fabs(area) <= minArea)
            throw std::invalid_argument("FxShape: area ≤ 2e-6");
        if (!is_convex(vertices))
            throw std::invalid_argument("FxShape: not convex");
        m_shape_type = FxShapeType::Polygon;
        // centroid will be pushed to {0.0f, 0.0f}
        FxVec2fArray verts = vertices;
        if (area > 0.0f) { // saved in CCW order only
            std::reverse(verts.begin(), verts.end());
        }
        m_vertices       = verts - verts.mean();
        m_skin_radius    = skin_radius;
        m_radius         = calc_radius(m_vertices) + skin_radius;
        m_world_vertices = m_vertices;
    }

    //–– Rectangle centered at origin, width=size.x(), height=size.y(); optional rounded corners
    FxShape(const FxVec2f& size, float skin_radius = 0.0f) {
        if (size.x() <= 0.0f || size.y() <= 0.0f)
            throw std::invalid_argument("FxShape: dimensions must be > 0");
        if (skin_radius < 0.0f)
            throw std::invalid_argument("FxShape: skin radius must be >= 0");
        float hx = size.x() * 0.5f;
        float hy = size.y() * 0.5f;
        // Check for valid area
        if (hx*hy <= 1e-6f)
            throw std::runtime_error("FxShape: degenerate rectangle");
         // build CCW rectangle around (0, 0)
        m_vertices       = { { -hx, -hy }, {  -hx, hy }, {  hx,  hy }, { hx,  -hy }};
        m_shape_type     = FxShapeType::Polygon;
        m_skin_radius    = skin_radius;
        m_radius         = std::sqrt(hx*hx + hy*hy) + skin_radius;
        m_world_vertices = m_vertices;
    }

    // getters for shape properties
    FxShapeType shape_type() const { return m_shape_type; }
    float radius() const { return m_radius; }
    float skin_radius() const { return m_skin_radius; }
    FxVec2fArray vertices() const { return m_world_vertices; }
    FxVec2fArray __vertices() const { return m_vertices; } // native coordinates of vertices with centroid as (0,0)
    FxVec2f centroid() const {  return m_centroid;}

    // methods to check shape type
    bool is_circle() const {
        return m_shape_type == FxShapeType::Circle;
    }

    bool is_capsule() const {
        return m_shape_type == FxShapeType::Capsule;
    }

    bool is_polygon() const {
        return m_shape_type == FxShapeType::Polygon;
    }

    // Get area of the shape (handles circle, capsule, and polygon — skin radius included)
    float area() const {
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
        // Skin contribution = perimeter * r + pi r^2 (full disc from summing exterior corner angles)
        float perim = 0.0f;
        const auto& V = m_vertices;
        for (std::size_t i = 0, n = V.size(); i < n; ++i) {
            perim += (V[(i + 1) % n] - V[i]).norm();
        }
        return core + perim * m_skin_radius + FxPif * m_skin_radius * m_skin_radius;
    }

    // Calculate moment of inertia for given mass (uniform density)
    float calc_inertia(float mass) const {
        if (is_circle()) {
            return 0.5f * mass * m_skin_radius * m_skin_radius;
        }
        if (is_capsule()) {
            // Uniform-density capsule: rectangle (L x 2r) + full disc (radius r) split between caps.
            const float L = (m_vertices[1] - m_vertices[0]).norm();
            const float r = m_skin_radius;
            const float A_rect = 2.0f * r * L;
            const float A_caps = FxPif * r * r;
            const float A_tot  = A_rect + A_caps;
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
        const float density    = mass / total_area;
        const float I_core     = (density / 12.0f) * std::abs(accum);
        // Skin mass approximated as a ring at the bounding radius.
        const float m_skin   = mass - density * core_area;
        const float r_eff_sq = m_radius * m_radius - m_skin_radius * m_skin_radius * 0.5f;
        return I_core + m_skin * std::max(0.0f, r_eff_sq);
    }

    // offset pose setter and getter 
    void set_offset_pose(const FxVec3f& o_pose){ m_offset_pose = o_pose; }
    FxVec3f offset_pose() const { return m_offset_pose; }

    // Returns current axis aligned bounding box of the shape and sets world pose
    FxArray<float> set_world_pose(const FxVec3f& world_pose){
        m_world_pose = world_pose;
        m_centroid = world_pose.xy() + m_offset_pose.xy();
        if (is_circle()) {
            float pX = m_centroid.x();
            float pY = m_centroid.y();
            float r  = m_skin_radius;
            return {pX - r, pY - r, pX + r, pY + r}; // AABB for circle
        }
        // Capsule and polygon: rotate local vertices into world frame, then inflate AABB by skin.
        m_world_vertices = m_vertices.rotate_rad(world_pose.theta() + m_offset_pose.theta());
        m_world_vertices += m_centroid;
        FxArray<float> bb = m_world_vertices.bounds();
        if (m_skin_radius > 0.0f) {
            bb[0] -= m_skin_radius; bb[1] -= m_skin_radius;
            bb[2] += m_skin_radius; bb[3] += m_skin_radius;
        }
        return bb;
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

    // Skin-inclusive projection interval [min, max] of the shape along an axis.
    FxArray<float> project_onto(const FxVec2f& axis) const {
        if (is_circle()) {
            float p = m_centroid.dot(axis);
            return {p - m_skin_radius, p + m_skin_radius};
        }
        FxArray<float> raw = m_world_vertices.dot(axis);
        float lo = raw[0], hi = raw[0];
        for (std::size_t i = 1; i < raw.size(); ++i) {
            if (raw[i] < lo) lo = raw[i];
            if (raw[i] > hi) hi = raw[i];
        }
        return {lo - m_skin_radius, hi + m_skin_radius};
    }

    // Per-vertex raw projection along axis with origin shifted (no skin applied).
    // SAT routines subtract the sum of both shapes' skin radii explicitly.
    FxArray<float> project_onto(const FxVec2f& axis, const FxVec2f& origin) const {
        if (is_circle()) {
            float p = (m_centroid - origin).dot(axis);
            // For circles, treat the centroid as a single "vertex" so argmin works uniformly.
            return {p, p};
        }
        return (m_world_vertices - origin).dot(axis);
    }

    //get the closest vertex of the shape from a point (returns surface point, skin-inclusive)
    FxVec2f get_closest_vertex(const FxVec2f& point) const {
        if (is_circle()) {
            FxVec2f v = point - m_centroid;     // vector from center to query point
            FxVec2f dir;
            if (v.dot(v) < 1e-6f) dir = FxVec2f(1.0f, 0.0f);      // arbitrary unit vector
            else dir = v.normalized();                         // safe to normalize
            return m_centroid + dir * m_skin_radius;
        }
        if (is_capsule()) {
            // Closest point on the capsule's central segment, then pushed out by skin radius.
            const FxVec2f& a = m_world_vertices[0];
            const FxVec2f& b = m_world_vertices[1];
            FxVec2f ab = b - a;
            float len2 = ab.dot(ab);
            FxVec2f q = (len2 < 1e-6f)
                        ? a
                        : a + std::clamp((point - a).dot(ab) / len2, 0.0f, 1.0f) * ab;
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
