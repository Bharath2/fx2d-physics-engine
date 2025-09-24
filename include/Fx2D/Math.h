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
    while (angle >= FxPif) angle -= 2.0f * FxPif;
    while (angle < -FxPif) angle += 2.0f * FxPif;
    return angle;
}


// concept to capture float, double, long double, int etc..
template<typename T>
concept Numeric = std::integral<T> || std::floating_point<T>; 

// Custom 2D float vector with .x() getter and .set_x() setter.
class FxVec2f : public Eigen::Vector2f {
public:
    // Inherit constructors
    using Eigen::Vector2f::Vector2f;

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



// define the FxArray class template
template<NumericOrFxVec T>
class FxArray {
  private:
    std::size_t    m_size  = 0;
    std::unique_ptr<T[]> m_arr;

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
        std::size_t bestIdx = 0;
        T bestVal = m_arr[0];
        for (std::size_t i = 1; i < m_size; ++i) {
            if (cmp(m_arr[i], bestVal)) {
                bestVal = m_arr[i];
                bestIdx = i;
            }
        }
        return {bestIdx, bestVal};
    }

  public:
    // 1) default (empty)
    FxArray() = default;
    // 2) n sized ctor (all zeros)
    explicit FxArray(std::size_t n)
      : m_size(n)
      , m_arr(std::make_unique<T[]>(n))
    {}

    // 3) Dedicated init_list ctor — for braced lists
    FxArray(std::initializer_list<T> init)
      : FxArray(init.size())
    { std::copy(init.begin(), init.end(), m_arr.get()); }

    // 4) one ctor for std::vector
    FxArray(std::vector<T> const& v)
      : FxArray(v.size())
    { std::copy(v.begin(), v.end(), m_arr.get()); }

    // 5) one overload for c style array
    template<std::size_t N>
    FxArray(T const (&arr)[N])
      : FxArray(N)
    { std::copy_n(arr, N, m_arr.get()); }

    // deep‐copy copy‐ctor using copy_n
    FxArray(FxArray const& o)
      : FxArray(o.m_size)
    { std::copy_n(o.m_arr.get(), m_size, m_arr.get()); }

    // move-assignment (O(1) swap of pointers and size)
    FxArray(FxArray&&) noexcept = default;

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

    // 1) UNCHECKED, inlined, noexcept operator[] for hot loops
    T&       operator[](size_t i)       noexcept { return m_arr[i]; }
    T const& operator[](size_t i) const noexcept { return m_arr[i]; }

    template<std::integral I>
    T& operator()(I i) noexcept { return m_arr[static_cast<size_t>(i)]; }

    template<std::integral I>
    T const& operator()(I i) const noexcept { return m_arr[static_cast<size_t>(i)]; }
    
    // 2) BOUNDS-CHECKED at(), still accepts *any* integral index
    template<std::integral I>
    T& at(I idx) { return m_arr[checkIndex(idx)]; }
    template<std::integral I>
    T const& at(I idx) const { return m_arr[checkIndex(idx)]; }

    // for iterators
    T*       begin()       noexcept { return data(); }
    T const* begin() const noexcept { return data(); }
    T*       end()         noexcept { return data() + m_size; }
    T const* end()   const noexcept { return data() + m_size; }

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
        for (std::size_t i = 0; i < m_size; ++i)
            result[i] = static_cast<U>(m_arr[i]);
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
        T sum = m_arr[0];                     // initialize with first element
        for (std::size_t i = 1; i < m_size; ++i)
            sum += m_arr[i];
        return sum / m_size;
    }

    // mean as float (casts each element to double for accuracy, returns float)
    float meanf() const requires Numeric<T> {
        throw_if_empty("meanf");
        double sum = 0.0;
        for (std::size_t i = 0; i < m_size; ++i)
            sum += static_cast<double>(m_arr[i]);
        return static_cast<float>(sum / m_size);
    }

    //population standard deviation as float
    float stddev() const requires Numeric<T> {
        throw_if_empty("stddev");
        double m = meanf();
        double acc = 0.0;
        for (std::size_t i = 0; i < m_size; ++i) {
            double d = static_cast<double>(m_arr[i]) - m;
            acc += d * d;
        }
        return static_cast<float>( std::sqrt(acc / m_size) );
    }

    // --- unary minus (element‐wise negate) ---
    FxArray operator-() const {
        FxArray result(m_size);
        for (std::size_t i = 0; i < m_size; ++i)
            result.m_arr[i] = -m_arr[i];
        return result;
    }

    // --- in-place with a single T ---
    template<typename U> requires ConvertibleOrNumeric<U, T>
    FxArray& operator+=(U const& v) {
        for (std::size_t i = 0; i < m_size; ++i) m_arr[i] += v;
        return *this;
    }

    template<typename U> requires ConvertibleOrNumeric<U, T>
    FxArray& operator-=(U const& v) {
        for (std::size_t i = 0; i < m_size; ++i) m_arr[i] -= v;
        return *this;
    }

    template<typename U> requires ConvertibleOrNumeric<U, T>
    FxArray& operator*=(U const& v) {
        for (std::size_t i = 0; i < m_size; ++i) m_arr[i] *= v;
        return *this;
    }

    template<typename U> requires ConvertibleOrNumeric<U, T>
    FxArray& operator/=(U const& v) {
        for (std::size_t i = 0; i < m_size; ++i) m_arr[i] /= v;
        return *this;
    }

    // --- in-place element‐wise with another FxArray ---
    template<typename U> requires ConvertibleOrNumeric<U, T>
    FxArray& operator+=(FxArray<U> const& o) {
        throw_if_size_mismatch("+=", o.size());
        for (std::size_t i = 0; i < m_size; ++i)
            m_arr[i] += o.m_arr[i];
        return *this;
    }

    template<typename U> requires ConvertibleOrNumeric<U, T>
    FxArray& operator-=(FxArray<U> const& o) {
        throw_if_size_mismatch("-=", o.size());
        for (std::size_t i = 0; i < m_size; ++i)
            m_arr[i] -= o.m_arr[i];
        return *this;
    }

    template<typename U> requires ConvertibleOrNumeric<U, T>
    FxArray& operator*=(FxArray<U> const& o) {
        throw_if_size_mismatch("*=", o.size());
        for (std::size_t i = 0; i < m_size; ++i)
            m_arr[i] *= o.m_arr[i];
        return *this;
    }

    template<typename U> requires ConvertibleOrNumeric<U, T>
    FxArray& operator/=(FxArray<U> const& o) {
        throw_if_size_mismatch("/=", o.size());
        for (std::size_t i = 0; i < m_size; ++i)
            m_arr[i] /= o.m_arr[i];
        return *this;
    }

    // dot with one FxVec2f → returns FxArray<float>
    FxArray<float> dot(T const& v) const requires FxVecT<T>{
        FxArray<float> out(this->size());
        for (size_t i = 0; i < this->size(); ++i)
            out[i] = (*this)[i].dot(v);
        return out;
    }

    // element-wise dot with another FxVec2fArray
    FxArray<float> dot(FxArray<T> const& o) const requires FxVecT<T>{
        this->throw_if_size_mismatch("dot", o.size());
        FxArray<float> out(this->size());
        for (size_t i = 0; i < this->size(); ++i)
            out[i] = (*this)[i].dot(o[i]);
        return out;
    }

    // 1) In-place rotate by radians
    FxArray& rotate_inplace_rad(float theta_rad) noexcept requires std::same_as<T, FxVec2f> {
        const float c = std::cos(theta_rad);
        const float s = std::sin(theta_rad);
        for (auto& e : *this) {
            float xi = e.x(), yi = e.y();
            e.x() = xi * c - yi * s;
            e.y() = xi * s + yi * c;
        }
        return *this;
    }

    // 2) In-place rotate by degrees
    FxArray& rotate_inplace(float degrees) noexcept requires std::same_as<T, FxVec2f> {
        constexpr float FX_DEG2RAD = 3.1415926535898f / 180.0f;
        return rotate_inplace_rad(degrees * FX_DEG2RAD);
    }

    // 3) In-place rotate by degrees
    FxArray& perp_inplace() noexcept requires std::same_as<T, FxVec2f> {
        for (auto& e : *this) e = e.perp();
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
        const auto& v0 = (*this)[0];
        float minx = v0.x(), maxx = v0.x();
        float miny = v0.y(), maxy = v0.y();
        for (size_t i = 1, n = this->size(); i < n; ++i) {
            const auto& v = (*this)[i];
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
  for (size_t i = 0; i < result.size(); ++i)
    result[i] = scalar / a[i];
  return result;
}


template<typename T>
inline FxArray<T> operator/(T const& scalar, FxArray<T> const& a) {
  FxArray<T> result(a.size());
  for (size_t i = 0; i < result.size(); ++i)
    result[i] = scalar / a[i];
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
//---------------------------------------------
enum class FxShapeType {
    Circle,
    Polygon
};

struct FxShape {
  protected:
    FxShapeType    m_shape_type;                     // "Circle" or "Polygon"
    float          m_radius;                         // bounding radius 
    FxVec2fArray   m_vertices;                       // only used for polygons relative to centroid
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
    FxShape() : m_shape_type(FxShapeType::Circle), m_radius(0.5f) {}

    //–– Circle ctor
    FxShape(float radius) {
        if (radius <= 1e-6f)
            throw std::invalid_argument("FxShape: radius must be > 0");
        m_shape_type = FxShapeType::Circle;
        m_radius     = radius;
    }

    //–– Polygon from arbitrary vertices
    FxShape(const FxVec2fArray& vertices) {
        constexpr float minArea = 1e-6f;
        if (vertices.size() < 3)
            throw std::invalid_argument("FxShape: less than 3 vertices");
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
        m_vertices   = verts - verts.mean();
        m_radius     = calc_radius(m_vertices);
        m_world_vertices = m_vertices;
    }

    //–– Rectangle centered at origin, width=size.x(), height=size.y()
    FxShape(const FxVec2f& size) {
        if (size.x() <= 0.0f || size.y() <= 0.0f)
            throw std::invalid_argument("FxShape: dimensions must be > 0");
        float hx = size.x() * 0.5f;
        float hy = size.y() * 0.5f;
        // Check for valid area
        if (hx*hy <= 1e-6f)
            throw std::runtime_error("FxShape: degenerate rectangle");
         // build CCW rectangle around (0, 0)
        m_vertices = { { -hx, -hy }, {  -hx, hy }, {  hx,  hy }, { hx,  -hy }};
        m_shape_type = FxShapeType::Polygon;
        m_radius     = std::sqrt(hx*hx + hy*hy);
        m_world_vertices = m_vertices;
    }

    // getters for shape properties
    FxShapeType shape_type() const { return m_shape_type; }
    float radius() const { return m_radius; }
    FxVec2fArray vertices() const { return m_world_vertices; }
    FxVec2fArray __vertices() const { return m_vertices; } // native coordinates of vertices with centroid as (0,0)
    FxVec2f centroid() const {  return m_centroid;}

    // methods to check shape type
    bool is_circle() const {
        return m_shape_type == FxShapeType::Circle;
    }

    bool is_polygon() const {
        return m_shape_type == FxShapeType::Polygon;
    }

    // Get area of the shape (handles both circle and polygon)
    float area() const {
        if (is_circle()) {
            return FxPif * m_radius * m_radius;
        } else {
            return std::abs(polygon_area(m_world_vertices));
        }
    }

    // Calculate moment of inertia for given mass
    float calc_inertia(float mass) const {
        if (is_circle()) {
            return 0.5f * mass * m_radius * m_radius;
        } else {
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

            float area = std::abs(signed_twice_area * 0.5f);
            if (area < 1e-6f) return 0.0f;

            float density = mass / area;
            return (density / 12.0f) * std::abs(accum);
        }
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
            return {pX - m_radius, pY - m_radius, pX + m_radius, pY + m_radius}; // AABB for circle
        } else {
            // rotate vertices by offset and world pose theta
            m_world_vertices = m_vertices.rotate_rad(world_pose.theta() + m_offset_pose.theta()); 
            m_world_vertices += m_centroid;
            return m_world_vertices.bounds();
        }
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

    // project the shape onto an axis
    FxArray<float> project_onto(const FxVec2f& axis) const {
        if (is_circle()) {
            float p = m_centroid.dot(axis);
            return {p - m_radius, p + m_radius};
        } else {
            return (m_world_vertices).dot(axis);
        }
    }

    // project the shape onto a line segment
    FxArray<float> project_onto(const FxVec2f& axis, const FxVec2f& origin) const {
        if (is_circle()) {
            float p = (m_centroid  - origin).dot(axis);
            return {p - m_radius, p + m_radius};
        } else {
            return (m_world_vertices - origin).dot(axis);
        }
    }

    //get the closest vertex of the shape from a point
    FxVec2f get_closest_vertex(const FxVec2f& point) const {
        if (is_circle()) {
            FxVec2f v = point - m_centroid;     // vector from center to query point
            FxVec2f dir;
            if (v.dot(v) < 1e-6f) dir = FxVec2f(1.0f, 0.0f);      // arbitrary unit vector
            else dir = v.normalized();                         // safe to normalize
            return m_centroid + dir * m_radius;
        } else {
            auto shifted = (m_world_vertices - point);
            auto dist = (shifted).dot(shifted);
            auto [min_ind, min_value] = dist.argmin();
            return m_world_vertices[min_ind];
        }
    } 

};
