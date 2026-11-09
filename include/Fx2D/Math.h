#pragma once
#include <algorithm>
#include <cmath>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <initializer_list>
#include <iostream>
#include <limits>
#include <memory>
#include <new>
#include <numbers>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <vector>

// Constant representing positive infinity
static constexpr float FxInfinityf = std::numeric_limits<float>::infinity();
static constexpr double FxInfinityd = std::numeric_limits<double>::infinity();

// Constant representing pi
static constexpr float FxPif = std::numbers::pi_v<float>;
static constexpr double FxPid = std::numbers::pi_v<double>;

// Wrap into [-pi, pi). In-range values pass through untouched (avoids float floor near pi).
static inline float FxAngleWrap(float angle) {
    if (angle >= -FxPif && angle < FxPif) return angle;
    angle = std::fmod(angle + FxPif, 2.0f * FxPif);
    if (angle < 0.0f) angle += 2.0f * FxPif;
    return angle - FxPif;
}

// Add inc to a float accumulator in double, keeping whatever did not survive the rounding in
// carry so the next call folds it back in. Returns the delta that actually landed.
static inline float FxCarryAdd(float& dst, double inc, double& carry) {
    double s = (double)dst + inc + carry;
    float f = (float)s;
    carry = s - (double)f;
    float d = f - dst;
    dst = f;
    return d;
}

// concept to capture float, double, long double, int etc..
template<typename T>
concept Numeric = std::integral<T> || std::floating_point<T>;

// ────────────────────────────────────────────────────────────────────────────
//  FxVec / FxMat: fixed-size vectors and matrices
// ────────────────────────────────────────────────────────────────────────────

// Every operation is written out element by element, so the arithmetic and the order sums are
// taken in are fixed here, the same on every compiler and CPU.
namespace fx_detail {

// Sum of a[i] * b[i]: p0 + p1, p0 + (p1 + p2), (p0 + p2) + (p1 + p3). These are the orders
// Eigen 3.4 produced on x86-64, so dropping Eigen moved no result.
template<typename T, int N>
constexpr T dot(const T* a, const T* b) noexcept {
    if constexpr (N == 2) {
        return a[0] * b[0] + a[1] * b[1];
    } else if constexpr (N == 3) {
        return a[0] * b[0] + (a[1] * b[1] + a[2] * b[2]);
    } else {
        static_assert(N == 4, "dot is defined for 2, 3 and 4 elements");
        return (a[0] * b[0] + a[2] * b[2]) + (a[1] * b[1] + a[3] * b[3]);
    }
}

} // namespace fx_detail

// N-element vector; default construction leaves it uninitialised, like a plain float. Sizes that
// are a multiple of 16 bytes keep the 16-byte alignment Eigen gave them, so layouts are unchanged.
template<typename T, int N>
class alignas((sizeof(T) * N) % 16 == 0 ? 16 : alignof(T)) FxVec {
  public:
    FxVec() = default;
    FxVec(const FxVec&) = default;
    // Assigning to a temporary (pose.xy() = v) would be a lost write, so it does not compile.
    FxVec& operator=(const FxVec&) & = default;

    // Constructor from a single value (fills every component)
    constexpr explicit FxVec(T a) noexcept {
        for (T& e : m_v)
            e = a;
    }

    // Constructor from one value per component
    template<Numeric... A>
        requires(sizeof...(A) == N)
    constexpr FxVec(A... values) noexcept : m_v{static_cast<T>(values)...} {}

    // Component access
    constexpr T& operator[](std::size_t i) noexcept { return m_v[i]; }
    constexpr const T& operator[](std::size_t i) const noexcept { return m_v[i]; }
    constexpr T* data() noexcept { return m_v; }
    constexpr const T* data() const noexcept { return m_v; }

    // Getters.
    constexpr T& x() noexcept { return m_v[0]; }
    constexpr T& y() noexcept { return m_v[1]; }
    constexpr T& z() noexcept
        requires(N >= 3)
    {
        return m_v[2];
    }
    // to use the third value as orientation
    constexpr T& theta() noexcept
        requires(N == 3)
    {
        return m_v[2];
    }
    constexpr T& a() noexcept
        requires(N == 4)
    {
        return m_v[3];
    }

    // Const getters.
    constexpr T x() const noexcept { return m_v[0]; }
    constexpr T y() const noexcept { return m_v[1]; }
    constexpr T z() const noexcept
        requires(N >= 3)
    {
        return m_v[2];
    }
    constexpr T theta() const noexcept
        requires(N == 3)
    {
        return m_v[2];
    }
    constexpr T a() const noexcept
        requires(N == 4)
    {
        return m_v[3];
    }

    // Setters.
    constexpr void set_x(T val) noexcept { m_v[0] = val; }
    constexpr void set_y(T val) noexcept { m_v[1] = val; }
    constexpr void set_z(T val) noexcept
        requires(N >= 3)
    {
        m_v[2] = val;
    }
    // when used as orientation
    constexpr void set_theta(T val) noexcept
        requires(N == 3)
    {
        m_v[2] = val;
    }
    constexpr void set_a(T val) noexcept
        requires(N == 4)
    {
        m_v[3] = val;
    }

    // The position part of a pose, as a copy; write it back with set_xy().
    constexpr FxVec<T, 2> xy() const noexcept
        requires(N == 3)
    {
        return FxVec<T, 2>(m_v[0], m_v[1]);
    }
    constexpr FxVec<T, 2> get_xy() const noexcept
        requires(N == 3)
    {
        return xy();
    }
    constexpr void set_xy(const FxVec<T, 2>& v2) noexcept
        requires(N == 3)
    {
        m_v[0] = v2.x();
        m_v[1] = v2.y();
    }

    // Dot product, squared length and length
    constexpr T dot(const FxVec& other) const noexcept {
        return fx_detail::dot<T, N>(m_v, other.m_v);
    }
    constexpr T squaredNorm() const noexcept { return dot(*this); }
    T norm() const noexcept
        requires std::floating_point<T>
    {
        return std::sqrt(squaredNorm());
    }

    // Unit vector in the same direction; a zero vector comes back unchanged
    FxVec normalized() const noexcept
        requires std::floating_point<T>
    {
        const T sq = squaredNorm();
        if (sq > T(0)) return *this / std::sqrt(sq);
        return *this;
    }

    // Cross product with another 2D vector (returns scalar)
    constexpr T cross(const FxVec& other) const noexcept
        requires(N == 2)
    {
        return x() * other.y() - y() * other.x();
    }

    // Perpendicular vectors
    constexpr FxVec perp() const noexcept
        requires(N == 2)
    {
        return FxVec(-y(), x()); // CCW perpendicular
    }
    constexpr FxVec perpCW() const noexcept
        requires(N == 2)
    {
        return FxVec(y(), -x()); // CW perpendicular
    }

    // Radian‐based rotate (suffix "_rad")
    FxVec& rotate_inplace_rad(T theta) & noexcept
        requires(N == 2 && std::floating_point<T>)
    {
        const T c = std::cos(theta), s = std::sin(theta);
        T xi = x(), yi = y();
        set_x(xi * c - yi * s);
        set_y(xi * s + yi * c);
        return *this;
    }

    // Degree‐based rotate (default "rotate_inplace" uses degrees)
    FxVec& rotate_inplace(T degrees) & noexcept
        requires(N == 2 && std::floating_point<T>)
    {
        constexpr T FX_DEG2RAD = std::numbers::pi_v<T> / T(180);
        return rotate_inplace_rad(degrees * FX_DEG2RAD);
    }

    // — non-mutating rotation: returns a rotated copy
    FxVec rotate(T degrees) const noexcept
        requires(N == 2 && std::floating_point<T>)
    {
        FxVec r = *this;
        r.rotate_inplace(degrees);
        return r;
    }
    FxVec rotate_rad(T theta) const noexcept
        requires(N == 2 && std::floating_point<T>)
    {
        FxVec r = *this;
        r.rotate_inplace_rad(theta);
        return r;
    }

    // In-place operations with another vector (component-wise)
    constexpr FxVec& operator+=(const FxVec& rhs) & noexcept {
        for (int i = 0; i < N; ++i)
            m_v[i] += rhs.m_v[i];
        return *this;
    }
    constexpr FxVec& operator-=(const FxVec& rhs) & noexcept {
        for (int i = 0; i < N; ++i)
            m_v[i] -= rhs.m_v[i];
        return *this;
    }
    constexpr FxVec& operator*=(const FxVec& rhs) & noexcept {
        for (int i = 0; i < N; ++i)
            m_v[i] *= rhs.m_v[i];
        return *this;
    }
    constexpr FxVec& operator/=(const FxVec& rhs) & noexcept {
        for (int i = 0; i < N; ++i)
            m_v[i] /= rhs.m_v[i];
        return *this;
    }

    // In-place scalar operations with generic Numeric s
    template<Numeric S>
    constexpr FxVec& operator+=(S s) & noexcept {
        for (T& e : m_v)
            e += static_cast<T>(s);
        return *this;
    }
    template<Numeric S>
    constexpr FxVec& operator-=(S s) & noexcept {
        for (T& e : m_v)
            e -= static_cast<T>(s);
        return *this;
    }
    template<Numeric S>
    constexpr FxVec& operator*=(S s) & noexcept {
        for (T& e : m_v)
            e *= static_cast<T>(s);
        return *this;
    }
    template<Numeric S>
    constexpr FxVec& operator/=(S s) & noexcept {
        for (T& e : m_v)
            e /= static_cast<T>(s);
        return *this;
    }

    // Vector operations. There is no vector * vector: use dot(), cross() or *=.
    friend constexpr FxVec operator+(FxVec lhs, const FxVec& rhs) noexcept { return lhs += rhs; }
    friend constexpr FxVec operator-(FxVec lhs, const FxVec& rhs) noexcept { return lhs -= rhs; }
    friend constexpr FxVec operator-(FxVec v) noexcept {
        for (T& e : v.m_v)
            e = -e;
        return v;
    }
    friend constexpr bool operator==(const FxVec& lhs, const FxVec& rhs) noexcept {
        for (int i = 0; i < N; ++i)
            if (!(lhs.m_v[i] == rhs.m_v[i])) return false;
        return true;
    }

    // Scalar operations with generic Numeric s
    template<Numeric S>
    friend constexpr FxVec operator*(FxVec v, S s) noexcept {
        return v *= s;
    }
    template<Numeric S>
    friend constexpr FxVec operator*(S s, FxVec v) noexcept {
        return v *= s;
    }
    template<Numeric S>
    friend constexpr FxVec operator/(FxVec v, S s) noexcept {
        return v /= s;
    }
    template<Numeric S>
    friend constexpr FxVec operator+(FxVec v, S s) noexcept {
        return v += s;
    }
    template<Numeric S>
    friend constexpr FxVec operator+(S s, FxVec v) noexcept {
        return v += s;
    }
    template<Numeric S>
    friend constexpr FxVec operator-(FxVec v, S s) noexcept {
        return v -= s;
    }
    template<Numeric S>
    friend constexpr FxVec operator-(S s, FxVec v) noexcept {
        for (T& e : v.m_v)
            e = static_cast<T>(s) - e;
        return v;
    }
    template<Numeric S>
    friend constexpr FxVec operator/(S s, FxVec v) noexcept {
        for (T& e : v.m_v)
            e = static_cast<T>(s) / e;
        return v;
    }

  private:
    T m_v[N];
};

using FxVec2f = FxVec<float, 2>;
using FxVec2d = FxVec<double, 2>;
using FxVec3f = FxVec<float, 3>;
using FxVec3d = FxVec<double, 3>;
using FxVec4f = FxVec<float, 4>;
using FxVec2ui = FxVec<unsigned int, 2>;
using FxVec4ui8 = FxVec<uint8_t, 4>;

static_assert(std::is_trivially_copyable_v<FxVec2f> &&
              std::is_trivially_default_constructible_v<FxVec2f>);
static_assert(sizeof(FxVec3f) == 12 && alignof(FxVec4f) == 16);

// N x N matrix, stored column by column; default construction leaves it uninitialised.
template<typename T, int N>
class alignas((sizeof(T) * N * N) % 16 == 0 ? 16 : alignof(T)) FxMat {
  public:
    FxMat() = default;
    FxMat(const FxMat&) = default;
    FxMat& operator=(const FxMat&) & = default;

    constexpr T& operator()(int row, int col) noexcept { return m_a[col * N + row]; }
    constexpr const T& operator()(int row, int col) const noexcept { return m_a[col * N + row]; }
    constexpr T* data() noexcept { return m_a; }
    constexpr const T* data() const noexcept { return m_a; }

    // Products: each element is a fixed-order dot of a row and a column (see fx_detail::dot)
    friend constexpr FxVec<T, N> operator*(const FxMat& m, const FxVec<T, N>& v) noexcept {
        FxVec<T, N> out;
        for (int row = 0; row < N; ++row)
            out.data()[row] = fx_detail::dot<T, N>(m.row(row).data(), v.data());
        return out;
    }
    friend constexpr FxMat operator*(const FxMat& lhs, const FxMat& rhs) noexcept {
        FxMat out;
        for (int row = 0; row < N; ++row) {
            const FxVec<T, N> r = lhs.row(row);
            for (int col = 0; col < N; ++col)
                out(row, col) = fx_detail::dot<T, N>(r.data(), &rhs.m_a[col * N]);
        }
        return out;
    }

  protected:
    constexpr FxVec<T, N> row(int r) const noexcept {
        FxVec<T, N> out;
        for (int col = 0; col < N; ++col)
            out.data()[col] = (*this)(r, col);
        return out;
    }

  private:
    T m_a[N * N];
};

class FxMat2f : public FxMat<float, 2> {
  public:
    using FxMat<float, 2>::FxMat;
    FxMat2f() = default;
    FxMat2f(const FxMat<float, 2>& m) : FxMat<float, 2>(m) {}

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

    // The inverse (transpose) of the rotation matrix.
    FxMat2f inv_rotation() const {
        FxMat2f t;
        t.set_a(a());
        t.set_b(c());
        t.set_c(b());
        t.set_d(d());
        return t;
    }
};

class FxMat3f : public FxMat<float, 3> {
  public:
    using FxMat<float, 3>::FxMat;
    FxMat3f() = default;
    FxMat3f(const FxMat<float, 3>& m) : FxMat<float, 3>(m) {}

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

    // Upper-left 2x2 rotation of a homogeneous transform [R t; 0 1].
    FxMat2f Rot() const {
        FxMat2f R;
        R.set_a(a());
        R.set_b(b());
        R.set_c(d());
        R.set_d(e());
        return R;
    }
    // Extract the translation vector (first two elements of the third column) as an FxVec2f
    FxVec2f t() const { return FxVec2f((*this)(0, 2), (*this)(1, 2)); }

    // set the rotation part from an FxMat2f
    void set_Rot(const FxMat2f& R) {
        (*this)(0, 0) = R(0, 0);
        (*this)(0, 1) = R(0, 1);
        (*this)(1, 0) = R(1, 0);
        (*this)(1, 1) = R(1, 1);
    }

    // set the translation part from an FxVec2f
    void set_t(const FxVec2f& trans) {
        (*this)(0, 2) = trans.x();
        (*this)(1, 2) = trans.y();
    }

    // Compute and return the inverse transformation.
    FxMat3f inv_transform() const {
        FxMat3f inv; // This will store the inverse.

        // Compute the rotation transpose (which is the inverse of a rotation).
        inv(0, 0) = (*this)(0, 0);
        inv(0, 1) = (*this)(1, 0);
        inv(1, 0) = (*this)(0, 1);
        inv(1, 1) = (*this)(1, 1);

        // Compute the new translation vector as -Rᵀt.
        float t0 = (*this)(0, 2); // original translation x value.
        float t1 = (*this)(1, 2); // original translation y value.
        inv(0, 2) = -(inv(0, 0) * t0 + inv(0, 1) * t1);
        inv(1, 2) = -(inv(1, 0) * t0 + inv(1, 1) * t1);

        // Set the homogeneous row.
        inv(2, 0) = 0;
        inv(2, 1) = 0;
        inv(2, 2) = 1;

        return inv;
    }
};

static_assert(alignof(FxMat2f) == 16 && sizeof(FxMat3f) == 36);

// ────────────────────────────────────────────────────────────────────────────
//  FxArray: numpy style array
// ────────────────────────────────────────────────────────────────────────────

// 1) define the concepts
template<typename T>
concept NumericOrFxVec =
    std::integral<T> // all integer types
    || std::floating_point<T> // float, double, long double
    || std::same_as<T, FxVec2f> || std::same_as<T, FxVec3f> || std::same_as<T, FxVec4f>;
// || std::same_as<T, FxVecXf>;

template<typename T>
concept FxVecT = std::same_as<T, FxVec2f> || std::same_as<T, FxVec3f> || std::same_as<T, FxVec4f>;

template<typename U, typename T>
concept ConvertibleOrNumeric = std::convertible_to<U, T> || Numeric<U>;

// Custom deleter for aligned arrays
template<class T>
struct FxArrayAlignedDelete {
    std::size_t align;
    void operator()(T* p) const noexcept { ::operator delete[](p, std::align_val_t(align)); }
};

// Helper function to create aligned arrays
template<typename T>
static std::unique_ptr<T[], FxArrayAlignedDelete<T>> FxArray_make_aligned(std::size_t n,
                                                                          std::size_t align = 32) {
    if (n == 0) return {nullptr, FxArrayAlignedDelete<T>{align}};
    // allocation (ctors run)
    T* p = static_cast<T*>(::operator new[](n * sizeof(T), std::align_val_t(align)));
    return std::unique_ptr<T[], FxArrayAlignedDelete<T>>(p, FxArrayAlignedDelete<T>{align});
}

// define the FxArray class template
template<NumericOrFxVec T>
class FxArray {
  private:
    static constexpr std::size_t kAlign = 32; // compile-time alignment
    std::size_t m_size = 0;
    std::unique_ptr<T[], FxArrayAlignedDelete<T>> m_arr;

  protected:
    // throws if index is out of bounds
    template<std::integral I>
    std::size_t checkIndex(I idx) const {
        auto i = static_cast<long long>(idx); // signed capture for error text
        auto u = static_cast<std::size_t>(i); // cast for bounds test
        if (i < 0 || u >= m_size) {
            throw std::out_of_range("FxArray::[] index " + std::to_string(i) +
                                    " out of range for [0," + std::to_string(m_size) + ")");
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
    std::pair<std::size_t, T> best_pair(Compare cmp, const char* name) const {
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
    explicit FxArray(std::size_t n) : m_size(n), m_arr(FxArray_make_aligned<T>(n, kAlign)) {}

    // 3) Dedicated init_list ctor — for braced lists
    FxArray(std::initializer_list<T> init) : FxArray(init.size()) {
        std::copy(init.begin(), init.end(), aligned_data());
    }

    // 4) one ctor for std::vector
    FxArray(std::vector<T> const& v) : FxArray(v.size()) {
        std::copy(v.begin(), v.end(), aligned_data());
    }

    // 5) one overload for c style array
    template<std::size_t N>
    FxArray(T const (&arr)[N]) : FxArray(N) {
        std::copy_n(arr, N, aligned_data());
    }

    // deep‐copy copy‐ctor using copy_n
    FxArray(FxArray const& o) : FxArray(o.m_size) {
        std::copy_n(o.aligned_data(), m_size, aligned_data());
    }

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
    T* aligned_data() noexcept { return std::assume_aligned<kAlign>(m_arr.get()); }
    T const* aligned_data() const noexcept { return std::assume_aligned<kAlign>(m_arr.get()); }

    // 1) UNCHECKED, inlined, noexcept operator[] for hot loops
    T& operator[](size_t i) noexcept { return aligned_data()[i]; }
    T const& operator[](size_t i) const noexcept { return aligned_data()[i]; }

    template<std::integral I>
    T& operator()(I i) noexcept {
        return aligned_data()[static_cast<size_t>(i)];
    }

    template<std::integral I>
    T const& operator()(I i) const noexcept {
        return aligned_data()[static_cast<size_t>(i)];
    }

    // 2) BOUNDS-CHECKED at(), still accepts *any* integral index
    template<std::integral I>
    T& at(I idx) {
        return aligned_data()[checkIndex(idx)];
    }
    template<std::integral I>
    T const& at(I idx) const {
        return aligned_data()[checkIndex(idx)];
    }

    // Iterators use aligned_data (helps vectorizers in range-for)
    T* begin() noexcept { return aligned_data(); }
    T const* begin() const noexcept { return aligned_data(); }
    T* end() noexcept { return aligned_data() + m_size; }
    T const* end() const noexcept { return aligned_data() + m_size; }

    // size & raw data
    size_t size() const noexcept { return m_size; }
    bool empty() const noexcept { return m_size == 0; }
    T* data() noexcept { return m_arr.get(); }
    T const* data() const noexcept { return m_arr.get(); }

    void swap(FxArray& o) noexcept {
        std::swap(m_size, o.m_size);
        std::swap(m_arr, o.m_arr);
    }

    template<typename U>
        requires(Numeric<T> && Numeric<U>)
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
        return best_pair([&](const T& a, const T& b) { return cmp(b, a); }, "max").second;
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
        return best_pair([&](const T& a, const T& b) { return cmp(b, a); }, "argmax");
    }

    // default operator< versions
    std::pair<std::size_t, T> argmin() const { return argmin(std::less<T>{}); }
    std::pair<std::size_t, T> argmax() const { return argmax(std::less<T>{}); }

    // --- mean: requires T() + T+= U + T /= scalar ---
    T mean() const {
        throw_if_empty("mean");
        T const* __restrict p = aligned_data();
        T sum = p[0]; // initialize with first element
        for (std::size_t i = 1; i < m_size; ++i)
            sum += p[i];
        return sum / m_size;
    }

    // mean as float (casts each element to double for accuracy, returns float)
    float meanf() const
        requires Numeric<T>
    {
        throw_if_empty("meanf");
        T const* __restrict p = aligned_data();
        double sum = 0.0;
        for (std::size_t i = 0; i < m_size; ++i)
            sum += static_cast<double>(p[i]);
        return static_cast<float>(sum / m_size);
    }

    // population standard deviation as float
    float stddev() const
        requires Numeric<T>
    {
        throw_if_empty("stddev");
        double m = meanf();
        T const* __restrict p = aligned_data();
        double acc = 0.0;
        for (std::size_t i = 0; i < m_size; ++i) {
            double d = static_cast<double>(p[i]) - m;
            acc += d * d;
        }
        return static_cast<float>(std::sqrt(acc / m_size));
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
    template<typename U>
        requires ConvertibleOrNumeric<U, T>
    FxArray& operator+=(U const& v) {
        T* __restrict p = aligned_data();
        const T vv = static_cast<T>(v);
        for (std::size_t i = 0; i < m_size; ++i)
            p[i] += vv;
        return *this;
    }

    template<typename U>
        requires ConvertibleOrNumeric<U, T>
    FxArray& operator-=(U const& v) {
        T* __restrict p = aligned_data();
        const T vv = static_cast<T>(v);
        for (std::size_t i = 0; i < m_size; ++i)
            p[i] -= vv;
        return *this;
    }

    template<typename U>
        requires ConvertibleOrNumeric<U, T>
    FxArray& operator*=(U const& v) {
        T* __restrict p = aligned_data();
        const T vv = static_cast<T>(v);
        for (std::size_t i = 0; i < m_size; ++i)
            p[i] *= vv;
        return *this;
    }

    template<typename U>
        requires ConvertibleOrNumeric<U, T>
    FxArray& operator/=(U const& v) {
        T* __restrict p = aligned_data();
        const T vv = static_cast<T>(v);
        for (std::size_t i = 0; i < m_size; ++i)
            p[i] /= vv;
        return *this;
    }

    // --- in-place element‐wise with another FxArray ---
    template<typename U>
        requires ConvertibleOrNumeric<U, T>
    FxArray& operator+=(FxArray<U> const& o) {
        throw_if_size_mismatch("+=", o.size());
        T* __restrict p = aligned_data();
        U const* __restrict op = o.aligned_data();
        for (std::size_t i = 0; i < m_size; ++i)
            p[i] += op[i];
        return *this;
    }

    template<typename U>
        requires ConvertibleOrNumeric<U, T>
    FxArray& operator-=(FxArray<U> const& o) {
        throw_if_size_mismatch("-=", o.size());
        T* __restrict p = aligned_data();
        U const* __restrict op = o.aligned_data();
        for (std::size_t i = 0; i < m_size; ++i)
            p[i] -= op[i];
        return *this;
    }

    template<typename U>
        requires ConvertibleOrNumeric<U, T>
    FxArray& operator*=(FxArray<U> const& o) {
        throw_if_size_mismatch("*=", o.size());
        T* __restrict p = aligned_data();
        U const* __restrict op = o.aligned_data();
        for (std::size_t i = 0; i < m_size; ++i)
            p[i] *= op[i];
        return *this;
    }

    template<typename U>
        requires ConvertibleOrNumeric<U, T>
    FxArray& operator/=(FxArray<U> const& o) {
        throw_if_size_mismatch("/=", o.size());
        T* __restrict p = aligned_data();
        U const* __restrict op = o.aligned_data();
        for (std::size_t i = 0; i < m_size; ++i)
            p[i] /= op[i];
        return *this;
    }

    // dot with one FxVec2f → returns FxArray<float>
    FxArray<float> dot(T const& v) const
        requires FxVecT<T>
    {
        FxArray<float> out(this->size());
        T const* __restrict p = aligned_data();
        float* __restrict result = out.aligned_data();
        for (size_t i = 0; i < this->size(); ++i)
            result[i] = p[i].dot(v);
        return out;
    }

    // element-wise dot with another FxVec2fArray
    FxArray<float> dot(FxArray<T> const& o) const
        requires FxVecT<T>
    {
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
    FxArray& rotate_inplace_rad(float theta_rad) noexcept
        requires std::same_as<T, FxVec2f>
    {
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
    FxArray& rotate_inplace(float degrees) noexcept
        requires std::same_as<T, FxVec2f>
    {
        constexpr float FX_DEG2RAD = FxPif / 180.0f;
        return rotate_inplace_rad(degrees * FX_DEG2RAD);
    }

    // 3) In-place rotate by degrees
    FxArray& perp_inplace() noexcept
        requires std::same_as<T, FxVec2f>
    {
        T* __restrict p = aligned_data();
        for (std::size_t i = 0; i < m_size; ++i)
            p[i] = p[i].perp();
        return *this;
    }

    // 4) Non-mutating perp → new array
    FxArray perp() const
        requires std::same_as<T, FxVec2f>
    {
        FxArray tmp = *this;
        tmp.perp_inplace();
        return tmp;
    }

    // 5) Non-mutating rotate by radians → new array
    FxArray rotate_rad(float theta_rad) const
        requires std::same_as<T, FxVec2f>
    {
        FxArray tmp = *this;
        tmp.rotate_inplace_rad(theta_rad);
        return tmp;
    }

    // 6) Non-mutating rotate by degrees → new array
    FxArray rotate(float degrees) const
        requires std::same_as<T, FxVec2f>
    {
        FxArray tmp = *this;
        tmp.rotate_inplace(degrees);
        return tmp;
    }

    FxArray<float> bounds() const
        requires std::same_as<T, FxVec2f>
    {
        this->throw_if_empty("extrema");
        T const* __restrict p = aligned_data();
        const auto& v0 = p[0];
        float minx = v0.x(), maxx = v0.x();
        float miny = v0.y(), maxy = v0.y();
        for (size_t i = 1, n = this->size(); i < n; ++i) {
            const auto& v = p[i];
            float x = v.x(), y = v.y();
            if (x < minx) minx = x;
            else if (x > maxx) maxx = x;
            if (y < miny) miny = y;
            else if (y > maxy) maxy = y;
        }
        return {minx, miny, maxx, maxy};
    }
};

template<NumericOrFxVec T>
void swap(FxArray<T>& a, FxArray<T>& b) noexcept {
    a.swap(b);
}

// ────────────────────────────────────────────────────────────────────────────
//  FxVec2fArray: fixed‐size array of FxVec2f
// ────────────────────────────────────────────────────────────────────────────
using FxVec2fArray = FxArray<FxVec2f>;

// Scalar–Array
template<typename T, typename U>
    requires ConvertibleOrNumeric<U, T>
inline FxArray<T> operator+(FxArray<T> a, U const& scalar) {
    return a += scalar;
}

template<typename T, typename U>
    requires ConvertibleOrNumeric<U, T>
inline FxArray<T> operator-(FxArray<T> a, U const& scalar) {
    return a -= scalar;
}

template<typename T, typename U>
    requires ConvertibleOrNumeric<U, T>
inline FxArray<T> operator*(FxArray<T> a, U const& scalar) {
    return a *= scalar;
}

template<typename T, typename U>
    requires ConvertibleOrNumeric<U, T>
inline FxArray<T> operator/(FxArray<T> a, U const& scalar) {
    return a /= scalar;
}

// Scalar on the left
template<typename T, typename U>
    requires ConvertibleOrNumeric<U, T>
inline FxArray<T> operator+(U const& scalar, FxArray<T> a) {
    return a += scalar;
}

template<typename T, typename U>
    requires ConvertibleOrNumeric<U, T>
inline FxArray<T> operator*(U const& scalar, FxArray<T> a) {
    return a *= scalar;
}

template<typename T, typename U>
    requires ConvertibleOrNumeric<U, T>
inline FxArray<T> operator-(U const& scalar, FxArray<T> a) {
    return -a + scalar;
}

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
inline FxArray<T> operator+(FxArray<T> a, const FxArray<T>& b) {
    return a += b;
}

template<typename T>
inline FxArray<T> operator-(FxArray<T> a, const FxArray<T>& b) {
    return a -= b;
}

template<typename T>
inline FxArray<T> operator*(FxArray<T> a, const FxArray<T>& b) {
    return a *= b;
}

template<typename T>
inline FxArray<T> operator/(FxArray<T> a, const FxArray<T>& b) {
    return a /= b;
}

template<Numeric T>
inline std::ostream& operator<<(std::ostream& os, FxArray<T> const& a) {
    os << "FxArray { ";
    for (size_t i = 0; i < a.size(); ++i)
        os << a[i] << ", ";
    os << "}";
    return os;
}

inline std::ostream& operator<<(std::ostream& os, FxVec2fArray const& a) {
    os << "FxVec2fArray { ";
    for (size_t i = 0; i < a.size(); ++i)
        os << "(" << a[i].x() << " " << a[i].y() << "), ";
    os << "} ";
    return os;
}

inline std::ostream& operator<<(std::ostream& os, FxVec2f const& a) {
    os << "FxVec2f { ";
    os << a.x() << " " << a.y();
    os << " }";
    return os;
}

inline std::ostream& operator<<(std::ostream& os, FxVec3f const& a) {
    os << "FxVec3f { ";
    os << a.x() << " " << a.y() << " " << a.z();
    os << " }";
    return os;
}
