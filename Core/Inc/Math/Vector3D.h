//
// Created by dyrel on 2/26/2026.
//

/**
 * @file Vector3D.h
 * @ingroup platform
 * @brief Header-only 3D vector template used for sensor readings.
 */

#ifndef KINGFISHER_SW_Vector3D_H
#define KINGFISHER_SW_Vector3D_H

#include <cmath>
#include <type_traits>
#include <limits>
#include <stdexcept>

/** @brief a 3D vector class. Vector3D<instantiation type> variable_name(x,y,z);
 *
 * Mixed-type arithmetic promotes to the common type, so a float vector plus a
 * double vector gives a double vector.
 *
 * @tparam T instantiation type. float creates a float vector,
 *          int produces an int vector, etc.
 *
 * @warning The firmware builds with -fno-exceptions, so operator/() and
 *          normalized(), which throw, would fail to compile if used.
 */
template<typename T>
class Vector3D {
    static_assert(std::is_arithmetic_v<T>,
                  "Vector3D requires arithmetic type");

public:
    T x;    ///< X component.
    T y;    ///< Y component.
    T z;    ///< Z component.

    using value_type = T;   ///< Component type.

    // =============================
    // Constructors
    // =============================

    /// @brief Zero vector.
    constexpr Vector3D() : x(0), y(0), z(0) {}
    /// @brief Vector from components.
    constexpr Vector3D(T X, T Y, T Z) : x(X), y(Y), z(Z) {}

    /// @brief Explicit conversion from a vector of another type.
    template<typename U>
    constexpr explicit Vector3D(const Vector3D<U>& v)
        : x(static_cast<T>(v.x)),
          y(static_cast<T>(v.y)),
          z(static_cast<T>(v.z)) {}

    // =============================
    // Type Promotion Helper
    // =============================

    /// Vector of the common type of T and U.
    template<typename U>
    using Promote = Vector3D<std::common_type_t<T, U>>;

    // =============================
    // Arithmetic (Mixed Type Safe)
    // =============================

    /// @brief Component-wise sum.
    template<typename U>
    constexpr Promote<U> operator+(const Vector3D<U>& v) const {
        using R = std::common_type_t<T,U>;
        return Promote<U>(
            static_cast<R>(x) + v.x,
            static_cast<R>(y) + v.y,
            static_cast<R>(z) + v.z
        );
    }

    /// @brief Component-wise difference.
    template<typename U>
    constexpr Promote<U> operator-(const Vector3D<U>& v) const {
        using R = std::common_type_t<T,U>;
        return Promote<U>(
            static_cast<R>(x) - v.x,
            static_cast<R>(y) - v.y,
            static_cast<R>(z) - v.z
        );
    }

    /// @brief Vector times scalar.
    template<typename U>
    constexpr Promote<U> operator*(U scalar) const {
        using R = std::common_type_t<T,U>;
        return Promote<U>(
            static_cast<R>(x) * scalar,
            static_cast<R>(y) * scalar,
            static_cast<R>(z) * scalar
        );
    }

    /// @brief Vector divided by scalar. Throws on division by zero (see the class warning).
    template<typename U>
    constexpr Promote<U> operator/(U scalar) const {
        if (scalar == 0)
            throw std::runtime_error("Division by zero");
        using R = std::common_type_t<T,U>;
        return Promote<U>(
            static_cast<R>(x) / scalar,
            static_cast<R>(y) / scalar,
            static_cast<R>(z) / scalar
        );
    }

    // =============================
    // Dot & Cross
    // =============================

    /// @brief Dot product.
    template<typename U>
    constexpr auto dot(const Vector3D<U>& v) const {
        using R = std::common_type_t<T,U>;
        return static_cast<R>(x)*v.x +
               static_cast<R>(y)*v.y +
               static_cast<R>(z)*v.z;
    }

    /// @brief Cross product, this × @p v.
    template<typename U>
    constexpr Promote<U> cross(const Vector3D<U>& v) const {
        using R = std::common_type_t<T,U>;
        return Promote<U>(
            static_cast<R>(y)*v.z - static_cast<R>(z)*v.y,
            static_cast<R>(z)*v.x - static_cast<R>(x)*v.z,
            static_cast<R>(x)*v.y - static_cast<R>(y)*v.x
        );
    }

    // =============================
    // Norms
    // =============================

    /// @brief Euclidean length. Returns T for floating-point vectors, double otherwise.
    auto magnitude() const {
        using R = std::conditional_t<
            std::is_floating_point<T>::value,
            T,
            double>;

        R xx = static_cast<R>(x);
        R yy = static_cast<R>(y);
        R zz = static_cast<R>(z);
        auto r = std::sqrt(xx*xx + yy*yy + zz*zz);
        return r;
    }

    /// @brief Squared length (no square root).
    constexpr auto magnitudeSquared() const {
        using R = std::common_type_t<T,T>;
        return static_cast<R>(x)*x +
               static_cast<R>(y)*y +
               static_cast<R>(z)*z;
    }

    /// @brief Unit vector in the same direction. Throws for a zero vector (see the class warning).
    auto normalized() const {
        auto mag = magnitude();
        if (mag == 0)
            throw std::runtime_error("Cannot normalize zero vector");

        using R = decltype(mag);
        return Vector3D<R>(
            static_cast<R>(x) / mag,
            static_cast<R>(y) / mag,
            static_cast<R>(z) / mag
        );
    }

    // =============================
    // Distance
    // =============================

    /// @brief Euclidean distance to @p v.
    template<typename U>
    auto distance(const Vector3D<U>& v) const {
        return (*this - v).magnitude();
    }

    // =============================
    // Safe Float Comparison
    // =============================

    /**
     * @brief Component-wise comparison within @p eps for floating types; exact for integers.
     * @param v Vector to compare with.
     * @param eps Tolerance per component. Defaults to 100 machine epsilons.
     */
    bool isApprox(const Vector3D& v,
                  T eps = std::numeric_limits<T>::epsilon()*100) const {
        if constexpr (std::is_floating_point<T>::value) {
            return std::fabs(x - v.x) < eps &&
                   std::fabs(y - v.y) < eps &&
                   std::fabs(z - v.z) < eps;
        } else {
            return *this == v;
        }
    }

    /// @brief Exact component-wise equality.
    constexpr bool operator==(const Vector3D& v) const {
        return x == v.x && y == v.y && z == v.z;
    }

    /// @brief Exact component-wise inequality.
    constexpr bool operator!=(const Vector3D& v) const {
        return !(*this == v);
    }
};

/// @brief Scalar times vector (scalar on the left).
template<typename T, typename U>
constexpr auto operator*(T scalar, const Vector3D<U>& v) {
    return v * scalar;
}

#endif //KINGFISHER_SW_Vector3D_H
