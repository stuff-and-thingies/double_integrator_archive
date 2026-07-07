#pragma once

#include <cmath>

namespace autonomy_core
{
struct Vector3
{
    double x{0.0};
    double y{0.0};
    double z{0.0};

    constexpr Vector3() = default;

    constexpr Vector3(double x_in, double y_in, double z_in) noexcept
        : x{x_in}, y{y_in}, z{z_in}
    {
    }

    [[nodiscard]] constexpr Vector3 operator+(
        const Vector3& other) const noexcept
    {
        return Vector3{x + other.x, y + other.y, z + other.z};
    }

    [[nodiscard]] constexpr Vector3 operator-(
        const Vector3& other) const noexcept
    {
        return Vector3{x - other.x, y - other.y, z - other.z};
    }

    [[nodiscard]] constexpr Vector3 operator*(double scalar) const noexcept
    {
        return Vector3{x * scalar, y * scalar, z * scalar};
    }

    [[nodiscard]] constexpr Vector3 operator/(double scalar) const noexcept
    {
        return Vector3{x / scalar, y / scalar, z / scalar};
    }

    Vector3& operator+=(const Vector3& other) noexcept
    {
        x += other.x;
        y += other.y;
        z += other.z;

        return *this;
    }

    Vector3& operator-=(const Vector3& other) noexcept
    {
        x -= other.x;
        y -= other.y;
        z -= other.z;

        return *this;
    }

    Vector3& operator*=(double scalar) noexcept
    {
        x *= scalar;
        y *= scalar;
        z *= scalar;

        return *this;
    }

    [[nodiscard]] constexpr bool operator==(const Vector3& other) const
    {
        return x == other.x && y == other.y && z == other.z;
    }

    [[nodiscard]] constexpr double squared_norm() const noexcept
    {
        return x * x + y * y + z * z;
    }

    [[nodiscard]] double norm() const noexcept { return std::hypot(x, y, z); }
};

[[nodiscard]] constexpr Vector3 operator*(double scalar,
                                          const Vector3& v) noexcept
{
    return v * scalar;
}

[[nodiscard]] constexpr double dot(const Vector3& a, const Vector3& b)
{
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

[[nodiscard]] constexpr double euclidean_distance(const Vector3& a,
                                                  const Vector3& b)
{
    return std::sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y) +
                     (a.z - b.z) * (a.z - b.z));
}
}  // namespace autonomy_core
