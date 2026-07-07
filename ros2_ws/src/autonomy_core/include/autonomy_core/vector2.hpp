#pragma once

#include <cmath>

namespace autonomy_core
{

struct Vector2
{
    double x{0.0};
    double y{0.0};

    constexpr Vector2() = default;

    constexpr Vector2(double x_in, double y_in) noexcept : x{x_in}, y{y_in} {}

    [[nodiscard]] constexpr Vector2 operator+(
        const Vector2& other) const noexcept
    {
        return Vector2{x + other.x, y + other.y};
    }

    [[nodiscard]] constexpr Vector2 operator-(
        const Vector2& other) const noexcept
    {
        return Vector2{x - other.x, y - other.y};
    }

    [[nodiscard]] constexpr Vector2 operator*(double scalar) const noexcept
    {
        return Vector2{x * scalar, y * scalar};
    }

    Vector2& operator+=(const Vector2& other) noexcept
    {
        x += other.x;
        y += other.y;

        return *this;
    }

    Vector2& operator-=(const Vector2& other) noexcept
    {
        x -= other.x;
        y -= other.y;

        return *this;
    }

    Vector2& operator*=(double scalar) noexcept
    {
        x *= scalar;
        y *= scalar;

        return *this;
    }

    [[nodiscard]] constexpr double squared_norm() const noexcept
    {
        return x * x + y * y;
    }

    [[nodiscard]] double norm() const noexcept { return std::hypot(x, y); }
};

[[nodiscard]] constexpr Vector2 operator*(double scalar,
                                          const Vector2& v) noexcept
{
    return v * scalar;
}

[[nodiscard]] constexpr double dot(const Vector2& a, const Vector2& b) noexcept
{
    return a.x * b.x + a.y * b.y;
}

}  // namespace autonomy_core
