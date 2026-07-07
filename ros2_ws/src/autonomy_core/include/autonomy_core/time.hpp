#pragma once

namespace autonomy_core
{

struct Duration
{
    double seconds{0.0};

    constexpr Duration() = default;

    constexpr Duration(double seconds_in) noexcept : seconds{seconds_in} {}

    Duration& operator+=(const Duration& other) noexcept
    {
        seconds += other.seconds;

        return *this;
    }

    Duration& operator*=(double scalar) noexcept
    {
        seconds *= scalar;

        return *this;
    }
};

struct Timestamp
{
    double seconds{0.0};

    constexpr Timestamp() = default;

    constexpr Timestamp(double seconds_in) noexcept : seconds{seconds_in} {}

    [[nodiscard]] Duration operator-(const Timestamp& other) const noexcept
    {
        return Duration{seconds - other.seconds};
    }

    [[nodiscard]] Timestamp operator+(const Duration& duration) const noexcept
    {
        return Timestamp{seconds + duration.seconds};
    }
};

}  // namespace autonomy_core
