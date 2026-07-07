#pragma once

#include "autonomy_core/vector3.hpp"

namespace autonomy_core
{

struct Pose3
{
    Vector3 position{};
    Vector3 orientation_rpy_rad{};

    constexpr Pose3() = default;

    constexpr Pose3(const Vector3& pos_in,
                    const Vector3& euler_ang_rad_in) noexcept
        : position{pos_in}, orientation_rpy_rad(euler_ang_rad_in)
    {
    }

    constexpr bool operator==(const Pose3& other) const noexcept
    {
        return position == other.position &&
               orientation_rpy_rad == other.orientation_rpy_rad;
    }
};

}  // namespace autonomy_core
