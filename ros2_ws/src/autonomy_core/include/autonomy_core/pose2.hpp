#pragma once

#include "autonomy_core/vector2.hpp"

namespace autonomy_core
{

struct Pose2
{
    Vector2 position{};
    double yaw_rad{0.0};
};

}  // namespace autonomy_core
