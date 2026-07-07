#pragma once

#include "autonomy_core/path.hpp"
#include "autonomy_core/trajectory.hpp"

namespace autonomy_trajectory
{

[[nodiscard]] autonomy_core::Trajectory path_to_constant_speed_trajectory(
    const autonomy_core::Path& path, double speed_mps);

}  // namespace autonomy_trajectory
