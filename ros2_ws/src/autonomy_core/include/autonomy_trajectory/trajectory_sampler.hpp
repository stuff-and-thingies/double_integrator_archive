#pragma once

#include <optional>

#include "autonomy_core/time.hpp"
#include "autonomy_core/trajectory.hpp"

namespace autonomy_trajectory
{

// Assumes trajectory.points are stored by increasing time.
// For interpolation, adjacent points should have distinct timestamps
[[nodiscard]] std::optional<autonomy_core::TrajectoryPoint> sample_trajectory(
    const autonomy_core::Trajectory& trajectory,
    const autonomy_core::Timestamp& query_time);

}  // namespace autonomy_trajectory
