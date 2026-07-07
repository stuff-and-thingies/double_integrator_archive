#pragma once
#include <vector>

#include "autonomy_planning/grid_index.hpp"
#include "autonomy_planning/grid_map.hpp"

namespace autonomy_planning
{
// Assumes raw_path is an ordered, collision-free grid path produced by a
// planner. The smoother removes intermediate points when line of sight exists
// between farther-apart cells.
[[nodiscard]] std::vector<GridIndex> smooth_grid_path_line_of_sight(
    const GridMap& map, const std::vector<GridIndex>& raw_path);

}  // namespace autonomy_planning
