#pragma once

#include "autonomy_planning/grid_index.hpp"
#include "autonomy_planning/grid_map.hpp"

namespace autonomy_planning
{

[[nodiscard]] bool has_line_of_sight(const GridMap& map, const GridIndex& start,
                                     const GridIndex& goal);

}  // namespace autonomy_planning
