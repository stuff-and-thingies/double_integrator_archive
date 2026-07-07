#pragma once
#include <vector>

#include "autonomy_core/path.hpp"
#include "autonomy_planning/grid_index.hpp"
#include "autonomy_planning/grid_map.hpp"

namespace autonomy_planning
{

[[nodiscard]] autonomy_core::Path grid_path_to_world_path(
    const GridMap& map, const std::vector<GridIndex>& grid_path);

}  // namespace autonomy_planning
