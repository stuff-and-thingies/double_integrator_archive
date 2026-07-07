#pragma once

#include <autonomy_planning/grid_index.hpp>
#include <autonomy_planning/grid_map.hpp>
#include <vector>

namespace autonomy_planning
{
std::vector<GridIndex> free_neighbors_4(const GridMap& map,
                                        const GridIndex& index);
}
