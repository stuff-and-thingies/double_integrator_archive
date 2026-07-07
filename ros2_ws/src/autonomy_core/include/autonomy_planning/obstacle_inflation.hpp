#pragma once

#include "autonomy_planning/grid_map.hpp"

namespace autonomy_planning
{

[[nodiscard]] GridMap inflate_obstacles(const GridMap& input_map,
                                        double inflation_radius_m);

}
