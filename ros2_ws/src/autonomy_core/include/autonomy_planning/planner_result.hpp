#pragma once

#include <vector>

#include "autonomy_core/path.hpp"
#include "autonomy_planning/grid_index.hpp"

namespace autonomy_planning
{

struct PlannerResult
{
    autonomy_core::Path world_path{};
    std::vector<GridIndex> grid_path{};
    double cost_m{0.0};
    int expanded_nodes{0};
};

}  // namespace autonomy_planning
