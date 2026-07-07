#pragma once

#include <optional>
#include <vector>

#include "autonomy_planning/grid_index.hpp"
#include "autonomy_planning/grid_map.hpp"
#include "autonomy_planning/planner_result.hpp"

namespace autonomy_planning
{

class AstarPlanner
{
   public:
    [[nodiscard]] std::optional<PlannerResult> plan(
        const GridMap& map, const GridIndex& start,
        const GridIndex& goal) const;

   private:
    int flat_index(const GridMap& map, const GridIndex& index) const noexcept;

    double heuristic_cost(const GridMap& map, const GridIndex& source,
                          const GridIndex& target) const noexcept;

    double movement_cost(const GridMap& map) const noexcept;

    std::optional<autonomy_core::Path> reconstruct_world_path(
        const GridMap& map,
        const std::vector<std::optional<GridIndex>>& came_from,
        const GridIndex& start, const GridIndex& goal) const;

    std::optional<std::vector<GridIndex>> reconstruct_grid_path(
        const GridMap& map,
        const std::vector<std::optional<GridIndex>>& came_from,
        const GridIndex& start, const GridIndex& goal) const;
};

}  // namespace autonomy_planning
