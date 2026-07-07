#include "autonomy_planning/astar_planner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>

#include "autonomy_planning/grid_neighbors.hpp"

namespace autonomy_planning
{

struct QueueItem
{
    GridIndex index{};
    double map_cost{};
    double queue_cost{};
};

struct CompareQueueItem
{
    bool operator()(const QueueItem& a, const QueueItem& b) const
    {
        return a.queue_cost > b.queue_cost;
    }
};

std::optional<PlannerResult> AstarPlanner::plan(const GridMap& map,
                                                const GridIndex& start,
                                                const GridIndex& goal) const
{
    PlannerResult result;

    if (!map.in_bounds(start) || !map.in_bounds(goal))
    {
        return std::nullopt;
    }

    if (map.is_occupied(start) || map.is_occupied(goal))
    {
        return std::nullopt;
    }

    if (start == goal)
    {
        const autonomy_core::Vector2 world_pos = map.grid_to_world(start);

        autonomy_core::Pose3 pose;
        pose.position.x = world_pos.x;
        pose.position.y = world_pos.y;
        pose.position.z = 0.0;

        result.world_path.poses.emplace_back(pose);
        result.grid_path.emplace_back(start);

        result.cost_m = 0.0;

        return result;
    }

    std::priority_queue<QueueItem, std::vector<QueueItem>, CompareQueueItem>
        frontier;

    std::vector<double> cost_so_far(map.width() * map.height(),
                                    std::numeric_limits<double>::infinity());

    std::vector<std::optional<GridIndex>> came_from(map.width() * map.height(),
                                                    std::nullopt);

    cost_so_far.at(flat_index(map, start)) = 0.0;

    frontier.push(QueueItem{start, 0.0, heuristic_cost(map, start, goal)});

    while (!frontier.empty())
    {
        const QueueItem current_queue_item = frontier.top();
        frontier.pop();

        const GridIndex& current_index = current_queue_item.index;
        const int flat_current_index = flat_index(map, current_index);

        if (current_queue_item.map_cost > cost_so_far.at(flat_current_index))
        {
            continue;
        }

        if (current_index == goal)
        {
            std::optional<autonomy_core::Path> world_path =
                reconstruct_world_path(map, came_from, start, goal);

            std::optional<std::vector<GridIndex>> grid_path =
                reconstruct_grid_path(map, came_from, start, goal);

            if (!world_path.has_value())
            {
                return std::nullopt;
            }

            if (!grid_path.has_value())
            {
                return std::nullopt;
            }

            result.world_path = world_path.value();
            result.grid_path = grid_path.value();
            result.cost_m = cost_so_far.at(flat_current_index);

            return result;
        }

        result.expanded_nodes += 1;

        for (const GridIndex& neighbor : free_neighbors_4(map, current_index))
        {
            // find cost from start to this particular neighbor point
            const double cost_to_current_index =
                cost_so_far.at(flat_index(map, current_index));

            // add movement cost to get the new cost to go to this neighbor
            const double new_cost_to_neighbor =
                cost_to_current_index + movement_cost(map);

            const double old_cost_to_neighbor =
                cost_so_far.at(flat_index(map, neighbor));

            if (new_cost_to_neighbor < old_cost_to_neighbor)
            {
                cost_so_far.at(flat_index(map, neighbor)) =
                    new_cost_to_neighbor;

                const double priority_queue_cost =
                    new_cost_to_neighbor + heuristic_cost(map, neighbor, goal);

                frontier.push(QueueItem{neighbor, new_cost_to_neighbor,
                                        priority_queue_cost});
                came_from.at(flat_index(map, neighbor)) = current_index;
            }
        }
    }

    return std::nullopt;
}

int AstarPlanner::flat_index(const GridMap& map,
                             const GridIndex& index) const noexcept
{
    return map.width() * index.row + index.col;
}

double AstarPlanner::movement_cost(const GridMap& map) const noexcept
{
    return map.resolution_m();
}

double AstarPlanner::heuristic_cost(const GridMap& map, const GridIndex& source,
                                    const GridIndex& target) const noexcept
{
    return map.resolution_m() * (std::abs(source.row - target.row) +
                                 std::abs(source.col - target.col));
}

std::optional<autonomy_core::Path> AstarPlanner::reconstruct_world_path(
    const GridMap& map, const std::vector<std::optional<GridIndex>>& came_from,
    const GridIndex& start, const GridIndex& goal) const
{
    autonomy_core::Path path;

    std::vector<autonomy_core::Pose3> path_poses;

    GridIndex curr = goal;

    while (curr != start)
    {
        autonomy_core::Vector2 world_pos = map.grid_to_world(curr);

        autonomy_core::Pose3 pose;

        pose.position.x = world_pos.x;
        pose.position.y = world_pos.y;
        pose.position.z = 0.0;

        path_poses.emplace_back(pose);

        const std::optional<GridIndex> parent =
            came_from.at(flat_index(map, curr));

        if (!parent.has_value())
        {
            return std::nullopt;
        }

        curr = parent.value();
    }

    autonomy_core::Vector2 start_position = map.grid_to_world(start);
    autonomy_core::Pose3 start_pose;

    start_pose.position.x = start_position.x;
    start_pose.position.y = start_position.y;
    start_pose.position.z = 0.0;

    path_poses.emplace_back(start_pose);

    std::reverse(path_poses.begin(), path_poses.end());

    path.poses = path_poses;

    return path;
}

std::optional<std::vector<GridIndex>> AstarPlanner::reconstruct_grid_path(
    const GridMap& map, const std::vector<std::optional<GridIndex>>& came_from,
    const GridIndex& start, const GridIndex& goal) const
{
    std::vector<GridIndex> grid_path;

    GridIndex curr = goal;

    while (curr != start)
    {
        grid_path.emplace_back(curr.row, curr.col);

        const std::optional<GridIndex> parent =
            came_from.at(flat_index(map, curr));

        if (!parent.has_value())
        {
            return std::nullopt;
        }

        curr = parent.value();
    }

    grid_path.emplace_back(start.row, start.col);

    std::reverse(grid_path.begin(), grid_path.end());

    return grid_path;
}

}  // namespace autonomy_planning
