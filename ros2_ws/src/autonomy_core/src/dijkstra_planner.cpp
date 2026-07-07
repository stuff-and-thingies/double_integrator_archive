#include "autonomy_planning/dijkstra_planner.hpp"

#include <algorithm>
#include <limits>
#include <queue>

#include "autonomy_planning/grid_neighbors.hpp"

namespace autonomy_planning
{
struct QueueItem
{
    GridIndex index{};
    double queue_cost{0.0};
};

struct CompareQueueItem
{
    bool operator()(const QueueItem& a, const QueueItem& b) const
    {
        // meaning: if a has higher cost, then "yes," it should be placed
        // further down the queue
        return a.queue_cost > b.queue_cost;
    }
};

std::optional<PlannerResult> DijkstraPlanner::plan(const GridMap& map,
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

    // cost_so_far means accumulated cost from the start of the algorithm to now
    // can also be thought of as cost_to_here in the geometric sense
    std::vector<double> cost_so_far(map.width() * map.height(),
                                    std::numeric_limits<double>::infinity());

    std::vector<std::optional<GridIndex>> came_from(map.width() * map.height(),
                                                    std::nullopt);

    cost_so_far.at(flat_index(map, start)) = 0.0;
    frontier.push(QueueItem{GridIndex{start}, 0.0});

    while (!frontier.empty())
    {
        const QueueItem current_item = frontier.top();
        frontier.pop();

        const GridIndex current_ind = current_item.index;
        const int flat_current_ind = flat_index(map, current_ind);

        if (current_item.queue_cost > cost_so_far.at(flat_current_ind))
        {
            // this says: have i already popped this cell and fully explored it
            // via a cheaper route? if yes, skip
            continue;
        }

        if (current_ind == goal)
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
            result.cost_m = cost_so_far.at(flat_index(map, goal));

            return result;
        }

        result.expanded_nodes += 1;

        for (GridIndex neighbor : free_neighbors_4(map, current_ind))
        {
            const int flat_neighbor_ind = flat_index(map, neighbor);

            const double new_cost =
                cost_so_far.at(flat_current_ind) + movement_cost(map);

            if (new_cost < cost_so_far.at(flat_neighbor_ind))
            {
                cost_so_far.at(flat_neighbor_ind) = new_cost;
                came_from.at(flat_neighbor_ind) = current_ind;
                frontier.push(QueueItem{GridIndex{neighbor}, new_cost});
            }
        }
    }

    return std::nullopt;
}

int DijkstraPlanner::flat_index(const GridMap& map,
                                const GridIndex& index) const noexcept
{
    return map.width() * index.row + index.col;
}

double DijkstraPlanner::movement_cost(const GridMap& map) const noexcept
{
    return map.resolution_m();
}

std::optional<autonomy_core::Path> DijkstraPlanner::reconstruct_world_path(
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

        const std::optional<GridIndex>& parent =
            came_from.at(flat_index(map, curr));

        if (!parent.has_value())
        {
            return std::nullopt;
        }
        curr = parent.value();
    }

    autonomy_core::Vector2 start_world_pos = map.grid_to_world(start);
    autonomy_core::Pose3 start_pose;

    start_pose.position.x = start_world_pos.x;
    start_pose.position.y = start_world_pos.y;
    start_pose.position.z = 0.0;
    path_poses.emplace_back(start_pose);

    std::reverse(path_poses.begin(), path_poses.end());

    path.poses = path_poses;

    return path;
}

std::optional<std::vector<GridIndex>> DijkstraPlanner::reconstruct_grid_path(
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
