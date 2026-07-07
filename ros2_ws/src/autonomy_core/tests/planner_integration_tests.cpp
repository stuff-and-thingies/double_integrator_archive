#include <cassert>

#include "autonomy_planning/astar_planner.hpp"
#include "autonomy_planning/dijkstra_planner.hpp"
#include "autonomy_planning/grid_index.hpp"
#include "autonomy_planning/grid_map.hpp"
#include "autonomy_planning/line_of_sight.hpp"
#include "autonomy_planning/obstacle_inflation.hpp"
#include "autonomy_planning/path_smoother.hpp"
#include "test_utils.hpp"

using autonomy_planning::AstarPlanner;
using autonomy_planning::DijkstraPlanner;
using autonomy_planning::GridIndex;
using autonomy_planning::GridMap;
using autonomy_planning::has_line_of_sight;
using autonomy_planning::inflate_obstacles;
using autonomy_planning::PlannerResult;
using autonomy_planning::smooth_grid_path_line_of_sight;

void test_astar_with_smoothing()
{
    GridMap map{3, 3, 1.0};

    const GridIndex start{0, 0};
    const GridIndex goal{2, 2};

    map.set_occupied(GridIndex{1, 0}, true);
    map.set_occupied(GridIndex{1, 1}, true);

    // Raw map:
    //
    // S . .
    // X X .
    // . . G
    //
    const AstarPlanner planner;
    const std::optional<PlannerResult> raw_result =
        planner.plan(map, start, goal);

    assert(raw_result.has_value());
    assert(raw_result.value().grid_path.front() == start);
    assert(raw_result.value().grid_path.back() == goal);
    assert(raw_result.value().grid_path.size() > 2);

    const std::vector<GridIndex> smoothed_grid_path =
        smooth_grid_path_line_of_sight(map, raw_result.value().grid_path);

    assert(smoothed_grid_path.size() < raw_result.value().grid_path.size());
    assert(smoothed_grid_path.front() == start);
    assert(smoothed_grid_path.back() == goal);

    for (size_t curr_index = 0; curr_index < smoothed_grid_path.size() - 1;
         ++curr_index)
    {
        assert(has_line_of_sight(map, smoothed_grid_path.at(curr_index),
                                 smoothed_grid_path.at(curr_index + 1)));
    }
}

void test_inflated_obstacles_can_close_narrow_gap()
{
    GridMap raw_map{5, 5, 1.0};

    // Raw map:
    //
    // . . . . .
    // . . X . .
    // S . . . G
    // . . X . .
    // . . . . .
    //
    // Start and goal are connected through the center gap at {2, 2}.

    const GridIndex start{2, 0};
    const GridIndex goal{2, 4};

    raw_map.set_occupied(GridIndex{1, 2}, true);
    raw_map.set_occupied(GridIndex{3, 2}, true);

    const AstarPlanner planner;

    const auto raw_result = planner.plan(raw_map, start, goal);

    assert(raw_result.has_value());
    assert(!raw_map.is_occupied(GridIndex{2, 2}));

    const GridMap inflated_map = inflate_obstacles(raw_map, 1.0);

    // Inflated map should close the center gap:
    //
    // . . X . .
    // . X X X .
    // S . X . G
    // . X X X .
    // . . X . .
    //
    // The inflated obstacles create a full vertical wall in column 2.

    assert(inflated_map.is_occupied(GridIndex{0, 2}));
    assert(inflated_map.is_occupied(GridIndex{1, 2}));
    assert(inflated_map.is_occupied(GridIndex{2, 2}));
    assert(inflated_map.is_occupied(GridIndex{3, 2}));
    assert(inflated_map.is_occupied(GridIndex{4, 2}));

    const auto inflated_result = planner.plan(inflated_map, start, goal);

    assert(!inflated_result.has_value());
}

void test_visualization_map_astar_matches_dijkstra()
{
    GridMap raw_map{10, 10, 1.0};

    raw_map.set_occupied(GridIndex{5, 5}, true);
    raw_map.set_occupied(GridIndex{3, 3}, true);
    raw_map.set_occupied(GridIndex{0, 4}, true);

    const GridMap inflated_map = inflate_obstacles(raw_map, 1.0);

    const GridIndex start{0, 0};
    const GridIndex goal{9, 9};

    const AstarPlanner astar;
    const DijkstraPlanner dijkstra;

    const auto astar_result = astar.plan(inflated_map, start, goal);
    const auto dijkstra_result = dijkstra.plan(inflated_map, start, goal);

    assert(astar_result.has_value());
    assert(dijkstra_result.has_value());

    assert(nearly_equal(astar_result->cost_m, dijkstra_result->cost_m));

    assert(astar_result->grid_path.front() == start);
    assert(astar_result->grid_path.back() == goal);

    for (const GridIndex& cell : astar_result->grid_path)
    {
        assert(inflated_map.in_bounds(cell));
        assert(!inflated_map.is_occupied(cell));
    }
}

int main()
{
    test_inflated_obstacles_can_close_narrow_gap();
    test_astar_with_smoothing();
    test_visualization_map_astar_matches_dijkstra();

    return 0;
}
