#include <cassert>

#include "autonomy_planning/astar_planner.hpp"
#include "autonomy_planning/grid_index.hpp"
#include "autonomy_planning/grid_map.hpp"
#include "autonomy_planning/line_of_sight.hpp"
#include "autonomy_planning/obstacle_inflation.hpp"
#include "autonomy_planning/path_conversion.hpp"
#include "autonomy_planning/path_smoother.hpp"

using autonomy_planning::AstarPlanner;
using autonomy_planning::grid_path_to_world_path;
using autonomy_planning::GridIndex;
using autonomy_planning::GridMap;
using autonomy_planning::has_line_of_sight;
using autonomy_planning::inflate_obstacles;
using autonomy_planning::PlannerResult;
using autonomy_planning::smooth_grid_path_line_of_sight;

void test_astar_conversion()
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

    const autonomy_core::Path converted_world_path =
        grid_path_to_world_path(map, raw_result->grid_path);

    int i = 0;
    for (autonomy_core::Pose3 pose : converted_world_path.poses)
    {
        assert(pose.position ==
               raw_result.value().world_path.poses.at(i).position);
        assert(pose.orientation_rpy_rad ==
               raw_result.value().world_path.poses.at(i).orientation_rpy_rad);
        i++;
    }
}

int main(int argc, char* argv[])
{
    (void)argc;
    (void)argv;

    test_astar_conversion();

    return 0;
}
