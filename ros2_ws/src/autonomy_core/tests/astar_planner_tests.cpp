#include <cassert>
#include <optional>

#include "autonomy_planning/astar_planner.hpp"
#include "autonomy_planning/dijkstra_planner.hpp"
#include "autonomy_planning/grid_index.hpp"
#include "autonomy_planning/grid_map.hpp"
#include "test_utils.hpp"

using autonomy_planning::AstarPlanner;
using autonomy_planning::DijkstraPlanner;
using autonomy_planning::GridIndex;
using autonomy_planning::GridMap;
using autonomy_planning::PlannerResult;

int main(int argc, char* argv[])
{
    (void)argc;
    (void)argv;

    AstarPlanner astar_planner;
    DijkstraPlanner dijkstra_planner;
    GridMap straight_map{3, 3, 1.0};
    GridMap obstacle_map{3, 3, 1.0};
    GridMap no_path_map{3, 3, 1.0};
    GridMap goal_is_obstacle_map{3, 3, 1.0};

    // start == goal test
    GridIndex start1{1, 1};
    GridIndex goal1{1, 1};

    std::optional<PlannerResult> result1 =
        astar_planner.plan(straight_map, start1, goal1);

    assert(result1.has_value() == true);
    assert(result1.value().world_path.size() == 1);
    assert(nearly_equal(result1.value().cost_m, 0.0));

    // straight path test
    GridIndex start2{0, 0};
    GridIndex goal2{0, 2};

    std::optional<PlannerResult> result2 =
        astar_planner.plan(straight_map, start2, goal2);

    assert(result2.has_value() == true);
    assert(
        nearly_equal(result2.value().world_path.poses.at(0).position.x, 0.5));
    assert(
        nearly_equal(result2.value().world_path.poses.at(0).position.y, 0.5));
    assert(
        nearly_equal(result2.value().world_path.poses.back().position.x, 2.5));
    assert(
        nearly_equal(result2.value().world_path.poses.back().position.y, 0.5));

    assert(nearly_equal(result2.value().cost_m, 2.0));

    // go around obstacle test
    obstacle_map.set_occupied(GridIndex{0, 1}, true);
    GridIndex start3{0, 0};
    GridIndex goal3{0, 2};

    std::optional<PlannerResult> result3_astar =
        astar_planner.plan(obstacle_map, start3, goal3);

    std::optional<PlannerResult> result3_dijkstra =
        dijkstra_planner.plan(obstacle_map, start3, goal3);

    assert(result3_astar.has_value() == true);

    for (const autonomy_core::Pose3 pose :
         result3_astar.value().world_path.poses)
    {
        double pose_x = pose.position.x;
        double pose_y = pose.position.y;

        double obstacle_x = 1.5;
        double obstacle_y = 0.5;

        if (nearly_equal(pose_x, obstacle_x) &&
            nearly_equal(pose_y, obstacle_y))
        {
            assert(false);
        }
    }

    assert(nearly_equal(result3_astar.value().cost_m, 4.0));

    assert(nearly_equal(
        result3_astar.value().world_path.poses.front().position.x, 0.5));
    assert(nearly_equal(
        result3_astar.value().world_path.poses.front().position.y, 0.5));
    assert(nearly_equal(
        result3_astar.value().world_path.poses.back().position.x, 2.5));
    assert(nearly_equal(
        result3_astar.value().world_path.poses.back().position.y, 0.5));

    assert(nearly_equal(result3_dijkstra.value().cost_m,
                        result3_astar.value().cost_m));

    // no path exists test
    no_path_map.set_occupied(GridIndex{0, 1}, true);
    no_path_map.set_occupied(GridIndex{1, 1}, true);
    no_path_map.set_occupied(GridIndex{2, 1}, true);

    GridIndex start4{0, 0};
    GridIndex goal4{0, 2};

    std::optional<PlannerResult> result4 =
        astar_planner.plan(no_path_map, start4, goal4);

    assert(result4 == std::nullopt);

    // invalid start test
    std::optional<PlannerResult> result5 =
        astar_planner.plan(straight_map, GridIndex{-1, -1}, goal4);

    assert(result5 == std::nullopt);

    // goal == obstacle test
    goal_is_obstacle_map.set_occupied(GridIndex{0, 1}, true);
    std::optional<PlannerResult> result6 = astar_planner.plan(
        goal_is_obstacle_map, GridIndex{0, 0}, GridIndex{0, 1});

    assert(result6 == std::nullopt);

    GridMap large_map{10, 3, 1.0};
    GridIndex start7{0, 0};
    GridIndex goal7{0, 9};

    std::optional<PlannerResult> result7_astar =
        astar_planner.plan(large_map, start7, goal7);
    std::optional<PlannerResult> result7_dijkstra =
        dijkstra_planner.plan(large_map, start7, goal7);

    assert(result7_astar.has_value() == true);
    assert(result7_dijkstra.has_value() == true);

    assert(result7_astar.value().expanded_nodes <=
           result7_dijkstra.value().expanded_nodes);

    return 0;
}
