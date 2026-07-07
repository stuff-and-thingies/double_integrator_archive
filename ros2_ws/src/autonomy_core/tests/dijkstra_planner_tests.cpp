#include <cassert>
#include <optional>

#include "autonomy_planning/dijkstra_planner.hpp"
#include "autonomy_planning/grid_index.hpp"
#include "autonomy_planning/grid_map.hpp"
#include "test_utils.hpp"

using autonomy_planning::DijkstraPlanner;
using autonomy_planning::GridIndex;
using autonomy_planning::GridMap;
using autonomy_planning::PlannerResult;

int main(int argc, char* argv[])
{
    (void)argc;
    (void)argv;

    DijkstraPlanner planner;
    GridMap map{3, 3, 1};

    // start == goal test
    GridIndex start1{1, 1};
    GridIndex goal1{1, 1};

    std::optional<PlannerResult> result1 = planner.plan(map, start1, goal1);

    assert(result1.has_value() == true);
    assert(result1.value().world_path.size() == 1);
    assert(nearly_equal(result1.value().cost_m, 0.0));

    // straight path test
    GridIndex start2{0, 0};
    GridIndex goal2{0, 2};

    std::optional<PlannerResult> result2 = planner.plan(map, start2, goal2);

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

    assert(result2.value().grid_path.front() == start2);
    assert(result2.value().grid_path.back() == goal2);
    assert(result2.value().grid_path.size() ==
           result2.value().world_path.size());

    // go around obstacle test
    map.set_occupied(GridIndex{0, 1}, true);
    GridIndex start3{0, 0};
    GridIndex goal3{0, 2};

    std::optional<PlannerResult> result3 = planner.plan(map, start3, goal3);

    assert(result3.has_value() == true);

    for (const autonomy_core::Pose3 pose : result3.value().world_path.poses)
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

    assert(nearly_equal(result3.value().cost_m, 4.0));

    assert(
        nearly_equal(result3.value().world_path.poses.front().position.x, 0.5));
    assert(
        nearly_equal(result3.value().world_path.poses.front().position.y, 0.5));
    assert(
        nearly_equal(result3.value().world_path.poses.back().position.x, 2.5));
    assert(
        nearly_equal(result3.value().world_path.poses.back().position.y, 0.5));

    // no path exists test
    map.set_occupied(GridIndex{1, 1}, true);
    map.set_occupied(GridIndex{2, 1}, true);

    GridIndex start4{0, 0};
    GridIndex goal4{0, 2};

    std::optional<PlannerResult> result4 = planner.plan(map, start4, goal4);

    assert(result4 == std::nullopt);

    // invalid start test
    std::optional<PlannerResult> result5 =
        planner.plan(map, GridIndex{-1, -1}, goal4);

    assert(result5 == std::nullopt);

    // goal == obstacle test
    std::optional<PlannerResult> result6 =
        planner.plan(map, GridIndex{0, 0}, GridIndex{0, 1});

    assert(result6 == std::nullopt);

    return 0;
}
