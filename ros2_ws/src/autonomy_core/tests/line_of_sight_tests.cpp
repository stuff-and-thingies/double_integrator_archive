#include <cassert>

#include "autonomy_planning/grid_index.hpp"
#include "autonomy_planning/grid_map.hpp"
#include "autonomy_planning/line_of_sight.hpp"

using autonomy_planning::GridIndex;
using autonomy_planning::GridMap;
using autonomy_planning::has_line_of_sight;

void test_out_of_bound()
{
    GridMap map{3, 3, 1.0};

    const GridIndex oob_start{-1, 0};
    const GridIndex normal_start{0, 0};

    const GridIndex oob_goal{-1, 1};
    const GridIndex normal_goal{0, 1};

    assert(!has_line_of_sight(map, oob_start, normal_goal));
    assert(!has_line_of_sight(map, normal_start, oob_goal));
}

void test_start_goal_occupied()
{
    GridMap map{3, 3, 1.0};

    const GridIndex start = {0, 0};
    const GridIndex goal = {0, 2};

    map.set_occupied(start, true);

    assert(!has_line_of_sight(map, start, goal));

    map.set_occupied(start, false);
    map.set_occupied(goal, true);

    assert(!has_line_of_sight(map, start, goal));
}

void test_start_is_goal()
{
    GridMap map{3, 3, 1.0};

    const GridIndex start = {0, 0};
    const GridIndex goal = {0, 0};

    assert(has_line_of_sight(map, start, goal));
}

void test_straight_line_of_sight()
{
    GridMap map{3, 3, 1.0};

    const GridIndex start = {0, 0};
    const GridIndex goal = {0, 2};

    assert(has_line_of_sight(map, start, goal));
}

void test_straight_line_of_sight_with_obstacle()
{
    GridMap map{3, 3, 1.0};

    map.set_occupied(GridIndex{0, 1}, true);

    const GridIndex start = {0, 0};
    const GridIndex goal = {0, 2};

    assert(!has_line_of_sight(map, start, goal));
}

void test_vertical_line_of_sight()
{
    GridMap map{3, 3, 1.0};

    assert(has_line_of_sight(map, GridIndex{0, 1}, GridIndex{2, 1}));
}

void test_vertical_line_of_sight_with_obstacle()
{
    GridMap map{3, 3, 1.0};
    map.set_occupied(GridIndex{1, 1}, true);

    assert(!has_line_of_sight(map, GridIndex{0, 1}, GridIndex{2, 1}));
}

void test_diagonal_line_of_sight()
{
    GridMap map{3, 3, 1.0};

    const GridIndex start = {0, 0};
    const GridIndex goal = {2, 2};

    assert(has_line_of_sight(map, start, goal));
}

void test_diagonal_line_of_sight_with_obstacle()
{
    GridMap map{3, 3, 1.0};

    map.set_occupied(GridIndex{1, 1}, true);

    const GridIndex start = {0, 0};
    const GridIndex goal = {2, 2};

    assert(!has_line_of_sight(map, start, goal));
}

void test_negative_slope_diagonal()
{
    GridMap map{3, 3, 1.0};

    assert(has_line_of_sight(map, GridIndex{2, 0}, GridIndex{0, 2}));
}

void test_negative_slope_diagonal_with_obstacle()
{
    GridMap map{3, 3, 1.0};
    map.set_occupied(GridIndex{1, 1}, true);

    assert(!has_line_of_sight(map, GridIndex{2, 0}, GridIndex{0, 2}));
}

void test_diagonal_corner_cutting_is_blocked()
{
    GridMap map{2, 2, 1.0};

    map.set_occupied(GridIndex{0, 1}, true);
    map.set_occupied(GridIndex{1, 0}, true);

    const GridIndex start{0, 0};
    const GridIndex goal{1, 1};

    assert(!has_line_of_sight(map, start, goal));
}

void test_diagonal_corner_cutting_blocked_by_lower_side()
{
    GridMap map{2, 2, 1.0};

    map.set_occupied(GridIndex{1, 0}, true);

    assert(!has_line_of_sight(map, GridIndex{0, 0}, GridIndex{1, 1}));
}

void test_diagonal_corner_cutting_blocked_by_upper_side()
{
    GridMap map{2, 2, 1.0};

    map.set_occupied(GridIndex{0, 1}, true);

    assert(!has_line_of_sight(map, GridIndex{0, 0}, GridIndex{1, 1}));
}

int main(int argc, char* argv[])
{
    (void)argc;
    (void)argv;

    test_out_of_bound();
    test_start_goal_occupied();
    test_start_is_goal();
    test_straight_line_of_sight();
    test_straight_line_of_sight_with_obstacle();
    test_diagonal_line_of_sight();
    test_diagonal_line_of_sight_with_obstacle();
    test_vertical_line_of_sight();
    test_vertical_line_of_sight_with_obstacle();
    test_negative_slope_diagonal();
    test_negative_slope_diagonal_with_obstacle();
    test_diagonal_corner_cutting_is_blocked();
    test_diagonal_corner_cutting_blocked_by_lower_side();
    test_diagonal_corner_cutting_blocked_by_upper_side();

    return 0;
}
