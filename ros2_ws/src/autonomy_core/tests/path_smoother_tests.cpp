#include <cassert>
#include <vector>

#include "autonomy_planning/grid_index.hpp"
#include "autonomy_planning/grid_map.hpp"
#include "autonomy_planning/line_of_sight.hpp"
#include "autonomy_planning/path_smoother.hpp"

using autonomy_planning::GridIndex;
using autonomy_planning::GridMap;
using autonomy_planning::smooth_grid_path_line_of_sight;

void test_empty_path()
{
    std::vector<GridIndex> raw_path;

    GridMap map{3, 3, 1.0};
    std::vector<GridIndex> smoothed_path =
        smooth_grid_path_line_of_sight(map, raw_path);

    assert(smoothed_path.size() == 0);
}

void test_one_point_path()
{
    GridMap map{3, 3, 1.0};
    std::vector<GridIndex> raw_path;
    raw_path.emplace_back(0, 0);

    std::vector<GridIndex> smoothed_path =
        smooth_grid_path_line_of_sight(map, raw_path);

    assert(smoothed_path.size() == 1);
}

void test_two_points_path()
{
    GridMap map{3, 3, 1.0};
    std::vector<GridIndex> raw_path;
    raw_path.emplace_back(0, 0);
    raw_path.emplace_back(1, 0);

    std::vector<GridIndex> smoothed_path =
        smooth_grid_path_line_of_sight(map, raw_path);

    assert(smoothed_path.size() == 2);
}

void test_straight_path()
{
    GridMap map{3, 3, 1.0};
    std::vector<GridIndex> raw_path;
    raw_path.emplace_back(0, 0);
    raw_path.emplace_back(0, 1);
    raw_path.emplace_back(0, 2);

    std::vector<GridIndex> smoothed_path =
        smooth_grid_path_line_of_sight(map, raw_path);

    assert(smoothed_path.front() == raw_path.front());
    assert(smoothed_path.back() == raw_path.back());
    assert(smoothed_path.size() == 2);
}

void test_diagonal_path()
{
    GridMap map{3, 3, 1.0};
    std::vector<GridIndex> raw_path;
    raw_path.emplace_back(0, 0);
    raw_path.emplace_back(1, 1);
    raw_path.emplace_back(2, 2);

    std::vector<GridIndex> smoothed_path =
        smooth_grid_path_line_of_sight(map, raw_path);

    assert(smoothed_path.front() == raw_path.front());
    assert(smoothed_path.back() == raw_path.back());
    assert(smoothed_path.size() == 2);
}

void test_path_with_obstacle()
{
    GridMap map{3, 3, 1.0};
    map.set_occupied(GridIndex{1, 1}, true);

    std::vector<GridIndex> raw_path;
    raw_path.emplace_back(0, 0);
    raw_path.emplace_back(0, 1);
    raw_path.emplace_back(0, 2);
    raw_path.emplace_back(1, 2);
    raw_path.emplace_back(2, 2);

    std::vector<GridIndex> smoothed_path =
        smooth_grid_path_line_of_sight(map, raw_path);

    assert(smoothed_path.front() == raw_path.front());
    assert(smoothed_path.back() == raw_path.back());
    assert(smoothed_path.size() > 2);
}

void test_blocked_adjacent_segment_does_not_hang()
{
    GridMap map{3, 3, 1.0};
    map.set_occupied(GridIndex{0, 1}, true);

    std::vector<GridIndex> raw_path;
    raw_path.emplace_back(0, 0);
    raw_path.emplace_back(0, 1);
    raw_path.emplace_back(0, 2);

    const std::vector<GridIndex> smoothed_path =
        smooth_grid_path_line_of_sight(map, raw_path);

    assert(smoothed_path.front() == raw_path.front());
}

int main(int argc, char* argv[])
{
    (void)argc;
    (void)argv;

    test_empty_path();
    test_one_point_path();
    test_two_points_path();
    test_straight_path();
    test_diagonal_path();
    test_path_with_obstacle();
    test_blocked_adjacent_segment_does_not_hang();

    return 0;
}
