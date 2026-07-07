#include <cassert>
#include <stdexcept>

#include "autonomy_planning/grid_index.hpp"
#include "autonomy_planning/grid_map.hpp"
#include "autonomy_planning/obstacle_inflation.hpp"

using autonomy_planning::GridIndex;
using autonomy_planning::GridMap;
using autonomy_planning::inflate_obstacles;

void assert_occupied(const GridMap& map, const GridIndex& index)
{
    assert(map.is_occupied(index));
}

void assert_free(const GridMap& map, const GridIndex& index)
{
    assert(!map.is_occupied(index));
}

void test_zero_radius_preserves_original_obstacle()
{
    GridMap raw_map{5, 5, 1.0};
    raw_map.set_occupied(GridIndex{2, 2}, true);

    const GridMap inflated_map = inflate_obstacles(raw_map, 0.0);

    // Expected:
    //
    // . . . . .
    // . . . . .
    // . . X . .
    // . . . . .
    // . . . . .

    assert_occupied(inflated_map, GridIndex{2, 2});

    assert_free(inflated_map, GridIndex{1, 2});
    assert_free(inflated_map, GridIndex{3, 2});
    assert_free(inflated_map, GridIndex{2, 1});
    assert_free(inflated_map, GridIndex{2, 3});

    assert_free(inflated_map, GridIndex{1, 1});
    assert_free(inflated_map, GridIndex{1, 3});
    assert_free(inflated_map, GridIndex{3, 1});
    assert_free(inflated_map, GridIndex{3, 3});
}

void test_radius_one_inflates_cardinal_neighbors()
{
    GridMap raw_map{5, 5, 1.0};
    raw_map.set_occupied(GridIndex{2, 2}, true);

    const GridMap inflated_map = inflate_obstacles(raw_map, 1.0);

    // Expected:
    //
    // . . . . .
    // . . X . .
    // . X X X .
    // . . X . .
    // . . . . .
    //
    // Diagonals stay free because sqrt(2) > 1.0.

    assert_occupied(inflated_map, GridIndex{2, 2});

    assert_occupied(inflated_map, GridIndex{1, 2});
    assert_occupied(inflated_map, GridIndex{3, 2});
    assert_occupied(inflated_map, GridIndex{2, 1});
    assert_occupied(inflated_map, GridIndex{2, 3});

    assert_free(inflated_map, GridIndex{1, 1});
    assert_free(inflated_map, GridIndex{1, 3});
    assert_free(inflated_map, GridIndex{3, 1});
    assert_free(inflated_map, GridIndex{3, 3});
}

void test_radius_one_point_five_includes_diagonals()
{
    GridMap raw_map{5, 5, 1.0};
    raw_map.set_occupied(GridIndex{2, 2}, true);

    const GridMap inflated_map = inflate_obstacles(raw_map, 1.5);

    // Expected:
    //
    // . . . . .
    // . X X X .
    // . X X X .
    // . X X X .
    // . . . . .
    //
    // Diagonals are occupied because sqrt(2) < 1.5.

    assert_occupied(inflated_map, GridIndex{2, 2});

    assert_occupied(inflated_map, GridIndex{1, 2});
    assert_occupied(inflated_map, GridIndex{3, 2});
    assert_occupied(inflated_map, GridIndex{2, 1});
    assert_occupied(inflated_map, GridIndex{2, 3});

    assert_occupied(inflated_map, GridIndex{1, 1});
    assert_occupied(inflated_map, GridIndex{1, 3});
    assert_occupied(inflated_map, GridIndex{3, 1});
    assert_occupied(inflated_map, GridIndex{3, 3});

    assert_free(inflated_map, GridIndex{0, 0});
    assert_free(inflated_map, GridIndex{0, 4});
    assert_free(inflated_map, GridIndex{4, 0});
    assert_free(inflated_map, GridIndex{4, 4});
}

void test_boundary_obstacle_does_not_throw()
{
    GridMap raw_map{5, 5, 1.0};
    raw_map.set_occupied(GridIndex{0, 0}, true);

    const GridMap inflated_map = inflate_obstacles(raw_map, 1.5);

    // Expected near top-left corner:
    //
    // X X . . .
    // X X . . .
    // . . . . .
    // . . . . .
    // . . . . .

    assert_occupied(inflated_map, GridIndex{0, 0});
    assert_occupied(inflated_map, GridIndex{0, 1});
    assert_occupied(inflated_map, GridIndex{1, 0});
    assert_occupied(inflated_map, GridIndex{1, 1});

    assert_free(inflated_map, GridIndex{0, 2});
    assert_free(inflated_map, GridIndex{2, 0});
    assert_free(inflated_map, GridIndex{4, 4});
}

void test_negative_radius_throws()
{
    GridMap raw_map{5, 5, 1.0};

    bool threw = false;

    try
    {
        const GridMap inflated_map = inflate_obstacles(raw_map, -1.0);
        (void)inflated_map;
    }
    catch (const std::invalid_argument&)
    {
        threw = true;
    }

    assert(threw);
}

int main()
{
    test_zero_radius_preserves_original_obstacle();
    test_radius_one_inflates_cardinal_neighbors();
    test_radius_one_point_five_includes_diagonals();
    test_boundary_obstacle_does_not_throw();
    test_negative_radius_throws();

    return 0;
}
