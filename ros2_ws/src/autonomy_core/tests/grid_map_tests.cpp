#include <cassert>
#include <stdexcept>

#include "autonomy_core/vector2.hpp"
#include "autonomy_planning/grid_map.hpp"
#include "test_utils.hpp"

using autonomy_core::Vector2;
using autonomy_planning::GridIndex;
using autonomy_planning::GridMap;

int main(int argc, char* argv[])
{
    (void)argc;
    (void)argv;

    GridMap map{10, 20, 0.5};

    // index test
    GridIndex test_ind1{0, 0};
    assert(map.in_bounds(test_ind1) == true);

    GridIndex test_ind2{19, 9};
    assert(map.in_bounds(test_ind2) == true);

    GridIndex test_ind3{-1, 0};
    assert(map.in_bounds(test_ind3) == false);

    GridIndex test_ind4{0, -1};
    assert(map.in_bounds(test_ind4) == false);

    GridIndex test_ind5{20, 0};
    assert(map.in_bounds(test_ind5) == false);

    GridIndex test_ind6{0, 10};
    assert(map.in_bounds(test_ind6) == false);

    // occupancy test
    GridIndex origin{0, 0};
    assert(map.is_occupied(origin) == false);

    map.set_occupied(origin, true);
    assert(map.is_occupied(origin) == true);

    map.set_occupied(origin, false);
    assert(map.is_occupied(origin) == false);

    // conversion tests
    Vector2 test_point1{0.25, 0.25};
    std::optional<GridIndex> candidate_index1 = map.world_to_grid(test_point1);
    assert((candidate_index1 == GridIndex{0, 0}) == true);

    Vector2 test_point2{1.25, 2.75};
    std::optional<GridIndex> candidate_index2 = map.world_to_grid(test_point2);
    assert((candidate_index2 == GridIndex{5, 2}) == true);

    Vector2 test_point3{-0.5, 0.25};
    std::optional<GridIndex> candidate_index3 = map.world_to_grid(test_point3);
    assert((candidate_index3.has_value()) == false);

    Vector2 test_point4{100.0, 0.0};
    std::optional<GridIndex> candidate_index4 = map.world_to_grid(test_point4);
    assert((candidate_index4.has_value()) == false);

    Vector2 test_point5{0.0, 100.0};
    std::optional<GridIndex> candidate_index5 = map.world_to_grid(test_point5);
    assert((candidate_index5.has_value()) == false);

    const Vector2 world_origin = map.grid_to_world(GridIndex{0, 0});
    assert(nearly_equal(world_origin.x, 0.25));
    assert(nearly_equal(world_origin.y, 0.25));

    const Vector2 world_5_2 = map.grid_to_world(GridIndex{5, 2});
    assert(nearly_equal(world_5_2.x, 1.25));
    assert(nearly_equal(world_5_2.y, 2.75));

    // invalid constructor
    bool threw_invalid_width = false;
    try
    {
        GridMap bad_map{0, 10, 0.5};
    }
    catch (const std::invalid_argument&)
    {
        threw_invalid_width = true;
    }
    assert(threw_invalid_width == true);

    // invalid occupancy set
    bool threw_bad_index = false;
    try
    {
        map.set_occupied(GridIndex{-1, 0}, true);
    }
    catch (const std::out_of_range&)
    {
        threw_bad_index = true;
    }
    assert(threw_bad_index == true);

    return 0;
}
