#include <cassert>

#include "autonomy_planning/grid_index.hpp"
#include "autonomy_planning/grid_map.hpp"
#include "autonomy_planning/grid_neighbors.hpp"

using autonomy_planning::free_neighbors_4;
using autonomy_planning::GridIndex;
using autonomy_planning::GridMap;

int main(int argc, char* argv[])
{
    (void)argc;
    (void)argv;

    GridMap map{3, 3, 1.0};

    GridIndex center_ind{1, 1};

    GridIndex corner_ind1{0, 0};
    GridIndex corner_ind2{0, 2};
    GridIndex corner_ind3{2, 0};
    GridIndex corner_ind4{2, 2};

    GridIndex edge_ind1{0, 1};
    GridIndex edge_ind2{1, 0};
    GridIndex edge_ind3{1, 2};
    GridIndex edge_ind4{2, 1};

    assert(free_neighbors_4(map, center_ind).size() == 4);

    assert(free_neighbors_4(map, corner_ind1).size() == 2);
    assert(free_neighbors_4(map, corner_ind2).size() == 2);
    assert(free_neighbors_4(map, corner_ind3).size() == 2);
    assert(free_neighbors_4(map, corner_ind4).size() == 2);

    assert(free_neighbors_4(map, edge_ind1).size() == 3);
    assert(free_neighbors_4(map, edge_ind2).size() == 3);
    assert(free_neighbors_4(map, edge_ind3).size() == 3);
    assert(free_neighbors_4(map, edge_ind4).size() == 3);

    map.set_occupied(GridIndex{0, 1}, true);
    assert(free_neighbors_4(map, center_ind).size() == 3);

    assert(free_neighbors_4(map, GridIndex{10, 10}).size() == 0);

    return 0;
}
