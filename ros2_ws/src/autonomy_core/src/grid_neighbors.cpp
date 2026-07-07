#include "autonomy_planning/grid_neighbors.hpp"

#include "autonomy_planning/grid_map.hpp"

namespace autonomy_planning
{

std::vector<GridIndex> free_neighbors_4(const GridMap& map,
                                        const GridIndex& index)
{
    std::vector<GridIndex> neighbors;
    neighbors.reserve(4);

    if (!map.in_bounds(index))
    {
        return neighbors;
    }

    for (int i = 0; i < 4; ++i)
    {
        GridIndex candidate_index{};

        switch (i)
        {
            case 0:
                candidate_index.row = index.row - 1;
                candidate_index.col = index.col;
                break;
            case 1:
                candidate_index.row = index.row + 1;
                candidate_index.col = index.col;
                break;
            case 2:
                candidate_index.row = index.row;
                candidate_index.col = index.col - 1;
                break;
            case 3:
                candidate_index.row = index.row;
                candidate_index.col = index.col + 1;
                break;
        }

        if (map.in_bounds(candidate_index) && !map.is_occupied(candidate_index))
        {
            neighbors.emplace_back(candidate_index);
        }
    }

    return neighbors;
}

}  // namespace autonomy_planning
