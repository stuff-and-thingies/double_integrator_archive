#include "autonomy_planning/obstacle_inflation.hpp"

#include <cmath>
#include <cstdlib>
#include <stdexcept>
#include <vector>

namespace autonomy_planning
{

GridMap inflate_obstacles(const GridMap& input_map, double inflation_radius_m)
{
    if (inflation_radius_m < 0.0)
    {
        throw std::invalid_argument(
            "inflation radius must be non-negative [m]");
    }

    std::vector<size_t> occupied_indices = input_map.get_occupied_indices();

    if (occupied_indices.empty())
    {
        return input_map;
    }

    GridMap output_map{input_map.width(), input_map.height(),
                       input_map.resolution_m()};

    const int inflation_cells = static_cast<int>(
        std::ceil(inflation_radius_m / input_map.resolution_m()));

    for (size_t occupied_index : occupied_indices)
    {
        const std::ldiv_t grid = std::div(static_cast<long>(occupied_index),
                                          static_cast<long>(input_map.width()));

        const GridIndex occupied_grid_index{static_cast<int>(grid.quot),
                                            static_cast<int>(grid.rem)};

        output_map.set_occupied(occupied_grid_index, true);

        if (inflation_cells < 1)
        {
            continue;
        }

        for (int dr = -inflation_cells; dr <= inflation_cells; ++dr)
        {
            for (int dc = -inflation_cells; dc <= inflation_cells; ++dc)
            {
                const GridIndex candidate{occupied_grid_index.row + dr,
                                          occupied_grid_index.col + dc};

                if (!input_map.in_bounds(candidate))
                {
                    continue;
                }

                const double distance_m =
                    input_map.resolution_m() *
                    std::sqrt(static_cast<double>(dr * dr + dc * dc));

                if (distance_m <= inflation_radius_m)
                {
                    output_map.set_occupied(candidate, true);
                }
            }
        }
    }

    return output_map;
}

}  // namespace autonomy_planning
