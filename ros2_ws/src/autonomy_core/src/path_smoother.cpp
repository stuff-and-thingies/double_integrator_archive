#include "autonomy_planning/path_smoother.hpp"

#include "autonomy_planning/line_of_sight.hpp"

namespace autonomy_planning
{

std::vector<GridIndex> smooth_grid_path_line_of_sight(
    const GridMap& map, const std::vector<GridIndex>& raw_path)
{
    if (raw_path.size() <= 2)
    {
        return raw_path;
    }

    std::vector<GridIndex> smoothed;

    size_t current_index = 0;
    smoothed.emplace_back(raw_path.at(current_index));

    while (current_index < (raw_path.size() - 1))
    {
        size_t farthest_visible = current_index + 1;

        // check backwards from the end
        for (size_t candidate_index = raw_path.size() - 1;
             candidate_index > current_index; --candidate_index)
        {
            if (has_line_of_sight(map, raw_path.at(current_index),
                                  raw_path.at(candidate_index)))
            {
                farthest_visible = candidate_index;
                break;
            }
        }

        smoothed.emplace_back(raw_path.at(farthest_visible));
        current_index = farthest_visible;
    }

    return smoothed;
}

}  // namespace autonomy_planning
