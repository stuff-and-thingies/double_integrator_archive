#include "autonomy_planning/line_of_sight.hpp"

#include <cstdlib>

namespace autonomy_planning
{

bool has_line_of_sight(const GridMap& map, const GridIndex& start,
                       const GridIndex& goal)
{
    if (!map.in_bounds(start) || !map.in_bounds(goal))
    {
        return false;
    }

    if (map.is_occupied(start) || map.is_occupied(goal))
    {
        return false;
    }

    if (start == goal)
    {
        return true;
    }

    int x0 = start.col;
    int y0 = start.row;

    const int x1 = goal.col;
    const int y1 = goal.row;

    const int dx = std::abs(x1 - x0);
    const int dy = std::abs(y1 - y0);

    const int sx = (x0 < x1) ? 1 : -1;
    const int sy = (y0 < y1) ? 1 : -1;

    int err = dx - dy;

    while (true)
    {
        if (x0 == x1 && y0 == y1)
        {
            return true;
        }

        const int old_x = x0;
        const int old_y = y0;

        bool stepped_x = false;
        bool stepped_y = false;

        const int e2 = 2 * err;

        if (e2 > -dy)
        {
            err -= dy;
            x0 += sx;
            stepped_x = true;
        }

        if (e2 < dx)
        {
            err += dx;
            y0 += sy;
            stepped_y = true;
        }

        if (stepped_x && stepped_y)
        {
            if (map.is_occupied(GridIndex{old_y, x0}) ||
                map.is_occupied(GridIndex{y0, old_x}))
            {
                return false;
            }
        }

        if (map.is_occupied(GridIndex{y0, x0}))
        {
            return false;
        }
    }
}

}  // namespace autonomy_planning
