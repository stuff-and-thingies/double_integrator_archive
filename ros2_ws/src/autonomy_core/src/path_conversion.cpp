#include "autonomy_planning/path_conversion.hpp"

#include "autonomy_core/vector3.hpp"

namespace autonomy_planning
{

autonomy_core::Path grid_path_to_world_path(
    const GridMap& map, const std::vector<GridIndex>& grid_path)
{
    autonomy_core::Path path;

    std::vector<autonomy_core::Pose3> poses;
    poses.reserve(grid_path.size());

    for (const GridIndex& grid_index : grid_path)
    {
        const autonomy_core::Vector2 world_pos_vec2 =
            map.grid_to_world(grid_index);

        const autonomy_core::Vector3 world_pos{world_pos_vec2.x,
                                               world_pos_vec2.y, 0.0};

        poses.emplace_back(world_pos, autonomy_core::Vector3{});
    }

    path.poses = poses;

    return path;
}

}  // namespace autonomy_planning
