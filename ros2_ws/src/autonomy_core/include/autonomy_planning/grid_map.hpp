#pragma once

#include <optional>
#include <vector>

#include "autonomy_core/vector2.hpp"
#include "autonomy_planning/grid_index.hpp"

namespace autonomy_planning
{

class GridMap
{
   public:
    GridMap(int width = 1, int height = 1, double resolution_m = 1.0);

    [[nodiscard]] int width() const noexcept;
    [[nodiscard]] int height() const noexcept;
    [[nodiscard]] double resolution_m() const noexcept;
    [[nodiscard]] std::vector<size_t> get_occupied_indices() const noexcept;

    [[nodiscard]] bool in_bounds(const GridIndex& index) const noexcept;

    [[nodiscard]] bool is_occupied(const GridIndex& index) const noexcept;

    void set_occupied(const GridIndex& index, bool occupied);

    [[nodiscard]] std::optional<GridIndex> world_to_grid(
        const autonomy_core::Vector2& point_m) const noexcept;

    [[nodiscard]] autonomy_core::Vector2 grid_to_world(
        const GridIndex& index) const;

    GridIndex flat_index_to_grid_index(size_t flat_index) const noexcept;

    int flat_index(GridIndex index) const noexcept;

   private:
    int width_{0};
    int height_{0};
    double resolution_m_{1.0};

    std::vector<bool> occupied_;
};

}  // namespace autonomy_planning
