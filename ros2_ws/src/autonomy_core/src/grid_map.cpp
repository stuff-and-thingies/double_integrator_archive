#include "autonomy_planning/grid_map.hpp"

#include <cmath>
#include <stdexcept>

namespace autonomy_planning
{

GridMap::GridMap(int width, int height, double resolution_m)
    : width_{width}, height_{height}, resolution_m_{resolution_m}
{
    if (width <= 0 || height <= 0 || resolution_m <= 0.0)
    {
        throw std::invalid_argument(
            "GridMap dimensions and resolution must be positive");
    }
    occupied_.resize(width_ * height_, false);
}

int GridMap::width() const noexcept { return width_; }

int GridMap::height() const noexcept { return height_; }

double GridMap::resolution_m() const noexcept { return resolution_m_; }

std::vector<size_t> GridMap::get_occupied_indices() const noexcept
{
    std::vector<size_t> true_indices;

    for (size_t i = 0; i < occupied_.size(); ++i)
    {
        if (occupied_[i])
        {
            true_indices.push_back(i);
        }
    }
    return true_indices;
}

bool GridMap::in_bounds(const GridIndex& index) const noexcept
{
    return index.row >= 0 && index.row < height_ && index.col >= 0 &&
           index.col < width_;
}

bool GridMap::is_occupied(const GridIndex& index) const noexcept
{
    if (!in_bounds(index))
    {
        return true;
    }

    return occupied_.at(flat_index(index));
}

void GridMap::set_occupied(const GridIndex& index, bool occupied)
{
    if (!in_bounds(index))
    {
        throw std::out_of_range("GridMap::set_occupied index out of bounds");
    }

    occupied_.at(flat_index(index)) = occupied;
}

int GridMap::flat_index(GridIndex index) const noexcept
{
    return width_ * index.row + index.col;
}

GridIndex GridMap::flat_index_to_grid_index(size_t flat_index) const noexcept
{
    const int row = static_cast<int>(flat_index / width_);
    const int col = static_cast<int>(flat_index % width_);

    return GridIndex{row, col};
}

std::optional<GridIndex> GridMap::world_to_grid(
    const autonomy_core::Vector2& point_m) const noexcept
{
    const int col = static_cast<int>(std::floor(point_m.x / resolution_m_));
    const int row = static_cast<int>(std::floor(point_m.y / resolution_m_));

    std::optional<GridIndex> grid_index = GridIndex(row, col);

    if (!in_bounds(grid_index.value()))
    {
        grid_index.reset();
    }

    return grid_index;
}

autonomy_core::Vector2 GridMap::grid_to_world(const GridIndex& index) const
{
    return autonomy_core::Vector2(
        (static_cast<double>(index.col) + 0.5) * resolution_m_,
        (static_cast<double>(index.row) + 0.5) * resolution_m_);
}

}  // namespace autonomy_planning
