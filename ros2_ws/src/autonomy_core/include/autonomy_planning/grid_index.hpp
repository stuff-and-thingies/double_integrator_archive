#pragma once

namespace autonomy_planning
{
struct GridIndex
{
    int row{};
    int col{};

    constexpr GridIndex() = default;

    constexpr GridIndex(int row_in, int col_in) noexcept
        : row{row_in}, col{col_in}
    {
    }

    [[nodiscard]] constexpr bool operator==(
        const GridIndex& other) const noexcept
    {
        return row == other.row && col == other.col;
    }

    [[nodiscard]] constexpr bool operator!=(
        const GridIndex& other) const noexcept
    {
        return !(*this == other);
    }
};

}  // namespace autonomy_planning
