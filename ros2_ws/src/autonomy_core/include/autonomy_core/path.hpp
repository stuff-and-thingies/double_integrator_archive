#pragma once

#include <cstddef>
#include <vector>

#include "autonomy_core/pose3.hpp"

namespace autonomy_core
{
struct Path
{
    std::vector<Pose3> poses{};

    bool empty() const noexcept { return this->poses.empty(); }

    std::size_t size() const noexcept { return this->poses.size(); }

    double length() const noexcept
    {
        double len_sum = 0.0;

        if (this->empty() || this->size() == 1)
        {
            return len_sum;
        }

        for (size_t i = 1; i < this->size(); ++i)
        {
            len_sum += (poses[i].position - poses[i - 1].position).norm();
        }

        return len_sum;
    }
};
}  // namespace autonomy_core
