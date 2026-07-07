#pragma once

#include <cstddef>
#include <vector>

#include "autonomy_core/pose3.hpp"
#include "autonomy_core/time.hpp"
#include "autonomy_core/twist.hpp"

namespace autonomy_core
{

struct TrajectoryPoint
{
    Timestamp time{};
    Pose3 pose{};
    Twist twist{};

    constexpr TrajectoryPoint() = default;

    constexpr TrajectoryPoint(Timestamp time_in, Pose3 pose_in, Twist twist_in)
        : time{time_in}, pose{pose_in}, twist{twist_in}
    {
    }
};

struct Trajectory
{
    std::vector<TrajectoryPoint> points{};

    bool empty() const noexcept { return this->points.empty(); }

    std::size_t size() const noexcept { return this->points.size(); }

    const TrajectoryPoint* first_point() const noexcept
    {
        if (this->empty())
        {
            return nullptr;
        }

        return &this->points.front();
    }

    const TrajectoryPoint* last_point() const noexcept
    {
        if (this->empty())
        {
            return nullptr;
        }

        return &this->points.back();
    }
};

}  // namespace autonomy_core
