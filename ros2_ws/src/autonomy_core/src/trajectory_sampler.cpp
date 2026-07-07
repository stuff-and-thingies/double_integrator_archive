#include "autonomy_trajectory/trajectory_sampler.hpp"

#include <algorithm>

namespace autonomy_trajectory
{

std::optional<autonomy_core::TrajectoryPoint> sample_trajectory(
    const autonomy_core::Trajectory& trajectory,
    const autonomy_core::Timestamp& query_time)
{
    if (trajectory.empty())
    {
        return std::nullopt;
    }

    if (query_time.seconds <= trajectory.first_point()->time.seconds)
    {
        return *trajectory.first_point();
    }

    if (query_time.seconds >= trajectory.last_point()->time.seconds)
    {
        return *trajectory.last_point();
    }

    auto iterator = std::lower_bound(
        trajectory.points.begin(), trajectory.points.end(), query_time,
        [](const autonomy_core::TrajectoryPoint& point,
           const autonomy_core::Timestamp& time)
        {
            // this lambda does this: while point.time.seconds is less than
            // query_time, keep searching
            // query_time is supplied to the "time" parameter of this lambda
            return point.time.seconds < time.seconds;
        });

    const autonomy_core::TrajectoryPoint& right_point = *iterator;
    const autonomy_core::TrajectoryPoint& left_point = *(iterator - 1);

    const double total_duration_s =
        right_point.time.seconds - left_point.time.seconds;

    const double elapsed_s = query_time.seconds - left_point.time.seconds;

    // alpha = 1 means use 100% of the right point
    // alpha = 0 means use 100% of the left point
    const double alpha = elapsed_s / total_duration_s;

    autonomy_core::TrajectoryPoint interpolated_point;

    interpolated_point.time = query_time;

    interpolated_point.pose.position =
        left_point.pose.position +
        alpha * (right_point.pose.position - left_point.pose.position);

    // orientation might need SLERP?
    interpolated_point.pose.orientation_rpy_rad =
        left_point.pose.orientation_rpy_rad +
        alpha * (right_point.pose.orientation_rpy_rad -
                 left_point.pose.orientation_rpy_rad);

    if (query_time.seconds == right_point.time.seconds)
    {
        interpolated_point.twist = right_point.twist;
    }
    else
    {
        interpolated_point.twist = left_point.twist;
    }

    return interpolated_point;
}

}  // namespace autonomy_trajectory
