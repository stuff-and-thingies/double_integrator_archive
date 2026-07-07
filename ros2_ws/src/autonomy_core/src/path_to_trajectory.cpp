#include "autonomy_trajectory/path_to_trajectory.hpp"

#include <cstddef>
#include <stdexcept>
#include <vector>

#include "autonomy_core/pose3.hpp"
#include "autonomy_core/time.hpp"
#include "autonomy_core/twist.hpp"
#include "autonomy_core/vector3.hpp"

namespace
{

constexpr double duplicate_pose_distance_threshold_m = 1.0e-3;

[[nodiscard]] autonomy_core::Twist make_segment_twist(
    const autonomy_core::Pose3& from, const autonomy_core::Pose3& to,
    double speed_mps)
{
    autonomy_core::Twist twist;

    const autonomy_core::Vector3 segment = to.position - from.position;
    double distance = segment.norm();

    if (distance < 0.0001)
    {
        return twist;
    }

    const autonomy_core::Vector3 direction = segment / distance;

    twist.linear = direction * speed_mps;

    return twist;
}

std::vector<autonomy_core::Pose3> remove_consecutive_duplicate_poses(
    const autonomy_core::Path& path)
{
    std::vector<autonomy_core::Pose3> filtered;

    for (const autonomy_core::Pose3& pose : path.poses)
    {
        if (filtered.empty())
        {
            filtered.emplace_back(pose);
            continue;
        }

        const double distance = autonomy_core::euclidean_distance(
            pose.position, filtered.back().position);

        if (distance >= duplicate_pose_distance_threshold_m)
        {
            filtered.emplace_back(pose);
        }
    }

    return filtered;
}

}  // namespace

namespace autonomy_trajectory
{

autonomy_core::Trajectory path_to_constant_speed_trajectory(
    const autonomy_core::Path& path, double speed_mps)
{
    if (speed_mps <= 0.0)
    {
        throw std::invalid_argument("Speed must be greater than 0 m/s");
    }

    const autonomy_core::Path filtered_path{
        remove_consecutive_duplicate_poses(path)};

    autonomy_core::Trajectory trajectory;

    if (filtered_path.empty())
    {
        return trajectory;
    }

    if (filtered_path.size() == 1)
    {
        trajectory.points.emplace_back(
            autonomy_core::Timestamp{0.0}, filtered_path.poses.front(),
            autonomy_core::Twist{autonomy_core::Vector3{},
                                 autonomy_core::Vector3{}});
        return trajectory;
    }

    for (std::size_t path_pose_index = 0;
         path_pose_index < filtered_path.size(); ++path_pose_index)
    {
        autonomy_core::Timestamp time{0.0};

        autonomy_core::Twist twist;

        const autonomy_core::Pose3& current_path_pose =
            filtered_path.poses.at(path_pose_index);

        if (path_pose_index == 0)
        {
            const autonomy_core::Pose3& second_path_pose =
                filtered_path.poses.at(1);

            twist = make_segment_twist(current_path_pose, second_path_pose,
                                       speed_mps);
        }
        else
        {
            const autonomy_core::Pose3& previous_path_pose =
                filtered_path.poses.at(path_pose_index - 1);

            const autonomy_core::TrajectoryPoint& previous_trajectory_point =
                trajectory.points.at(path_pose_index - 1);

            const double distance = autonomy_core::euclidean_distance(
                current_path_pose.position, previous_path_pose.position);

            const double dt = distance / speed_mps;

            time.seconds = previous_trajectory_point.time.seconds + dt;

            if (path_pose_index != filtered_path.size() - 1)
            {
                const autonomy_core::Pose3& next_path_pose =
                    filtered_path.poses.at(path_pose_index + 1);

                twist = make_segment_twist(current_path_pose, next_path_pose,
                                           speed_mps);
            }
            else
            {
                twist = autonomy_core::Twist{};
            }
        }

        trajectory.points.emplace_back(time, current_path_pose, twist);
    }

    return trajectory;
}

}  // namespace autonomy_trajectory
