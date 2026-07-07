#include <cassert>
#include <optional>

#include "autonomy_core/time.hpp"
#include "autonomy_core/trajectory.hpp"
#include "autonomy_trajectory/trajectory_sampler.hpp"
#include "test_utils.hpp"

using autonomy_core::Pose3;
using autonomy_core::Timestamp;
using autonomy_core::Trajectory;
using autonomy_core::TrajectoryPoint;
using autonomy_core::Twist;
using autonomy_core::Vector3;
using autonomy_trajectory::sample_trajectory;

bool trajectory_points_are_equal(const TrajectoryPoint& a,
                                 const TrajectoryPoint& b)
{
    bool time_check = nearly_equal(a.time.seconds, b.time.seconds);

    bool position_check = nearly_equal(a.pose.position.x, b.pose.position.x) &&
                          nearly_equal(a.pose.position.y, b.pose.position.y) &&
                          nearly_equal(a.pose.position.z, b.pose.position.z);

    bool orientation_check = nearly_equal(a.pose.orientation_rpy_rad.x,
                                          b.pose.orientation_rpy_rad.x) &&
                             nearly_equal(a.pose.orientation_rpy_rad.y,
                                          b.pose.orientation_rpy_rad.y) &&
                             nearly_equal(a.pose.orientation_rpy_rad.z,
                                          b.pose.orientation_rpy_rad.z);

    bool linear_velocity_check =
        nearly_equal(a.twist.linear.x, b.twist.linear.x) &&
        nearly_equal(a.twist.linear.y, b.twist.linear.y) &&
        nearly_equal(a.twist.linear.z, b.twist.linear.z);

    bool angular_velocity_check =
        nearly_equal(a.twist.angular.x, b.twist.angular.x) &&
        nearly_equal(a.twist.angular.y, b.twist.angular.y) &&
        nearly_equal(a.twist.angular.z, b.twist.angular.z);

    return time_check && position_check && orientation_check &&
           linear_velocity_check && angular_velocity_check;
}

void test_empty_trajectory()
{
    Trajectory trajectory{};
    std::optional<TrajectoryPoint> interpolated_point =
        sample_trajectory(trajectory, Timestamp{0.0});

    assert(interpolated_point.has_value() == false);
}

void test_query_before_first_point()
{
    Trajectory trajectory{};
    TrajectoryPoint point1{Timestamp{0.0}, Pose3{}, Twist{}};
    trajectory.points.emplace_back(point1);

    std::optional<TrajectoryPoint> interpolated_point =
        sample_trajectory(trajectory, Timestamp{-0.5});

    assert(interpolated_point.has_value());

    assert(trajectory_points_are_equal(interpolated_point.value(), point1));
}

void test_query_after_last_point()
{
    Trajectory trajectory{};
    TrajectoryPoint point1{Timestamp{0.0}, Pose3{}, Twist{}};
    trajectory.points.emplace_back(point1);

    std::optional<TrajectoryPoint> interpolated_point =
        sample_trajectory(trajectory, Timestamp{10.0});

    assert(interpolated_point.has_value());

    assert(trajectory_points_are_equal(interpolated_point.value(), point1));
}

void test_query_at_point()
{
    Trajectory trajectory{};
    TrajectoryPoint point1{Timestamp{0.0}, Pose3{},
                           Twist{Vector3{0.5, 0.0, 0.0}, Vector3{}}};
    TrajectoryPoint point2{Timestamp{2.0},
                           Pose3{Vector3{1.0, 0.0, 0.0}, Vector3{}}, Twist{}};

    trajectory.points.emplace_back(point1);
    trajectory.points.emplace_back(point2);

    std::optional<TrajectoryPoint> interpolated_point =
        sample_trajectory(trajectory, Timestamp{2.0});

    assert(interpolated_point.has_value());

    assert(trajectory_points_are_equal(interpolated_point.value(), point2));
}

void test_query_between_points()
{
    Trajectory trajectory{};
    TrajectoryPoint point1{Timestamp{0.0}, Pose3{},
                           Twist{Vector3{0.5, 0.0, 0.0}, Vector3{}}};
    TrajectoryPoint point2{Timestamp{2.0},
                           Pose3{Vector3{1.0, 0.0, 0.0}, Vector3{}}, Twist{}};

    trajectory.points.emplace_back(point1);
    trajectory.points.emplace_back(point2);

    std::optional<TrajectoryPoint> interpolated_point =
        sample_trajectory(trajectory, Timestamp{1.0});

    assert(interpolated_point.has_value());

    TrajectoryPoint actual_point{Timestamp{1.0},
                                 Pose3{Vector3{0.5, 0.0, 0.0}, Vector3{}},
                                 Twist{Vector3{0.5, 0.0, 0.0}, Vector3{}}};

    assert(
        trajectory_points_are_equal(interpolated_point.value(), actual_point));
}

int main(int argc, char* argv[])
{
    (void)argc;
    (void)argv;

    test_empty_trajectory();
    test_query_before_first_point();
    test_query_after_last_point();
    test_query_at_point();
    test_query_between_points();

    return 0;
}
