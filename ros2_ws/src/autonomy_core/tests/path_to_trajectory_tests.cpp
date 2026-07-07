#include <cassert>
#include <cstddef>
#include <stdexcept>

#include "autonomy_core/path.hpp"
#include "autonomy_trajectory/path_to_trajectory.hpp"
#include "test_utils.hpp"

using autonomy_core::euclidean_distance;
using autonomy_core::Path;
using autonomy_core::Pose3;
using autonomy_core::Trajectory;
using autonomy_core::Vector3;
using autonomy_trajectory::path_to_constant_speed_trajectory;

void test_invalid_speed()
{
    Path path{};

    path.poses.emplace_back(Pose3{Vector3{0.0, 0.0, 0.0}, Vector3{}});
    path.poses.emplace_back(Pose3{Vector3{5.0, 5.0, 0.0}, Vector3{}});
    path.poses.emplace_back(Pose3{Vector3{10.0, 10.0, 0.0}, Vector3{}});

    bool neg_threw = false;
    bool zero_threw = false;

    try
    {
        Trajectory trajectory1 = path_to_constant_speed_trajectory(path, -1.0);
        (void)trajectory1;
    }
    catch (const std::invalid_argument&)
    {
        neg_threw = true;
    }

    try
    {
        Trajectory trajectory2 = path_to_constant_speed_trajectory(path, 0.0);
        (void)trajectory2;
    }
    catch (const std::invalid_argument&)
    {
        zero_threw = true;
    }

    assert(neg_threw);
    assert(zero_threw);
}

void test_empty_path()
{
    Path path{};

    Trajectory trajecotry = path_to_constant_speed_trajectory(path, 1.0);

    assert(trajecotry.empty());
}

void test_one_point_path()
{
    Path path{};
    Vector3 position1{1.0, 2.0, 0.0};
    path.poses.emplace_back(Pose3{position1, Vector3{}});

    const Trajectory trajectory = path_to_constant_speed_trajectory(path, 1.0);

    assert(trajectory.size() == 1);
    assert(nearly_equal(trajectory.first_point()->time.seconds, 0.0));
    assert(trajectory.first_point()->pose.position == position1);
}

void test_two_points_path()
{
    Path path{};
    Vector3 position1{0.0, 0.0, 0.0};
    Vector3 position2{1.0, 0.0, 0.0};

    path.poses.emplace_back(position1, Vector3{});
    path.poses.emplace_back(position2, Vector3{});

    const Trajectory trajectory = path_to_constant_speed_trajectory(path, 0.5);

    assert(trajectory.size() == 2);

    assert(nearly_equal(trajectory.first_point()->time.seconds, 0.0));
    assert(nearly_equal(trajectory.last_point()->time.seconds, 2.0));

    assert(nearly_equal(trajectory.first_point()->twist.linear.x, 0.5));
    assert(nearly_equal(trajectory.first_point()->twist.linear.y, 0.0));
    assert(nearly_equal(trajectory.first_point()->twist.linear.z, 0.0));

    assert(nearly_equal(trajectory.last_point()->twist.linear.x, 0.0));
    assert(nearly_equal(trajectory.last_point()->twist.linear.y, 0.0));
    assert(nearly_equal(trajectory.last_point()->twist.linear.z, 0.0));
}

void test_three_points_path()
{
    Path path{};
    Vector3 position1{0.0, 0.0, 0.0};
    Vector3 position2{3.0, 0.0, 0.0};
    Vector3 position3{3.0, 4.0, 0.0};

    double speed_mps = 1.0;

    path.poses.emplace_back(position1, Vector3{});
    path.poses.emplace_back(position2, Vector3{});
    path.poses.emplace_back(position3, Vector3{});

    const Trajectory trajectory =
        path_to_constant_speed_trajectory(path, speed_mps);

    assert(trajectory.size() == 3);

    double distance1 =
        euclidean_distance(trajectory.points.at(0).pose.position,
                           trajectory.points.at(1).pose.position);
    double distance2 =
        euclidean_distance(trajectory.points.at(1).pose.position,
                           trajectory.points.at(2).pose.position);

    assert(nearly_equal(distance1, 3.0));
    assert(nearly_equal(distance2, 4.0));

    assert(nearly_equal(trajectory.points.at(0).time.seconds, 0.0));
    assert(nearly_equal(trajectory.points.at(1).time.seconds, 3.0));
    assert(nearly_equal(trajectory.points.at(2).time.seconds, 7.0));

    assert(nearly_equal(trajectory.points.at(0).twist.linear.x, 1.0));
    assert(nearly_equal(trajectory.points.at(0).twist.linear.y, 0.0));

    assert(nearly_equal(trajectory.points.at(1).twist.linear.x, 0.0));
    assert(nearly_equal(trajectory.points.at(1).twist.linear.y, 1.0));

    assert(nearly_equal(trajectory.points.at(2).twist.linear.x, 0.0));
    assert(nearly_equal(trajectory.points.at(2).twist.linear.y, 0.0));
}

void test_duplicate_points_path()
{
    Path path{};
    Vector3 position1{0.0, 0.0, 0.0};
    Vector3 position2{3.0, 0.0, 0.0};
    Vector3 position3{3.0, 0.0, 0.0};

    double speed_mps = 1.0;

    path.poses.emplace_back(position1, Vector3{});
    path.poses.emplace_back(position2, Vector3{});
    path.poses.emplace_back(position3, Vector3{});

    const Trajectory trajectory =
        path_to_constant_speed_trajectory(path, speed_mps);

    assert(trajectory.size() == 2);

    double distance1 =
        euclidean_distance(trajectory.points.at(0).pose.position,
                           trajectory.points.at(1).pose.position);

    assert(nearly_equal(distance1, 3.0));

    assert(nearly_equal(trajectory.points.at(0).time.seconds, 0.0));
    assert(nearly_equal(trajectory.points.at(1).time.seconds, 3.0));

    assert(nearly_equal(trajectory.points.at(0).twist.linear.x, 1.0));
    assert(nearly_equal(trajectory.points.at(0).twist.linear.y, 0.0));

    assert(nearly_equal(trajectory.points.at(1).twist.linear.x, 0.0));
    assert(nearly_equal(trajectory.points.at(1).twist.linear.y, 0.0));
}

void test_duplicate_middle_point_is_skipped()
{
    Path path{};

    path.poses.emplace_back(Vector3{0.0, 0.0, 0.0}, Vector3{});
    path.poses.emplace_back(Vector3{3.0, 0.0, 0.0}, Vector3{});
    path.poses.emplace_back(Vector3{3.0, 0.0, 0.0}, Vector3{});
    path.poses.emplace_back(Vector3{3.0, 4.0, 0.0}, Vector3{});

    const Trajectory trajectory = path_to_constant_speed_trajectory(path, 1.0);

    assert(trajectory.size() == 3);

    assert(nearly_equal(trajectory.points.at(0).time.seconds, 0.0));
    assert(nearly_equal(trajectory.points.at(1).time.seconds, 3.0));
    assert(nearly_equal(trajectory.points.at(2).time.seconds, 7.0));

    assert(nearly_equal(trajectory.points.at(0).twist.linear.x, 1.0));
    assert(nearly_equal(trajectory.points.at(0).twist.linear.y, 0.0));

    assert(nearly_equal(trajectory.points.at(1).twist.linear.x, 0.0));
    assert(nearly_equal(trajectory.points.at(1).twist.linear.y, 1.0));

    assert(nearly_equal(trajectory.points.at(2).twist.linear.x, 0.0));
    assert(nearly_equal(trajectory.points.at(2).twist.linear.y, 0.0));
}

void test_duplicate_start_point_is_skipped()
{
    Path path{};

    path.poses.emplace_back(Vector3{0.0, 0.0, 0.0}, Vector3{});
    path.poses.emplace_back(Vector3{0.0, 0.0, 0.0}, Vector3{});
    path.poses.emplace_back(Vector3{1.0, 0.0, 0.0}, Vector3{});

    const Trajectory trajectory = path_to_constant_speed_trajectory(path, 1.0);

    assert(trajectory.size() == 2);
    assert(nearly_equal(trajectory.points.at(0).time.seconds, 0.0));
    assert(nearly_equal(trajectory.points.at(1).time.seconds, 1.0));

    assert(nearly_equal(trajectory.points.at(0).twist.linear.x, 1.0));
    assert(nearly_equal(trajectory.points.at(1).twist.linear.x, 0.0));
}

void test_multiple_duplicate_points()
{
    Path path{};

    path.poses.emplace_back(Vector3{0.0, 0.0, 0.0}, Vector3{});
    path.poses.emplace_back(Vector3{0.0, 0.0, 0.0}, Vector3{});
    path.poses.emplace_back(Vector3{0.0, 0.0, 0.0}, Vector3{});

    const Trajectory trajectory = path_to_constant_speed_trajectory(path, 1.0);

    assert(trajectory.size() == 1);
    assert(nearly_equal(trajectory.points.at(0).time.seconds, 0.0));

    assert(nearly_equal(trajectory.points.at(0).twist.linear.x, 0.0));
}

int main(int argc, char* argv[])
{
    (void)argc;
    (void)argv;

    test_invalid_speed();
    test_empty_path();
    test_one_point_path();
    test_two_points_path();
    test_three_points_path();
    test_duplicate_points_path();
    test_duplicate_middle_point_is_skipped();
    test_duplicate_start_point_is_skipped();
    test_multiple_duplicate_points();

    return 0;
}
