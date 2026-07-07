#include <cassert>

#include "autonomy_core/time.hpp"
#include "autonomy_core/trajectory.hpp"
#include "autonomy_core/vector3.hpp"
#include "test_utils.hpp"

using autonomy_core::Timestamp;
using autonomy_core::Trajectory;
using autonomy_core::TrajectoryPoint;
using autonomy_core::Vector3;

int main(int argc, char* argv[])
{
    (void)argc;
    (void)argv;

    TrajectoryPoint traj_point0;
    TrajectoryPoint traj_point1;
    traj_point1.time = Timestamp{1.0};
    traj_point1.pose.position = Vector3{1.0, 1.0, 1.0};

    Trajectory traj_test;
    assert(traj_test.empty() == true);
    assert(traj_test.size() == 0);

    assert(traj_test.first_point() == nullptr);
    assert(traj_test.last_point() == nullptr);

    traj_test.points.emplace_back(traj_point0);
    traj_test.points.emplace_back(traj_point1);

    assert(traj_test.empty() == false);
    assert(traj_test.size() == 2);

    assert(nearly_equal(traj_test.first_point()->time.seconds, 0.0));
    assert(nearly_equal(traj_test.last_point()->pose.position.x, 1.0));

    return 0;
}
