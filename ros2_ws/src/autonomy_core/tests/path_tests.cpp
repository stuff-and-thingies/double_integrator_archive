#include <cassert>
#include <cmath>

#include "autonomy_core/path.hpp"
#include "autonomy_core/vector3.hpp"
#include "test_utils.hpp"

using autonomy_core::Path;
using autonomy_core::Pose3;
using autonomy_core::Vector3;

int main(int argc, char* argv[])
{
    (void)argc;
    (void)argv;

    Pose3 pose0;
    Pose3 pose1{Vector3{1.0, 1.0, 1.0}, Vector3{}};
    Pose3 pose2{Vector3{2.0, 2.0, 2.0}, Vector3{}};

    Path path_test;

    assert(path_test.empty() == true);
    assert(path_test.size() == 0);

    path_test.poses.emplace_back(pose0);
    path_test.poses.emplace_back(pose1);
    path_test.poses.emplace_back(pose2);

    assert(path_test.size() == 3);
    assert(nearly_equal(path_test.length(), std::sqrt(3.0) * 2.0));

    return 0;
}
