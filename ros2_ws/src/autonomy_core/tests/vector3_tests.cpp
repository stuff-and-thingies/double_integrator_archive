#include <cassert>
#include <cmath>

#include "autonomy_core/vector3.hpp"
#include "test_utils.hpp"

using autonomy_core::dot;
using autonomy_core::Vector3;

int main(int argc, char* argv[])
{
    (void)argc;
    (void)argv;

    const Vector3 a{1.0, 2.0, 3.0};
    const Vector3 b{2.0, 3.0, 4.0};

    const Vector3 sum = a + b;
    assert(nearly_equal(sum.x, 3.0));
    assert(nearly_equal(sum.y, 5.0));
    assert(nearly_equal(sum.z, 7.0));

    const Vector3 diff = a - b;
    assert(nearly_equal(diff.x, -1.0));
    assert(nearly_equal(diff.y, -1.0));
    assert(nearly_equal(diff.z, -1.0));

    const Vector3 scaled = a * 2.0;
    assert(nearly_equal(scaled.x, 2.0));
    assert(nearly_equal(scaled.y, 4.0));
    assert(nearly_equal(scaled.z, 6.0));

    assert(nearly_equal(a.squared_norm(), 14.0));
    assert(nearly_equal(Vector3{1.0, 2.0, 3.0}.norm(), std::sqrt(14.0)));

    assert(nearly_equal(dot(a, b), 20.0));

    return 0;
}
