#include <cassert>
#include <cmath>

#include "autonomy_core/vector2.hpp"
#include "test_utils.hpp"

using autonomy_core::dot;
using autonomy_core::Vector2;

int main(int argc, char* argv[])
{
    (void)argc;
    (void)argv;

    const Vector2 a{1.0, 2.0};
    const Vector2 b{3.0, 4.0};

    const Vector2 sum = a + b;
    assert(nearly_equal(sum.x, 4.0));
    assert(nearly_equal(sum.y, 6.0));

    const Vector2 diff = a - b;
    assert(nearly_equal(diff.x, -2.0));
    assert(nearly_equal(diff.y, -2.0));

    const Vector2 scaled = 2.0 * a;
    assert(nearly_equal(scaled.x, 2.0));
    assert(nearly_equal(scaled.y, 4.0));

    assert(nearly_equal(a.squared_norm(), 5.0));
    assert(nearly_equal(Vector2{3.0, 4.0}.norm(), 5.0));

    assert(nearly_equal(dot(a, b), 11.0));

    return 0;
}
