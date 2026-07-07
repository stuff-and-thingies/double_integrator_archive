#include <cassert>
#include <cstdlib>

#include "autonomy_core/time.hpp"
#include "test_utils.hpp"

using autonomy_core::Duration;
using autonomy_core::Timestamp;

int main(int argc, char* argv[])
{
    (void)argc;
    (void)argv;

    Timestamp t1{0.0};
    Timestamp t2{1.0};

    Duration dt = t2 - t1;
    assert(nearly_equal(dt.seconds, 1.0));

    Duration duration_test{2.0};
    assert(nearly_equal((t1 + duration_test).seconds, 2.0));

    Duration dur1{1.0};
    Duration dur2{2.0};
    assert(nearly_equal((dur1 += dur2).seconds, 3.0));
    assert(nearly_equal((dur2 *= 2.0).seconds, 4.0));

    return 0;
}
