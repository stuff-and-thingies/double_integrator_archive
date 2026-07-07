#include <cassert>

#include "autonomy_core/time.hpp"
#include "autonomy_core/vector3.hpp"
#include "autonomy_sim/double_integrator.hpp"
#include "autonomy_sim/fake_drone_state.hpp"
#include "test_utils.hpp"

using autonomy_core::Duration;
using autonomy_core::Vector3;
using autonomy_sim::AccelerationCommand;
using autonomy_sim::DoubleIntegrator;
using autonomy_sim::DoubleIntegratorLimits;
using autonomy_sim::FakeDroneState;

int main(int argc, char* argv[])
{
    (void)argc;
    (void)argv;

    const double max_accel_mps2 = 1.0;
    const double max_vel_mps = 10.0;
    DoubleIntegrator model{DoubleIntegratorLimits{max_accel_mps2, max_vel_mps}};
    FakeDroneState state{};
    AccelerationCommand command{};

    double t_end = 2.0;
    double dt = 0.001;
    const int num_steps = static_cast<int>(t_end / dt);

    // initial condition at origin
    assert(nearly_equal(state.position.x, 0.0));
    assert(nearly_equal(state.position.y, 0.0));
    assert(nearly_equal(state.position.z, 0.0));
    assert(nearly_equal(state.velocity.x, 0.0));
    assert(nearly_equal(state.velocity.y, 0.0));
    assert(nearly_equal(state.velocity.z, 0.0));

    for (int curr_step = 0; curr_step < num_steps; ++curr_step)
    {
        model.update(state, command, Duration{dt});
    }

    // should not move from origin with zero accel command
    assert(nearly_equal(state.position.x, 0.0));
    assert(nearly_equal(state.position.y, 0.0));
    assert(nearly_equal(state.position.z, 0.0));
    assert(nearly_equal(state.velocity.x, 0.0));
    assert(nearly_equal(state.velocity.y, 0.0));
    assert(nearly_equal(state.velocity.z, 0.0));

    Vector3 origin_pos{0.0, 0.0, 0.0};
    Vector3 init_vel{1.0, 1.0, 1.0};

    FakeDroneState zero_accel_state{origin_pos, init_vel};

    for (int curr_step = 0; curr_step < num_steps; ++curr_step)
    {
        model.update(zero_accel_state, command, Duration{dt});
    }

    // zero accel command should not change initial velocity
    assert(nearly_equal(zero_accel_state.velocity.x, 1.0));
    assert(nearly_equal(zero_accel_state.velocity.y, 1.0));
    assert(nearly_equal(zero_accel_state.velocity.z, 1.0));

    // positive initial velocity should lead to positive change in position
    assert(zero_accel_state.position.x > 0.0);
    assert(zero_accel_state.position.y > 0.0);
    assert(zero_accel_state.position.z > 0.0);

    // reset state to make sure
    state.velocity *= 0.0;
    state.position *= 0.0;

    // provide positive acceleration
    command.acceleration.x = 1.0;
    command.acceleration.y = 1.0;
    command.acceleration.z = 1.0;

    for (int curr_step = 0; curr_step < num_steps; ++curr_step)
    {
        model.update(state, command, Duration{dt});
    }

    // positive accel command should increase velocity
    assert(state.velocity.x > 0.0);
    assert(state.velocity.y > 0.0);
    assert(state.velocity.z > 0.0);

    // reset state
    state.velocity *= 0.0;
    state.position *= 0.0;

    command.acceleration *= 0.0;

    state.velocity.x = 100.0;
    model.update(state, command, Duration{dt});

    assert(nearly_equal(state.velocity.x, max_vel_mps));

    DoubleIntegrator accel_lim_model{DoubleIntegratorLimits{1.0, 10.0}};

    // reset state
    state.velocity *= 0.0;
    state.position *= 0.0;

    command.acceleration.x = 2.0;

    for (int curr_step = 0; curr_step < num_steps; ++curr_step)
    {
        accel_lim_model.update(state, command, Duration{dt});
    }

    assert(nearly_equal(state.velocity.x, 2.0, 0.001));

    return 0;
}
