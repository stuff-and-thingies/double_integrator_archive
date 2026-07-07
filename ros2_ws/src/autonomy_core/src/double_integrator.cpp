#include "autonomy_sim/double_integrator.hpp"

#include "autonomy_core/time.hpp"
#include "autonomy_core/vector3.hpp"
#include "autonomy_sim/fake_drone_state.hpp"

namespace autonomy_sim
{

DoubleIntegrator::DoubleIntegrator(DoubleIntegratorLimits limits)
    : limits_{limits}
{
}

void DoubleIntegrator::update(FakeDroneState& state,
                              const AccelerationCommand& accel_command,
                              autonomy_core::Duration dt) const
{
    AccelerationCommand legal_accel_command =
        clamp_accel_command_norm(accel_command);

    state.velocity += legal_accel_command.acceleration * dt.seconds;

    enforce_state_limits(state);

    state.position += state.velocity * dt.seconds;
}

void DoubleIntegrator::enforce_state_limits(FakeDroneState& state) const
{
    const double velocity_norm = state.velocity.norm();

    if (velocity_norm <= limits_.max_velocity_mps)
    {
        return;
    }

    const autonomy_core::Vector3 constrained_vel =
        state.velocity / velocity_norm * limits_.max_velocity_mps;

    state.velocity = constrained_vel;
}

AccelerationCommand DoubleIntegrator::clamp_accel_command_norm(
    const AccelerationCommand& accel_command) const
{
    AccelerationCommand legal_accel_command = accel_command;

    const double accel_command_norm = accel_command.acceleration.norm();

    if (accel_command_norm >= limits_.max_acceleration_mps2)
    {
        autonomy_core::Vector3 accel_component_ratio =
            accel_command.acceleration / accel_command_norm;

        legal_accel_command.acceleration =
            accel_component_ratio * limits_.max_acceleration_mps2;
    }

    return legal_accel_command;
}

}  // namespace autonomy_sim
