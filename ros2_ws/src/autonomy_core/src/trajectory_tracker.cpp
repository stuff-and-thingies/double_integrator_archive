#include "autonomy_control/trajectory_tracker.hpp"

#include <stdexcept>

namespace autonomy_control
{

TrajectoryTracker::TrajectoryTracker(TrajectoryTrackerGains gains)
    : gains_(gains)
{
    if (gains_.kp_position < 0.0)
    {
        throw std::invalid_argument("kp_position must be non-negative");
    }

    if (gains_.kd_velocity < 0.0)
    {
        throw std::invalid_argument("kd_velocity must be non-negative");
    }

    if (gains_.max_acceleration_mps2 <= 0.0)
    {
        throw std::invalid_argument("max_acceleration_mps2 must be positive");
    }
}

autonomy_sim::AccelerationCommand TrajectoryTracker::compute_command(
    const autonomy_sim::FakeDroneState& state,
    const autonomy_core::TrajectoryPoint& reference) const
{
    autonomy_sim::AccelerationCommand accel_command;

    const autonomy_core::Vector3 position_error =
        reference.pose.position - state.position;
    const autonomy_core::Vector3 velocity_error =
        reference.twist.linear - state.velocity;

    accel_command.acceleration = position_error * gains_.kp_position +
                                 velocity_error * gains_.kd_velocity;

    const double accel_command_norm = accel_command.acceleration.norm();

    if (accel_command_norm > gains_.max_acceleration_mps2)
    {
        const autonomy_core::Vector3 accel_component_ratio =
            accel_command.acceleration / accel_command_norm;

        accel_command.acceleration =
            accel_component_ratio * gains_.max_acceleration_mps2;
    }

    return accel_command;
}

}  // namespace autonomy_control
