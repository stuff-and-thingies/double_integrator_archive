#pragma once

#include "autonomy_core/trajectory.hpp"
#include "autonomy_sim/fake_drone_state.hpp"

namespace autonomy_control
{

struct TrajectoryTrackerGains
{
    double kp_position{1.0};
    double kd_velocity{2.0};
    double max_acceleration_mps2{5.0};
};

class TrajectoryTracker
{
   public:
    explicit TrajectoryTracker(TrajectoryTrackerGains gains = {});

    [[nodiscard]] autonomy_sim::AccelerationCommand compute_command(
        const autonomy_sim::FakeDroneState& state,
        const autonomy_core::TrajectoryPoint& reference) const;

   private:
    TrajectoryTrackerGains gains_{};
};

}  // namespace autonomy_control
