#pragma once

#include "autonomy_core/time.hpp"
#include "autonomy_sim/fake_drone_state.hpp"

namespace autonomy_sim
{

struct DoubleIntegratorLimits
{
    double max_acceleration_mps2{5.0};
    double max_velocity_mps{10.0};
};

class DoubleIntegrator
{
   public:
    explicit DoubleIntegrator(DoubleIntegratorLimits limits = {});

    void update(FakeDroneState& state, const AccelerationCommand& accel_command,
                autonomy_core::Duration dt) const;

   private:
    void enforce_state_limits(FakeDroneState& state) const;

    AccelerationCommand clamp_accel_command_norm(
        const AccelerationCommand& accel_command) const;

    DoubleIntegratorLimits limits_{};
};

}  // namespace autonomy_sim
