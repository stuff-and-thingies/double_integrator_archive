#pragma once

#include "autonomy_core/vector3.hpp"

namespace autonomy_sim
{

struct FakeDroneState
{
    autonomy_core::Vector3 position{};
    autonomy_core::Vector3 velocity{};

    constexpr FakeDroneState() = default;

    constexpr FakeDroneState(const autonomy_core::Vector3& position_init,
                             const autonomy_core::Vector3& velocity_init)
        : position{position_init}, velocity{velocity_init}
    {
    }
};

struct AccelerationCommand
{
    autonomy_core::Vector3 acceleration{};
};

}  // namespace autonomy_sim
