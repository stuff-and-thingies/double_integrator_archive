#pragma once
#include "autonomy_core/vector3.hpp"

namespace autonomy_core
{

struct Twist
{
    Vector3 linear{};
    Vector3 angular{};
};

}  // namespace autonomy_core
