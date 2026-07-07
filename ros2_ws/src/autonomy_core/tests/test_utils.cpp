#include "test_utils.hpp"

#include <cmath>

bool nearly_equal(double a, double b, double eps /* = 1e-12 */)
{
  return std::abs(a - b) < eps;
}
