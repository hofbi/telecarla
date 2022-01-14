#include "linear_function.h"

namespace
{
using namespace lmt::util;

// Linear function from gradient
static_assert(LinearFunction(0.0, 0.0)(0.0) == 0.0);
static_assert(LinearFunction(0.0, 0.0)(1.0) == 0.0);
static_assert(LinearFunction(1.0, 0.0)(1.0) == 1.0);
static_assert(LinearFunction(0.0, 1.0)(0.0) == 1.0);
static_assert(LinearFunction(1.0, 1.0)(3.0) == 4.0);  // NOLINT(readability-magic-numbers)
static_assert(LinearFunction(-1.0, 0.0)(1.0) == -1.0);

// Linear function from points
static_assert(LinearFunction(std::make_pair(0.0, 0.0), std::make_pair(1.0, 1.0))(0.0) == 0.0);
static_assert(LinearFunction(std::make_pair(0.0, 0.0), std::make_pair(1.0, 0.0))(0.0) == 0.0);
static_assert(LinearFunction(std::make_pair(0.0, 0.0), std::make_pair(1.0, 0.0))(1.0) == 0.0);
static_assert(LinearFunction(std::make_pair(0.0, 0.0), std::make_pair(1.0, 1.0))(1.0) == 1.0);
}  // namespace
