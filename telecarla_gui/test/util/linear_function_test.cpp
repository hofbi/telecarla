#include "linear_function.h"

#include <cmath>

#include <gtest/gtest.h>

using namespace lmt::util;

TEST(LinearFunctionTest, constructor_fromTwoPoints_isNaN)
{
    LinearFunction unit(std::make_pair(0.0, 0.0), std::make_pair(0.0, 1.0));

    const auto actualValue = unit(0.0);

    EXPECT_TRUE(std::isnan(actualValue));
}
