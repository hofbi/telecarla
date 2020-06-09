#include "linear_function.h"

#include <cmath>
#include <gtest/gtest.h>

using namespace lmt::math;

struct LinearFunctionFromGradientAndYIntercept_params
{
    double gradient;
    double yIntercept;
    double functionValue;
    double expectedValue;
};

class LinearFunctionTestFromGradientAndYIntercept
    : public ::testing::TestWithParam<LinearFunctionFromGradientAndYIntercept_params>
{
};

TEST_P(LinearFunctionTestFromGradientAndYIntercept, constructor_fromGradientAndYIntercept_evaluateCorrect)
{
    const auto& params = GetParam();
    LinearFunction unit(params.gradient, params.yIntercept);

    auto actualValue = unit(params.functionValue);

    EXPECT_EQ(params.expectedValue, actualValue);
}

INSTANTIATE_TEST_CASE_P(LinearFunctionTest,
                        LinearFunctionTestFromGradientAndYIntercept,
                        ::testing::Values(LinearFunctionFromGradientAndYIntercept_params{0.0, 0.0, 0.0, 0.0},
                                          LinearFunctionFromGradientAndYIntercept_params{0.0, 0.0, 1.0, 0.0},
                                          LinearFunctionFromGradientAndYIntercept_params{1.0, 0.0, 1.0, 1.0},
                                          LinearFunctionFromGradientAndYIntercept_params{0.0, 1.0, 0.0, 1.0},
                                          LinearFunctionFromGradientAndYIntercept_params{1.0, 1.0, 3.0, 4.0},
                                          LinearFunctionFromGradientAndYIntercept_params{-1.0, 0.0, 1.0, -1.0}));

struct LinearFunctionFromPoint_params
{
    std::pair<double, double> a;
    std::pair<double, double> b;
    double functionValue;
    double expectedValue;
};

class LinearFunctionTestFromTwoPoints : public ::testing::TestWithParam<LinearFunctionFromPoint_params>
{
};

TEST_P(LinearFunctionTestFromTwoPoints, constructor_fromTwoPoints_evaluateCorrect)
{
    const auto& params = GetParam();
    LinearFunction unit(params.a, params.b);

    auto actualValue = unit(params.functionValue);

    EXPECT_EQ(params.expectedValue, actualValue);
}

INSTANTIATE_TEST_CASE_P(
    LinearFunctionTest,
    LinearFunctionTestFromTwoPoints,
    ::testing::Values(LinearFunctionFromPoint_params{std::make_pair(0.0, 0.0), std::make_pair(1.0, 1.0), 0.0, 0.0},
                      LinearFunctionFromPoint_params{std::make_pair(0.0, 0.0), std::make_pair(1.0, 0.0), 1.0, 0.0},
                      LinearFunctionFromPoint_params{std::make_pair(0.0, 0.0), std::make_pair(1.0, 1.0), 1.0, 1.0}));

TEST(LinearFunctionTest, constructor_fromTwoPoints_isNaN)
{
    LinearFunction unit(std::make_pair(0.0, 0.0), std::make_pair(0.0, 1.0));

    auto actualValue = unit(0.0);

    EXPECT_TRUE(std::isnan(actualValue));
}