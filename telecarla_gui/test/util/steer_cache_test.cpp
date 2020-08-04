#include "steer_cache.h"

#include <gtest/gtest.h>

using namespace lmt::util;

TEST(SteerCacheTest, constructor_default_valueIsZero)
{
    SteerCache unit;

    EXPECT_EQ(0, unit.get());
}

TEST(SteerCacheTest, left_turnLeftOnce_valueLessThanZero)
{
    SteerCache unit;

    unit.left();

    EXPECT_LT(unit.get(), 0);
}

TEST(SteerCacheTest, right_rightLeftOnce_valueGreaterThanZero)
{
    SteerCache unit;

    unit.right();

    EXPECT_GT(unit.get(), 0);
}

TEST(SteerCacheTest, left_turnLeftTwice_valueLessThanBefore)
{
    SteerCache unit;

    unit.left();
    const auto old = unit.get();
    unit.left();

    EXPECT_LT(unit.get(), old);
}

TEST(SteerCacheTest, right_turnRightTwice_valueGreaterThanBefore)
{
    SteerCache unit;

    unit.right();
    const auto old = unit.get();
    unit.right();

    EXPECT_GT(unit.get(), old);
}

TEST(SteerCacheTest, set_setToOne_valueEqualToMaxCache)
{
    SteerCache unit;

    unit.set(1);

    EXPECT_DOUBLE_EQ(0.7, unit.get());
}

TEST(SteerCacheTest, set_setToMinusOne_valueEqualToMinCache)
{
    SteerCache unit;

    unit.set(-1);

    EXPECT_DOUBLE_EQ(-0.7, unit.get());
}

TEST(SteerCacheTest, reset_valueIsZero)
{
    SteerCache unit;
    unit.set(0.5);  // NOLINT(readability-magic-numbers)

    unit.reset();

    EXPECT_EQ(0, unit.get());
}
