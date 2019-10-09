#include "ros_spatial_utils/angles.h"

#include <gtest/gtest.h>

using namespace ros_spatial_utils;

TEST(TestUtils, normaliseAngle)
{
  EXPECT_FLOAT_EQ(0.f, normaliseAngle(0.f));
  EXPECT_DOUBLE_EQ(0.0, normaliseAngle(0.0));
  EXPECT_FLOAT_EQ(0, normaliseAngle(2 * M_PI));
  EXPECT_FLOAT_EQ(M_PI - 0.1, normaliseAngle(3 * M_PI - 0.1));
  EXPECT_FLOAT_EQ(0, normaliseAngle(4 * M_PI));
  EXPECT_FLOAT_EQ(0, normaliseAngle(-2 * M_PI));
  EXPECT_FLOAT_EQ(-M_PI + 0.1, normaliseAngle(-3 * M_PI + 0.1));
  EXPECT_FLOAT_EQ(0, normaliseAngle(-4 * M_PI));
  EXPECT_FLOAT_EQ(-M_PI + 2, normaliseAngle(M_PI + 2));
}

TEST(TestUtils, angleDelta)
{
  EXPECT_FLOAT_EQ(-0.1, angleDelta(0.1, 0.2));
  EXPECT_FLOAT_EQ(0.1, angleDelta(0.2, 0.1));

  EXPECT_FLOAT_EQ(-0.1, angleDelta(-0.2, -0.1));
  EXPECT_FLOAT_EQ(0.1, angleDelta(-0.1, -0.2));

  EXPECT_FLOAT_EQ(-0.2, angleDelta(-0.1, 0.1));
  EXPECT_FLOAT_EQ(0.2, angleDelta(0.1, -0.1));

  EXPECT_FLOAT_EQ(0.0, angleDelta(0.1, 0.1));
  EXPECT_FLOAT_EQ(0.0, angleDelta(-0.1, -0.1));

  EXPECT_FLOAT_EQ(-0.2, angleDelta(M_PI - 0.1, -M_PI + 0.1));
  EXPECT_FLOAT_EQ(0.2, angleDelta(-M_PI + 0.1, M_PI - 0.1));

  EXPECT_FLOAT_EQ(0.1, angleDelta(-M_PI, M_PI - 0.1));
  EXPECT_FLOAT_EQ(-0.1, angleDelta(-M_PI, -M_PI + 0.1));

  EXPECT_FLOAT_EQ(0.1, angleDelta(M_PI, M_PI - 0.1));
  EXPECT_FLOAT_EQ(-0.1, angleDelta(M_PI, -M_PI + 0.1));
}

TEST(TestUtils, radians)
{
  EXPECT_DOUBLE_EQ(M_PI / 2, radians<double>(90.0));
  EXPECT_FLOAT_EQ(-M_PI / 2, radians<float>(-90.0f));
  EXPECT_FLOAT_EQ(0.0f, radians<float>(0.0f));
}
