#include <gtest/gtest.h>

#include "pca9685_hardware_interface/servo_mapping.hpp"

using pca9685_hardware_interface::ServoConfig;
using pca9685_hardware_interface::servo_pulse_us;

namespace
{
// The steering joint as written in description.urdf.xacro: measured lock 0.575 rad.
ServoConfig steering()
{
  ServoConfig c;
  c.min_angle = -0.575;
  c.max_angle = 0.575;
  c.offset = 0.0;
  c.min_pulse_us = 1000.0;
  c.neutral_pulse_us = 1500.0;
  c.max_pulse_us = 2000.0;
  return c;
}
}  // namespace

TEST(ServoMapping, ZeroAngleIsNeutralPulse)
{
  EXPECT_DOUBLE_EQ(servo_pulse_us(0.0, steering()), 1500.0);
}

TEST(ServoMapping, FullLockReachesPulseEndpoints)
{
  EXPECT_DOUBLE_EQ(servo_pulse_us(0.575, steering()), 2000.0);
  EXPECT_DOUBLE_EQ(servo_pulse_us(-0.575, steering()), 1000.0);
}

TEST(ServoMapping, LinearBetweenNeutralAndLock)
{
  EXPECT_NEAR(servo_pulse_us(0.2875, steering()), 1750.0, 1e-9);
  EXPECT_NEAR(servo_pulse_us(-0.2875, steering()), 1250.0, 1e-9);
}

TEST(ServoMapping, BeyondLockClampsToEndpoint)
{
  EXPECT_DOUBLE_EQ(servo_pulse_us(0.785, steering()), 2000.0);
  EXPECT_DOUBLE_EQ(servo_pulse_us(-3.0, steering()), 1000.0);
}

TEST(ServoMapping, OffsetTrimsBeforeClamping)
{
  ServoConfig c = steering();
  c.offset = 0.05;
  EXPECT_NEAR(servo_pulse_us(0.0, c), 1500.0 + (0.05 / 0.575) * 500.0, 1e-9);
  EXPECT_DOUBLE_EQ(servo_pulse_us(0.575, c), 2000.0);  // clamped after the trim
}

TEST(ServoMapping, AsymmetricLockAndOffCentreNeutral)
{
  ServoConfig c;
  c.min_angle = -0.4;
  c.max_angle = 0.6;
  c.min_pulse_us = 1100.0;
  c.neutral_pulse_us = 1450.0;
  c.max_pulse_us = 1950.0;
  EXPECT_DOUBLE_EQ(servo_pulse_us(0.0, c), 1450.0);
  EXPECT_DOUBLE_EQ(servo_pulse_us(0.6, c), 1950.0);
  EXPECT_DOUBLE_EQ(servo_pulse_us(-0.4, c), 1100.0);
  EXPECT_NEAR(servo_pulse_us(0.3, c), 1700.0, 1e-9);
  EXPECT_NEAR(servo_pulse_us(-0.2, c), 1275.0, 1e-9);
}
