#include <gtest/gtest.h>

#include <cmath>
#include <stdexcept>

#include "lawn_mower_tracker/vehicle_command.hpp"

namespace {

TEST(VehicleCommandTest, StopCommandIsDisabledAndZero)
{
  const auto command = lawn_mower_tracker::make_stop_command();
  EXPECT_FALSE(command.enable);
  EXPECT_EQ(command.gear, 0U);
  EXPECT_FLOAT_EQ(command.linear_x, 0.0F);
  EXPECT_FLOAT_EQ(command.linear_y, 0.0F);
  EXPECT_FLOAT_EQ(command.angular_z, 0.0F);
}

TEST(VehicleCommandTest, ConvertsBicycleSteeringToYawRate)
{
  const auto command = lawn_mower_tracker::make_drive_command(1.0, 0.2, 1.45, 3.0);
  EXPECT_TRUE(command.enable);
  EXPECT_EQ(command.gear, 6U);
  EXPECT_FLOAT_EQ(command.linear_x, 1.0F);
  EXPECT_FLOAT_EQ(command.linear_y, 0.0F);
  EXPECT_NEAR(command.angular_z, std::tan(0.2) / 1.45, 1e-6);
}

TEST(VehicleCommandTest, PreservesReverseYawSignAndClamps)
{
  const auto reverse = lawn_mower_tracker::make_drive_command(-1.0, 0.2, 1.45, 3.0);
  EXPECT_LT(reverse.angular_z, 0.0F);
  const auto clamped = lawn_mower_tracker::make_drive_command(3.0, 1.0, 1.0, 0.5);
  EXPECT_FLOAT_EQ(clamped.angular_z, 0.5F);
}

TEST(VehicleCommandTest, RejectsInvalidInputs)
{
  EXPECT_THROW(
    lawn_mower_tracker::make_drive_command(NAN, 0.0, 1.45, 3.0), std::invalid_argument);
  EXPECT_THROW(
    lawn_mower_tracker::make_drive_command(1.0, 0.0, 0.0, 3.0), std::invalid_argument);
}

}  // namespace
