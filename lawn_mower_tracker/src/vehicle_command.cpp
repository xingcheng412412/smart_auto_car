#include "lawn_mower_tracker/vehicle_command.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace lawn_mower_tracker
{

can_client::msg::VehicleCmd make_stop_command()
{
  can_client::msg::VehicleCmd command;
  command.enable = false;
  command.gear = can_client::msg::VehicleCmd::GEAR_DISABLE;
  command.linear_x = 0.0F;
  command.linear_y = 0.0F;
  command.angular_z = 0.0F;
  return command;
}

can_client::msg::VehicleCmd make_drive_command(
  double linear_velocity, double steering_angle, double wheelbase,
  double max_angular_speed)
{
  if (!std::isfinite(linear_velocity) || !std::isfinite(steering_angle) ||
    !std::isfinite(wheelbase) || !std::isfinite(max_angular_speed) ||
    wheelbase <= 0.0 || max_angular_speed <= 0.0)
  {
    throw std::invalid_argument("invalid vehicle command input");
  }

  const double yaw_rate = linear_velocity * std::tan(steering_angle) / wheelbase;
  if (!std::isfinite(yaw_rate)) {
    throw std::invalid_argument("vehicle yaw rate is not finite");
  }

  can_client::msg::VehicleCmd command;
  command.enable = true;
  command.gear = can_client::msg::VehicleCmd::GEAR_4WD4WS;
  command.linear_x = static_cast<float>(linear_velocity);
  command.linear_y = 0.0F;
  command.angular_z = static_cast<float>(
    std::clamp(yaw_rate, -max_angular_speed, max_angular_speed));
  return command;
}

}  // namespace lawn_mower_tracker
