#ifndef LAWN_MOWER_TRACKER__VEHICLE_COMMAND_HPP_
#define LAWN_MOWER_TRACKER__VEHICLE_COMMAND_HPP_

#include <can_client/msg/vehicle_cmd.hpp>

namespace lawn_mower_tracker
{

can_client::msg::VehicleCmd make_drive_command(
  double linear_velocity, double steering_angle, double wheelbase,
  double max_angular_speed);

can_client::msg::VehicleCmd make_stop_command();

}  // namespace lawn_mower_tracker

#endif  // LAWN_MOWER_TRACKER__VEHICLE_COMMAND_HPP_
