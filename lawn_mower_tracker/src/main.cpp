#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "lawn_mower_tracker/tracker_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<lawn_mower_tracker::TrackerNode>());
  rclcpp::shutdown();
  return 0;
}
