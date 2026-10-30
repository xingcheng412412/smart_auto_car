#pragma once

#include <sensor_msgs/msg/point_cloud2.hpp>

namespace fw_mini_pro_slam3d {

sensor_msgs::msg::PointCloud2 convert_cloud(
  const sensor_msgs::msg::PointCloud2 & input,
  double max_scan_duration = 0.2,
  int scan_lines = 128);

}  // namespace fw_mini_pro_slam3d
