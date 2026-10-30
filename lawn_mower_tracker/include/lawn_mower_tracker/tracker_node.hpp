#ifndef LAWN_MOWER_TRACKER__TRACKER_NODE_HPP_
#define LAWN_MOWER_TRACKER__TRACKER_NODE_HPP_

#include <chrono>
#include <filesystem>
#include <memory>
#include <mutex>
#include <string>

#include <can_client/msg/vehicle_cmd.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/path.hpp>
#include "lawn_mower_tracker/obstacle_planner.hpp"

#include "lawn_mower_tracker/msg/path_status.hpp"
#include "lawn_mower_tracker/path_loader.hpp"
#include "lawn_mower_tracker/srv/mower_track_control.hpp"
#include "lawn_mower_tracker/tracker_core.hpp"

namespace lawn_mower_tracker
{

class TrackerNode : public rclcpp::Node
{
public:
  explicit TrackerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~TrackerNode() override;

private:
  void cloud_callback(sensor_msgs::msg::PointCloud2::ConstSharedPtr message);
  bool apply_avoidance(ControlResult & result);
  bool cloud_fresh() const;
  void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr message);
  void service_callback(
    const std::shared_ptr<srv::MowerTrackControl::Request> request,
    std::shared_ptr<srv::MowerTrackControl::Response> response);
  void control_callback();
  void publish_stop();
  void publish_status(int32_t state);
  std::filesystem::path resolve_path(const std::string & map_name) const;

  bool avoidance_enabled_{false};
  std::unique_ptr<ObstaclePlanner> obstacle_planner_;
  double cloud_timeout_{0.3}, avoidance_speed_{0.2}, planning_distance_{3.6};
  bool have_cloud_{false};
  bool need_plan_{true};
  AvoidanceResult previous_plan_;
  double planned_direction_{0};
  Pose2D current_pose_, cloud_pose_;
  std::string cloud_frame_;
  rclcpp::Time cloud_stamp_{0, 0, RCL_ROS_TIME};
  std::chrono::steady_clock::time_point cloud_received_;
  std::vector<ObstaclePoint> obstacles_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_subscription_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr local_path_publisher_;
  std::mutex mutex_;
  TrackerCore core_;
  CsvColumns columns_;
  std::string path_file_;
  std::string path_root_;
  std::string path_filename_;
  double wheelbase_;
  double max_angular_speed_;
  double rtk_timeout_sec_;
  bool have_rtk_{false};
  uint32_t line_id_{0U};
  std::chrono::steady_clock::time_point last_rtk_;

  rclcpp::Publisher<can_client::msg::VehicleCmd>::SharedPtr command_publisher_;
  rclcpp::Publisher<msg::PathStatus>::SharedPtr status_publisher_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscription_;
  rclcpp::Service<srv::MowerTrackControl>::SharedPtr control_service_;
  rclcpp::TimerBase::SharedPtr control_timer_;
};

}  // namespace lawn_mower_tracker

#endif  // LAWN_MOWER_TRACKER__TRACKER_NODE_HPP_
