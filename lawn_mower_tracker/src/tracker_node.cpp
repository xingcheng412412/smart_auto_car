#include "lawn_mower_tracker/tracker_node.hpp"

#include <cmath>
#include <algorithm>
#include <cstring>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <cstdlib>
#include <functional>
#include <stdexcept>

#include "lawn_mower_tracker/vehicle_command.hpp"

namespace lawn_mower_tracker
{
namespace
{
std::string expand_user_path(const std::string & value)
{
  if (value == "~" || value.rfind("~/", 0U) == 0U) {
    const char * home = std::getenv("HOME");
    if (home == nullptr || home[0] == '\0') {
      throw std::invalid_argument("HOME is required to expand path parameters");
    }
    return value == "~" ? std::string(home) : std::string(home) + value.substr(1U);
  }
  return value;
}

TrackerParams read_tracker_params(rclcpp::Node & node)
{
  TrackerParams params;
  params.wheelbase = node.declare_parameter("wheelbase", 1.45);
  params.lookahead_distance = node.declare_parameter("lookahead_distance", 2.0);
  params.min_velocity = node.declare_parameter("min_velocity", 0.3);
  params.max_velocity = node.declare_parameter("max_velocity", 1.2);
  params.max_steering_angle = node.declare_parameter("max_steering_angle", 0.942477796);
  params.slowdown_distance = node.declare_parameter("slowdown_distance", 2.0);
  params.stop_distance = node.declare_parameter("stop_distance", 0.2);
  const auto search_back = node.declare_parameter("search_back_points", 5);
  const auto search_forward = node.declare_parameter("search_forward_points", 30);
  if (search_back < 0 || search_forward <= 0) {
    throw std::invalid_argument("path search window parameters are invalid");
  }
  params.search_back_points = static_cast<std::size_t>(search_back);
  params.search_forward_points = static_cast<std::size_t>(search_forward);
  params.steering_gain = node.declare_parameter("steering_gain", 1.0);
  params.max_slope_tangent = node.declare_parameter("max_slope_tangent", 0.3);
  params.direction_switch_distance = node.declare_parameter("direction_switch_distance", 0.5);
  params.initial_search_distance = node.declare_parameter("initial_search_distance", 10.0);
  const auto initial_search = node.declare_parameter("initial_search_points", 10);
  if (initial_search <= 0) {
    throw std::invalid_argument("initial_search_points must be positive");
  }
  params.initial_search_points = static_cast<std::size_t>(initial_search);
  params.max_start_distance = node.declare_parameter("max_start_distance", 3.0);
  params.terminal_alignment_distance =
    node.declare_parameter("terminal_alignment_distance", 3.0);
  params.terminal_yaw_tolerance =
    node.declare_parameter("terminal_yaw_tolerance", 0.174532925);
  return params;
}
}  // namespace

TrackerNode::TrackerNode(const rclcpp::NodeOptions & options)
: Node("lawn_mower_tracker", options), core_(read_tracker_params(*this))
{
  path_file_ = expand_user_path(declare_parameter("path_file", ""));
  path_root_ = expand_user_path(declare_parameter(
    "path_root", "~/fw_mini_pro_control_ros2/recorded_files"));
  path_filename_ = declare_parameter("path_filename", "lawn_mower_records.csv");
  columns_.x = declare_parameter("csv_x_column", "pose_x");
  columns_.y = declare_parameter("csv_y_column", "pose_y");
  columns_.z = declare_parameter("csv_z_column", "pose_z");
  columns_.yaw = declare_parameter("csv_yaw_column", "yaw");
  columns_.z_enabled = declare_parameter("z_enabled", true);
  wheelbase_ = get_parameter("wheelbase").as_double();
  const double control_rate_hz = declare_parameter("control_rate_hz", 50.0);
  rtk_timeout_sec_ = declare_parameter("rtk_timeout_sec", 0.5);
  max_angular_speed_ = declare_parameter("max_angular_speed", 3.0);
  declare_parameter("log_enabled", false);
  declare_parameter("log_directory", "");
  if (!std::isfinite(control_rate_hz) || !std::isfinite(rtk_timeout_sec_) ||
    !std::isfinite(max_angular_speed_) || control_rate_hz <= 0.0 ||
    rtk_timeout_sec_ <= 0.0 || max_angular_speed_ <= 0.0)
  {
    throw std::invalid_argument("control and safety parameters must be positive");
  }

  const auto odom_topic = declare_parameter("odom_topic", "/gnss_pose");
  const auto command_topic = declare_parameter("vehicle_cmd_topic", "/vehicle_cmd");
  const auto status_topic = declare_parameter("path_status_topic", "/path_status");
  const auto service_name = declare_parameter("control_service", "/mowerTrackControl");

  command_publisher_ = create_publisher<can_client::msg::VehicleCmd>(
    command_topic, rclcpp::QoS(1).reliable());
  status_publisher_ = create_publisher<msg::PathStatus>(status_topic, rclcpp::QoS(10).reliable());
  pose_subscription_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    odom_topic, rclcpp::SensorDataQoS(),
    std::bind(&TrackerNode::pose_callback, this, std::placeholders::_1));
  control_service_ = create_service<srv::MowerTrackControl>(
    service_name,
    std::bind(&TrackerNode::service_callback, this, std::placeholders::_1, std::placeholders::_2));
  control_timer_ = create_wall_timer(
    std::chrono::duration<double>(1.0 / control_rate_hz),
    std::bind(&TrackerNode::control_callback, this));
  avoidance_enabled_ = declare_parameter("avoidance_enabled", false);
  const auto cloud_topic = declare_parameter("obstacle_cloud_topic", "/rslidar_points");
  cloud_frame_ = declare_parameter("obstacle_cloud_frame", "fairy_optical");
  cloud_timeout_ = declare_parameter("obstacle_timeout_sec", 0.3);
  avoidance_speed_ = declare_parameter("avoidance_max_speed", 0.2);
  planning_distance_ = declare_parameter("avoidance_planning_distance", 3.6);
  if (avoidance_enabled_) {
    ObstacleParams p;
    p.wheelbase = wheelbase_;
    p.max_steering = get_parameter("max_steering_angle").as_double();
    const double legacy_height = declare_parameter("lidar_optical_height", p.optical_height);
    p.optical_height = declare_parameter("lidar_origin_height", legacy_height);
    const double braking_deceleration = declare_parameter("avoidance_braking_deceleration", 0.5);
    p.body_half_length = declare_parameter("body_half_length", p.body_half_length);
    p.body_half_width = declare_parameter("body_half_width", p.body_half_width);
    p.body_min_z = declare_parameter("body_min_z", p.body_min_z);
    p.body_max_z = declare_parameter("body_max_z", p.body_max_z);
    p.min_height = declare_parameter("obstacle_min_height", p.min_height);
    p.max_height = declare_parameter("obstacle_max_height", p.max_height);
    p.safety_margin = declare_parameter("obstacle_safety_margin", p.safety_margin);
    p.range = declare_parameter("avoidance_range", p.range);
    p.resolution = declare_parameter("avoidance_resolution", p.resolution);
    p.max_expansions = declare_parameter("avoidance_max_expansions", p.max_expansions);
    if (!std::isfinite(cloud_timeout_) || cloud_timeout_<=0 ||
      !std::isfinite(avoidance_speed_) || avoidance_speed_<=0 ||
      !std::isfinite(planning_distance_) || planning_distance_<=0 ||
      planning_distance_ >= p.range-std::hypot(p.body_half_length,p.body_half_width)-
      p.safety_margin-p.resolution || cloud_frame_.empty())
    {throw std::invalid_argument("invalid avoidance timing/speed/horizon");}
    if (!std::isfinite(braking_deceleration) || braking_deceleration<=0) {
      throw std::invalid_argument("avoidance_braking_deceleration must be finite and positive");
    }
    // Latency and braking allowance; deployment must verify the configured deceleration.
    if (p.safety_margin < avoidance_speed_*(cloud_timeout_+1/control_rate_hz)+
      avoidance_speed_*avoidance_speed_/(2*braking_deceleration))
    {throw std::invalid_argument("safety margin is too small for avoidance speed/timeout");}
    p.max_steering = std::min(p.max_steering,
      std::atan(max_angular_speed_*wheelbase_/avoidance_speed_));
    obstacle_planner_ = std::make_unique<ObstaclePlanner>(p);
    filtered_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "/avoidance/filtered_points",rclcpp::SensorDataQoS());
    local_path_publisher_ = create_publisher<nav_msgs::msg::Path>("/avoidance/local_path",1);
    cloud_subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      cloud_topic,rclcpp::SensorDataQoS().keep_last(1),
      std::bind(&TrackerNode::cloud_callback,this,std::placeholders::_1));
  }
  publish_stop();
}

TrackerNode::~TrackerNode()
{
  if (command_publisher_) {
    for (int count = 0; count < 3; ++count) {
      publish_stop();
    }
  }
}

void TrackerNode::pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr message)
{
  const auto & position = message->pose.position;
  const auto & q = message->pose.orientation;
  const double norm = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
  if (!std::isfinite(position.x) || !std::isfinite(position.y) || !std::isfinite(position.z) ||
    !std::isfinite(norm) || norm < 1e-6)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    have_rtk_ = false;
    core_.stop();
    publish_stop();
    publish_status(5);
    RCLCPP_ERROR(get_logger(), "Rejected invalid RTK pose");
    return;
  }
  const double x = q.x / norm;
  const double y = q.y / norm;
  const double z = q.z / norm;
  const double w = q.w / norm;
  const double yaw = std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));

  std::lock_guard<std::mutex> lock(mutex_);
  have_rtk_ = core_.update_pose({position.x, position.y, position.z, yaw});
  if (have_rtk_) {
    current_pose_ = {position.x, position.y, position.z, yaw};
    last_rtk_ = std::chrono::steady_clock::now();
  }
}

std::filesystem::path TrackerNode::resolve_path(const std::string & map_name) const
{
  if (!path_file_.empty()) {
    return path_file_;
  }
  const std::filesystem::path component(map_name);
  if (path_root_.empty() || map_name.empty() || component.is_absolute() ||
    component.has_parent_path() || map_name == "." || map_name == "..")
  {
    return {};
  }
  return std::filesystem::path(path_root_) / component / path_filename_;
}

void TrackerNode::service_callback(
  const std::shared_ptr<srv::MowerTrackControl::Request> request,
  std::shared_ptr<srv::MowerTrackControl::Response> response)
{
  const auto & command = request->track_state;
  if (command == "start") {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      core_.stop();
      have_cloud_ = false;
      need_plan_ = true;
      have_rtk_ = false;
      line_id_ = request->line_id;
      publish_stop();
    }
    const auto path = resolve_path(request->map_name);
    const auto loaded = load_path_csv(path, columns_);
    std::lock_guard<std::mutex> lock(mutex_);
    if (!loaded.ok()) {
      core_.stop();
      publish_stop();
      publish_status(5);
      response->success = false;
      response->message = loaded.error.empty() ? "invalid path selection" : loaded.error;
      return;
    }
    const auto error = core_.start(loaded.points);
    if (!error.empty()) {
      publish_stop();
      publish_status(5);
      response->success = false;
      response->message = error;
      return;
    }
    publish_status(3);
  } else if (command == "pause") {
    std::lock_guard<std::mutex> lock(mutex_);
    if (core_.state() != TrackerState::kTracking) {
      response->success = false;
      response->message = "pause requires a tracking task";
      return;
    }
    core_.pause();
    publish_stop();
    publish_status(4);
  } else if (command == "resume") {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!core_.resume()) {
      response->success = false;
      response->message = "resume requires a paused task";
      return;
    }
    have_rtk_ = false;
    have_cloud_ = false;
    need_plan_ = true;
    publish_status(3);
  } else if (command == "stop") {
    std::lock_guard<std::mutex> lock(mutex_);
    core_.stop();
    publish_stop();
    publish_status(5);
  } else {
    std::lock_guard<std::mutex> lock(mutex_);
    core_.stop();
    have_rtk_ = false;
    response->success = false;
    response->message = "track_state must be start, pause, resume, or stop";
    publish_stop();
    publish_status(5);
    return;
  }
  response->success = true;
  response->message = command + " accepted";
}

void TrackerNode::control_callback()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (core_.state() != TrackerState::kTracking || !have_rtk_) {
    publish_stop();
    return;
  }
  const double age = std::chrono::duration<double>(
    std::chrono::steady_clock::now() - last_rtk_).count();
  if (age > rtk_timeout_sec_) {
    have_rtk_ = false;
    publish_stop();
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "RTK timeout; chassis disabled");
    return;
  }
  auto result = core_.step();
  if (!result.error.empty() || result.completed) {
    publish_stop();
    if (result.completed) {
      publish_status(6);
    } else {
      core_.stop();
      have_cloud_ = false;
      need_plan_ = true;
      have_rtk_ = false;
      publish_status(5);
    }
    return;
  }
  if (avoidance_enabled_ && result.linear_velocity != 0 && !apply_avoidance(result)) {
    publish_stop();
    return;
  }
  // Planning is synchronous: recheck both inputs immediately before enabling traction.
  if (std::chrono::duration<double>(std::chrono::steady_clock::now()-last_rtk_).count() >
    rtk_timeout_sec_ || (avoidance_enabled_ && !cloud_fresh()))
  {
    publish_stop();
    return;
  }
  try {
    command_publisher_->publish(make_drive_command(
      result.linear_velocity, result.steering_angle, wheelbase_, max_angular_speed_));
  } catch (const std::exception & error) {
    RCLCPP_ERROR(get_logger(), "Unsafe command rejected: %s", error.what());
    core_.stop();
    publish_stop();
    publish_status(5);
  }
}

void TrackerNode::publish_stop()
{
  if (local_path_publisher_) {
    nav_msgs::msg::Path empty;
    empty.header.frame_id = cloud_frame_;
    empty.header.stamp = now();
    local_path_publisher_->publish(empty);
  }
  command_publisher_->publish(make_stop_command());
}

void TrackerNode::publish_status(int32_t state)
{
  msg::PathStatus status;
  status.tracker_state = state;
  status.line_id = line_id_;
  status_publisher_->publish(status);
}

}  // namespace lawn_mower_tracker
