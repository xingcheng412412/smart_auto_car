#include "rtk_gps_driver_cpp/rtk_gps_driver_node.h"
#include "rtk_gps_driver_cpp/gnss_origin_storage.h"
#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <cmath>
#include <exception>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <vector>

namespace rtk_gps_driver_cpp
{
  namespace
  {

    double monotonic_seconds()
    {
      return std::chrono::duration<double>(
                 std::chrono::steady_clock::now().time_since_epoch())
          .count();
    }

    constexpr double kInitialPoseMaximumSampleAgeSeconds = 0.5;

    bool finite_and_nonnegative(double value)
    {
      return std::isfinite(value) && value >= 0.0;
    }

    bool valid_initial_pose_sample(const NavigationSample &sample)
    {
      return std::isfinite(sample.latitude) && sample.latitude >= -90.0 &&
             sample.latitude <= 90.0 && std::isfinite(sample.longitude) &&
             sample.longitude >= -180.0 && sample.longitude <= 180.0 &&
             std::isfinite(sample.altitude) &&
             std::isfinite(sample.heading_deg) &&
             std::isfinite(sample.pitch_deg) && std::isfinite(sample.roll_deg) &&
             finite_and_nonnegative(sample.latitude_std) &&
             finite_and_nonnegative(sample.longitude_std) &&
             finite_and_nonnegative(sample.altitude_std);
    }

  } // namespace

  RTKGPSDriverNode::RTKGPSDriverNode(const rclcpp::NodeOptions &options)
      : Node("rtk_gps_node", options),
        bddb0b_parser_("gps"),
        last_fix_quality_(0),
        last_latitude_(0.0),
        last_longitude_(0.0),
        last_altitude_(0.0),
        last_ins_status_(0),
        last_lat_std_(0.0),
        last_lon_std_(0.0),
        last_h_std_(0.0),
        last_vn_std_(0.0),
        last_ve_std_(0.0),
        last_vd_std_(0.0),
        last_roll_std_(0.0),
        last_pitch_std_(0.0),
        last_yaw_std_(0.0),
        last_temperature_(0.0),
        last_gps_position_status_(0),
        last_gps_num_satellites_(0),
        last_gps_diff_status_(0),
        last_wheel_speed_status_(0),
        latest_navigation_time_(0.0),
        have_latest_navigation_sample_(false),
        latest_navigation_rejected_as_outlier_(false),
        pose_publishing_(false),
        pose_gate_tracking_(false),
        gnss_pose_state_("WAITING"),
        non_rtk_fixed_count_(0),
        in_non_rtk_fixed_state_(false),
        non_rtk_fixed_duration_(0.0),
        serial_connected_(false)
  {

    // Declare and get parameters
    declare_parameter<std::string>("serial_port", "/dev/ttyS0");
    declare_parameter<int>("baudrate", 460800);
    declare_parameter<std::string>("frame_id", "gps");
    declare_parameter<double>("log_interval", 0.5);
    declare_parameter<double>("pose_publish_rate", 50.0);
    declare_parameter<std::string>("map_frame", "map");
    declare_parameter<std::string>("pose_frame", "gps");
    declare_parameter<std::string>("gnss_config_file", "");
    declare_parameter<double>("pose_timeout", 0.5);
    declare_parameter<double>("max_pose_speed", 10.0);
    declare_parameter<double>("pose_process_noise", 0.01);
    declare_parameter<double>("lat", 0.0);
    declare_parameter<double>("lon", 0.0);
    declare_parameter<double>("h", 0.0);
    declare_parameter<double>("serial_reconnect_interval", 1.0);

    // Load parameters
    serial_port_name_ = get_parameter("serial_port").as_string();
    baudrate_ = get_parameter("baudrate").as_int();
    frame_id_ = get_parameter("frame_id").as_string();
    log_interval_ = get_parameter("log_interval").as_double();
    if (!std::isfinite(log_interval_) || log_interval_ <= 0.0)
    {
      RCLCPP_WARN(get_logger(), "Invalid log_interval %.3f; using 0.5 seconds", log_interval_);
      log_interval_ = 0.5;
    }

    pose_publish_rate_ = get_parameter("pose_publish_rate").as_double();
    if (!std::isfinite(pose_publish_rate_) || pose_publish_rate_ <= 0.0)
    {
      RCLCPP_WARN(get_logger(), "Invalid pose_publish_rate %.3f; using 50Hz", pose_publish_rate_);
      pose_publish_rate_ = 50.0;
    }
    map_frame_ = get_parameter("map_frame").as_string();
    pose_frame_ = get_parameter("pose_frame").as_string();
    gnss_config_file_ = get_parameter("gnss_config_file").as_string();
    serial_reconnect_interval_ = get_parameter("serial_reconnect_interval").as_double();
    if (!std::isfinite(serial_reconnect_interval_) || serial_reconnect_interval_ <= 0.0)
    {
      serial_reconnect_interval_ = 1.0;
    }

    pose_config_.origin_latitude = get_parameter("lat").as_double();
    pose_config_.origin_longitude = get_parameter("lon").as_double();
    pose_config_.origin_altitude = get_parameter("h").as_double();

    pose_config_.pose_timeout_seconds = get_parameter("pose_timeout").as_double();
    pose_config_.max_speed_mps = get_parameter("max_pose_speed").as_double();
    pose_config_.process_noise = get_parameter("pose_process_noise").as_double();
    pose_config_.status_hold_seconds = 5.0;

    bool have_pose_origin = has_parameter("lat") && has_parameter("lon") && has_parameter("h");

    if (!std::isfinite(pose_config_.pose_timeout_seconds) ||
        pose_config_.pose_timeout_seconds <= 0.0)
    {
      RCLCPP_WARN(get_logger(), "Invalid pose_timeout %.3f; using 0.5 seconds",
                  pose_config_.pose_timeout_seconds);
      pose_config_.pose_timeout_seconds = 0.5;
    }
    if (!std::isfinite(pose_config_.max_speed_mps) ||
        pose_config_.max_speed_mps <= 0.0)
    {
      RCLCPP_WARN(get_logger(), "Invalid max_pose_speed %.3f; using 10m/s",
                  pose_config_.max_speed_mps);
      pose_config_.max_speed_mps = 10.0;
    }
    if (!std::isfinite(pose_config_.process_noise) ||
        pose_config_.process_noise < 0.0)
    {
      RCLCPP_WARN(get_logger(), "Invalid pose_process_noise %.3f; using 0.01",
                  pose_config_.process_noise);
      pose_config_.process_noise = 0.01;
    }
    if (have_pose_origin)
    {
      try
      {
        pose_localizer_.reset(new PoseLocalizer(pose_config_));
        RCLCPP_INFO(get_logger(), "GNSS pose origin lat=%.10f lon=%.10f h=%.3f",
                    pose_config_.origin_latitude, pose_config_.origin_longitude,
                    pose_config_.origin_altitude);
      }
      catch (const std::exception &error)
      {
        RCLCPP_ERROR(get_logger(), "GNSS pose output disabled: %s", error.what());
      }
    }
    else
    {
      RCLCPP_WARN(get_logger(), "GNSS pose output disabled: private lat/lon/h parameters missing");
    }

    bddb0b_parser_ = Bddb0bParser(frame_id_);

    RCLCPP_INFO(get_logger(), "********************************");
    RCLCPP_INFO(get_logger(), "Serial Port:     %s", serial_port_name_.c_str());
    RCLCPP_INFO(get_logger(), "Baudrate:        %u", baudrate_);
    RCLCPP_INFO(get_logger(), "Protocol:        BDDB0B (INS5720DAA)");
    RCLCPP_INFO(get_logger(), "********************************");

    // Publishers
    fix_pub_ = create_publisher<sensor_msgs::msg::NavSatFix>("fix", 1);
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("imu_raw", 10);
    velocity_pub_ = create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(
        "velocity_enu", 10);
    log_pub_ = create_publisher<rtk_gps_driver_cpp::msg::RTKGPSLog>("rtk_log", 10);
    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("gnss_pose", 1);

    // Services
    initial_gnss_pose_srv_ = create_service<std_srvs::srv::SetBool>(
        "initial_gnss_pose",
        std::bind(&RTKGPSDriverNode::initial_gnss_pose_callback, this,
                  std::placeholders::_1, std::placeholders::_2));

    // Pose timer
    pose_timer_ = create_wall_timer(
        std::chrono::duration<double>(1.0 / pose_publish_rate_),
        std::bind(&RTKGPSDriverNode::pose_timer_callback, this));

    // Serial timer for reconnection and data handling
    serial_timer_ = create_wall_timer(
        std::chrono::milliseconds(10),
        std::bind(&RTKGPSDriverNode::serial_timer_callback, this));

    // TF broadcaster
    pose_tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    // Set parser callbacks
    bddb0b_parser_.set_fix_callback([this](const sensor_msgs::msg::NavSatFix &msg)
                                    {
    fix_pub_->publish(msg);
    std::lock_guard<std::mutex> lock(state_mutex_);
    last_latitude_ = msg.latitude;
    last_longitude_ = msg.longitude;
    last_altitude_ = msg.altitude; });

    bddb0b_parser_.set_imu_callback([this](const sensor_msgs::msg::Imu &msg)
                                    { imu_pub_->publish(msg); });

    bddb0b_parser_.set_velocity_callback(
        [this](const geometry_msgs::msg::TwistWithCovarianceStamped &msg)
        {
          velocity_pub_->publish(msg);
        });

    bddb0b_parser_.set_navigation_callback([this](const NavigationSample &sample)
                                           {
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      last_fix_quality_ = sample.fix_quality;
      last_latitude_ = sample.latitude;
      last_longitude_ = sample.longitude;
      last_altitude_ = sample.altitude;
      last_ins_status_ = sample.chc_status;
      // Parse poll data according to type
      switch (sample.poll_type) {
        case 0: // Position accuracy (m)
          last_lat_std_ = sample.poll_data1 / 100.0;
          last_lon_std_ = sample.poll_data2 / 100.0;
          last_h_std_ = sample.poll_data3 / 100.0;
          break;
        case 1: // Velocity accuracy (m/s)
          last_vn_std_ = sample.poll_data1 / 100.0;
          last_ve_std_ = sample.poll_data2 / 100.0;
          last_vd_std_ = sample.poll_data3 / 100.0;
          break;
        case 2: // Attitude accuracy (deg), formula: data*200/32768
          last_roll_std_ = sample.poll_data1 * 200.0 / 32768.0;
          last_pitch_std_ = sample.poll_data2 * 200.0 / 32768.0;
          last_yaw_std_ = sample.poll_data3 * 200.0 / 32768.0;
          break;
        case 22: // Temperature (℃)
          last_temperature_ = sample.poll_data1 * 200.0 / 32768.0;
          break;
        case 32: // GPS status
          last_gps_position_status_ = sample.poll_data1;
          last_gps_num_satellites_ = sample.poll_data2;
          last_gps_diff_status_ = sample.gps_diff_status;
          break;
        case 33: // Wheel speed status
          last_wheel_speed_status_ = sample.poll_data2;
          break;
      }

      if (sample.fix_quality != 4) {
        non_rtk_fixed_count_++;
        if (!in_non_rtk_fixed_state_) {
          in_non_rtk_fixed_state_ = true;
          non_rtk_fixed_start_ = std::chrono::steady_clock::now();
        }
        const auto now = std::chrono::steady_clock::now();
        non_rtk_fixed_duration_ =
            std::chrono::duration<double>(now - non_rtk_fixed_start_).count();
      } else {
        in_non_rtk_fixed_state_ = false;
        non_rtk_fixed_duration_ = 0.0;
      }
    }

    const double sample_time = monotonic_seconds();
    PoseUpdateResult result = PoseUpdateResult::INVALID_SAMPLE;
    if (pose_localizer_) {
      result = pose_localizer_->update(sample, sample_time);
    }

    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      latest_navigation_sample_ = sample;
      latest_navigation_time_ = sample_time;
      have_latest_navigation_sample_ = true;
      latest_navigation_rejected_as_outlier_ =
          result == PoseUpdateResult::REJECTED_OUTLIER;
    }

    if (!pose_localizer_) {
      std::lock_guard<std::mutex> lock(state_mutex_);
      gnss_pose_state_ = "INACTIVE";
      return;
    }

    switch (result) {
      case PoseUpdateResult::WAITING_FOR_STATUS:
        if (!pose_gate_tracking_) {
          RCLCPP_INFO(get_logger(), "[gnss_pose] INS status alignment in progress; starting continuous 5s gate");
          pose_gate_tracking_ = true;
        }
        {
          std::lock_guard<std::mutex> lock(state_mutex_);
          gnss_pose_state_ = "STATUS_HOLD";
        }
        break;
      case PoseUpdateResult::ACCEPTED:
        pose_gate_tracking_ = false;
        {
          std::lock_guard<std::mutex> lock(state_mutex_);
          gnss_pose_state_ = "PUBLISHING";
        }
        break;
      case PoseUpdateResult::REJECTED_OUTLIER:
        RCLCPP_WARN_SKIPFIRST_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "[gnss_pose] rejected BDDB0B position above configured 3D speed limit");
        {
          std::lock_guard<std::mutex> lock(state_mutex_);
          gnss_pose_state_ = "REJECTED_OUTLIER";
        }
        break;
      case PoseUpdateResult::STATUS_RESET:
        if (pose_gate_tracking_ || pose_publishing_) {
          RCLCPP_WARN(get_logger(), "[gnss_pose] ins_status=%u; pose gate reset",
                   static_cast<unsigned int>(sample.chc_status));
        }
        pose_gate_tracking_ = false;
        pose_publishing_ = false;
        {
          std::lock_guard<std::mutex> lock(state_mutex_);
          gnss_pose_state_ = "STATUS_RESET";
        }
        break;
      case PoseUpdateResult::INVALID_SAMPLE:
        RCLCPP_WARN_SKIPFIRST_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "[gnss_pose] ignored invalid BDDB0B navigation sample");
        {
          std::lock_guard<std::mutex> lock(state_mutex_);
          gnss_pose_state_ = "INVALID_SAMPLE";
        }
        break;
    } });

    bddb0b_parser_.set_log_callback([this](const std::string &log_level,
                                           const std::string &error_type,
                                           const std::string &error_description)
                                    { publish_log(log_level, error_type, error_description); });
  }

  RTKGPSDriverNode::~RTKGPSDriverNode()
  {
    if (serial_connected_)
    {
      serial_port_.close();
    }
  }

  bool RTKGPSDriverNode::try_connect_serial()
  {
    if (serial_connected_ && serial_port_.is_open())
    {
      return true;
    }

    // Clean up previous connection if any
    if (serial_port_.is_open())
    {
      serial_port_.close();
    }
    serial_connected_ = false;

    if (serial_port_.open(serial_port_name_, baudrate_))
    {
      serial_connected_ = true;
      RCLCPP_INFO(get_logger(), "Serial port %s opened at %d baud",
                  serial_port_name_.c_str(), baudrate_);
      return true;
    }

    return false;
  }

  void RTKGPSDriverNode::serial_timer_callback()
  {
    // Try to reconnect if not connected
    if (!serial_connected_)
    {
      auto now = std::chrono::steady_clock::now();
      auto elapsed = std::chrono::duration<double>(now - last_reconnect_attempt_).count();

      if (elapsed >= serial_reconnect_interval_)
      {
        last_reconnect_attempt_ = now;
        if (try_connect_serial())
        {
          // Successfully reconnected
        }
        else
        {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                               "Failed to open serial port %s: %s. Retrying...",
                               serial_port_name_.c_str(), serial_port_.last_error().c_str());
        }
      }
      return;
    }

    // Check if serial port is still valid
    if (!serial_port_.is_open())
    {
      serial_connected_ = false;
      RCLCPP_WARN(get_logger(), "Serial port %s disconnected", serial_port_name_.c_str());
      return;
    }

    // Handle serial data
    handle_serial_data();

    // Periodically publish status log
    static int status_counter = 0;
    if (++status_counter >= 100)
    {
      status_counter = 0;
      publish_status_log();
    }
  }

  void RTKGPSDriverNode::handle_serial_data()
  {
    std::vector<uint8_t> data = serial_port_.drain_bytes();
    if (!data.empty())
    {
      bddb0b_parser_.parse_bytes(data);
    }
  }

  void RTKGPSDriverNode::pose_timer_callback()
  {
    if (!pose_localizer_)
    {
      return;
    }

    geometry_msgs::msg::Pose pose;
    const bool ready = pose_localizer_->get_pose(monotonic_seconds(), &pose);
    if (!ready)
    {
      if (pose_publishing_)
      {
        RCLCPP_WARN(get_logger(), "[gnss_pose] accepted BDDB0B data exceeded configured timeout; "
                                  "pose and TF stopped");
        pose_gate_tracking_ = false;
        {
          std::lock_guard<std::mutex> lock(state_mutex_);
          gnss_pose_state_ = "TIMEOUT";
        }
      }
      pose_publishing_ = false;
      return;
    }

    const rclcpp::Time stamp = now();
    geometry_msgs::msg::PoseStamped pose_message;
    pose_message.header.stamp = stamp;
    pose_message.header.frame_id = map_frame_;
    pose_message.pose = pose;
    pose_pub_->publish(pose_message);

    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = stamp;
    transform.header.frame_id = map_frame_;
    transform.child_frame_id = pose_frame_;
    transform.transform.translation.x = pose.position.x;
    transform.transform.translation.y = pose.position.y;
    transform.transform.translation.z = pose.position.z;
    transform.transform.rotation.x = pose.orientation.x;
    transform.transform.rotation.y = pose.orientation.y;
    transform.transform.rotation.z = pose.orientation.z;
    transform.transform.rotation.w = pose.orientation.w;
    pose_tf_broadcaster_->sendTransform(transform);

    if (!pose_publishing_)
    {
      RCLCPP_INFO(get_logger(), "[gnss_pose] INS fully aligned; publishing at %.1fHz",
                  pose_publish_rate_);
    }
    pose_publishing_ = true;
  }

  void RTKGPSDriverNode::publish_log(const std::string &log_level,
                                     const std::string &error_type,
                                     const std::string &error_description)
  {
    rtk_gps_driver_cpp::msg::RTKGPSLog log_msg;
    log_msg.header.stamp = now();
    log_msg.header.frame_id = frame_id_;

    log_msg.log_level = log_level;

    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      log_msg.initial_state = last_fix_quality_;
      log_msg.latitude = last_latitude_;
      log_msg.longitude = last_longitude_;
      log_msg.altitude = last_altitude_;
      log_msg.ins_status = last_ins_status_;
      log_msg.lat_std = last_lat_std_;
      log_msg.lon_std = last_lon_std_;
      log_msg.h_std = last_h_std_;
      log_msg.vn_std = last_vn_std_;
      log_msg.ve_std = last_ve_std_;
      log_msg.vd_std = last_vd_std_;
      log_msg.roll_std = last_roll_std_;
      log_msg.pitch_std = last_pitch_std_;
      log_msg.yaw_std = last_yaw_std_;
      log_msg.temperature = last_temperature_;
      log_msg.gps_position_status = last_gps_position_status_;
      log_msg.gps_num_satellites = last_gps_num_satellites_;
      log_msg.gps_diff_status = last_gps_diff_status_;
      log_msg.wheel_speed_status = last_wheel_speed_status_;

      if (in_non_rtk_fixed_state_)
      {
        const auto now = std::chrono::steady_clock::now();
        non_rtk_fixed_duration_ =
            std::chrono::duration<double>(now - non_rtk_fixed_start_).count();
      }
      log_msg.non_rtk_fixed_count = non_rtk_fixed_count_;
      log_msg.non_rtk_fixed_duration = non_rtk_fixed_duration_;
      log_msg.gnss_pose_state = gnss_pose_state_;
    }

    log_msg.error_type = error_type;
    log_msg.error_description = error_description;

    log_pub_->publish(log_msg);
  }

  void RTKGPSDriverNode::publish_status_log()
  {
    publish_log(serial_connected_ ? "INFO" : "WARN", "", "");
  }

  void RTKGPSDriverNode::initial_gnss_pose_callback(
      const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
      std::shared_ptr<std_srvs::srv::SetBool::Response> response)
  {
    const auto fail = [this, &response](const std::string &category,
                                        const std::string &description)
    {
      response->success = false;
      response->message = category + ": " + description;
      RCLCPP_WARN(get_logger(), "[initial_gnss_pose] %s", response->message.c_str());
    };

    if (!request->data)
    {
      fail("REQUEST_DISABLED", "request data must be true");
      return;
    }

    NavigationSample sample;
    double sample_time = 0.0;
    bool have_sample = false;
    bool rejected_as_outlier = false;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      sample = latest_navigation_sample_;
      sample_time = latest_navigation_time_;
      have_sample = have_latest_navigation_sample_;
      rejected_as_outlier = latest_navigation_rejected_as_outlier_;
    }

    if (!have_sample)
    {
      fail("NO_GNSS_DATA", "no BDDB0B navigation sample received");
      return;
    }

    const double now = monotonic_seconds();
    const double sample_age = now - sample_time;
    if (!std::isfinite(sample_age) || sample_age < 0.0 ||
        sample_age > kInitialPoseMaximumSampleAgeSeconds)
    {
      std::ostringstream description;
      description << std::fixed << std::setprecision(3)
                  << "newest BDDB0B sample age is " << sample_age
                  << "s; maximum is "
                  << kInitialPoseMaximumSampleAgeSeconds << "s";
      fail("STALE_GNSS_DATA", description.str());
      return;
    }

    // INS fully aligned: all 4 bits set (0x0F)
    if (sample.chc_status != 0x0F)
    {
      std::ostringstream description;
      description << "newest BDDB0B ins_status is "
                  << static_cast<unsigned int>(sample.chc_status)
                  << "; required status is 15 (0x0F, fully aligned)";
      fail("STATUS_NOT_FIXED", description.str());
      return;
    }
    if (!valid_initial_pose_sample(sample))
    {
      fail("INVALID_GNSS_DATA",
           "newest BDDB0B position, attitude or standard deviation is invalid");
      return;
    }
    if (rejected_as_outlier)
    {
      fail("OUTLIER_REJECTED",
           "newest BDDB0B sample exceeded the configured 3D speed limit");
      return;
    }

    PoseLocalizerConfig candidate_config = pose_config_;
    candidate_config.origin_latitude = sample.latitude;
    candidate_config.origin_longitude = sample.longitude;
    candidate_config.origin_altitude = sample.altitude;

    std::unique_ptr<PoseLocalizer> candidate_localizer;
    try
    {
      candidate_localizer.reset(new PoseLocalizer(candidate_config));
    }
    catch (const std::exception &error)
    {
      fail("INITIALIZATION_ERROR", std::string("PoseLocalizer failed: ") + error.what());
      return;
    }

    std::string update_error;
    if (!update_gnss_origin_yaml(gnss_config_file_,
                                 sample.latitude, sample.longitude, sample.altitude,
                                 &update_error))
    {
      fail("FILE_UPDATE_FAILED", update_error);
      return;
    }

    pose_localizer_ = std::move(candidate_localizer);
    pose_config_ = candidate_config;

    RCLCPP_INFO(get_logger(),
                "[initial_gnss_pose] updated GNSS origin to lat=%.10f lon=%.10f h=%.3f",
                sample.latitude, sample.longitude, sample.altitude);

    response->success = true;
    std::ostringstream success_message;
    success_message << std::fixed << std::setprecision(10)
                    << "GNSS origin updated to lat=" << sample.latitude
                    << " lon=" << sample.longitude
                    << " h=" << sample.altitude;
    response->message = success_message.str();
  }

} // namespace rtk_gps_driver_cpp

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rtk_gps_driver_cpp::RTKGPSDriverNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
