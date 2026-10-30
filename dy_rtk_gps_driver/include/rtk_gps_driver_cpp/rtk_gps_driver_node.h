#ifndef RTK_GPS_DRIVER_CPP_RTK_GPS_DRIVER_NODE_H
#define RTK_GPS_DRIVER_CPP_RTK_GPS_DRIVER_NODE_H

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rtk_gps_driver_cpp/msg/rtkgps_log.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "rtk_gps_driver_cpp/serial_port.h"
#include "rtk_gps_driver_cpp/nmea_parser.h"
#include "rtk_gps_driver_cpp/pose_localizer.h"

#include <string>
#include <cstdint>
#include <chrono>
#include <memory>
#include <mutex>

namespace rtk_gps_driver_cpp
{

  class RTKGPSDriverNode : public rclcpp::Node
  {
  public:
    explicit RTKGPSDriverNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    ~RTKGPSDriverNode();

  private:
    // ROS interface
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr fix_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr velocity_pub_;
    rclcpp::Publisher<rtk_gps_driver_cpp::msg::RTKGPSLog>::SharedPtr log_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr initial_gnss_pose_srv_;
    rclcpp::TimerBase::SharedPtr pose_timer_;
    rclcpp::TimerBase::SharedPtr serial_timer_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> pose_tf_broadcaster_;

    // Components
    SerialPort serial_port_;
    Bddb0bParser bddb0b_parser_;
    std::unique_ptr<PoseLocalizer> pose_localizer_;

    // Parameters
    std::string serial_port_name_;
    int baudrate_;
    std::string frame_id_;
    double log_interval_;
    double pose_publish_rate_;
    std::string map_frame_;
    std::string pose_frame_;
    std::string gnss_config_file_;
    PoseLocalizerConfig pose_config_;
    double serial_reconnect_interval_;

    // State tracking
    std::mutex state_mutex_;
    int last_fix_quality_;
    double last_latitude_;
    double last_longitude_;
    double last_altitude_;
    uint8_t last_ins_status_;
    // Poll data (parsed)
    double last_lat_std_;
    double last_lon_std_;
    double last_h_std_;
    double last_vn_std_;
    double last_ve_std_;
    double last_vd_std_;
    double last_roll_std_;
    double last_pitch_std_;
    double last_yaw_std_;
    double last_temperature_;
    int last_gps_position_status_;
    int last_gps_num_satellites_;
    int last_gps_diff_status_;
    int last_wheel_speed_status_;
    NavigationSample latest_navigation_sample_;
    double latest_navigation_time_;
    bool have_latest_navigation_sample_;
    bool latest_navigation_rejected_as_outlier_;

    bool pose_publishing_;
    bool pose_gate_tracking_;

    // GNSS pose state for logging
    // INACTIVE, WAITING, STATUS_HOLD, PUBLISHING,
    // STATUS_RESET, INVALID_SAMPLE, REJECTED_OUTLIER, TIMEOUT
    std::string gnss_pose_state_;

    // INS alignment tracking
    int non_rtk_fixed_count_;
    std::chrono::steady_clock::time_point non_rtk_fixed_start_;
    bool in_non_rtk_fixed_state_;
    double non_rtk_fixed_duration_;

    // Serial reconnection
    bool serial_connected_;
    std::chrono::steady_clock::time_point last_reconnect_attempt_;

    // Service callback
    void initial_gnss_pose_callback(
        const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response);
    void pose_timer_callback();
    void serial_timer_callback();

    // Helpers
    void publish_status_log();
    void publish_log(const std::string &log_level,
                     const std::string &error_type,
                     const std::string &error_description);
    bool try_connect_serial();
    void handle_serial_data();
  };

} // namespace rtk_gps_driver_cpp

#endif // RTK_GPS_DRIVER_CPP_RTK_GPS_DRIVER_NODE_H
