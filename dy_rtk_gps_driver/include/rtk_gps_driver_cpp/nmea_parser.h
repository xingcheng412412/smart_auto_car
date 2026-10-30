#ifndef RTK_GPS_DRIVER_CPP_NMEA_PARSER_H
#define RTK_GPS_DRIVER_CPP_NMEA_PARSER_H

#include <string>
#include <cstdint>
#include <vector>
#include <functional>
#include <rclcpp/time.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>

namespace rtk_gps_driver_cpp
{

  struct NavigationSample
  {
    rclcpp::Time stamp;
    double latitude = 0.0;
    double longitude = 0.0;
    double altitude = 0.0;
    double heading_deg = 0.0;
    double pitch_deg = 0.0;
    double roll_deg = 0.0;
    double velocity_east = 0.0;
    double velocity_north = 0.0;
    double velocity_up = 0.0;
    double latitude_std = 0.0;
    double longitude_std = 0.0;
    double altitude_std = 0.0;
    uint8_t chc_status = 0;
    int fix_quality = 0;
    int num_satellites = 0;
    // Poll data (offset 46-51)
    uint8_t poll_type = 0;
    int16_t poll_data1 = 0;
    int16_t poll_data2 = 0;
    int16_t poll_data3 = 0;
    // GPS差分标志位(仅poll_type=32时有效): 48=L1_INT, 49=WIDE_INT, 50=NARROW_INT
    int32_t gps_diff_status = 0;
  };

  using FixCallback = std::function<void(const sensor_msgs::msg::NavSatFix &)>;
  using ImuCallback = std::function<void(const sensor_msgs::msg::Imu &)>;
  using VelocityCallback = std::function<void(
      const geometry_msgs::msg::TwistWithCovarianceStamped &)>;
  using LogCallback = std::function<void(const std::string &log_level,
                                         const std::string &error_type,
                                         const std::string &error_description)>;

  using NavigationCallback = std::function<void(const NavigationSample &)>;

  class Bddb0bParser
  {
  public:
    explicit Bddb0bParser(const std::string &frame_id = "gps");

    void parse_bytes(const std::vector<uint8_t> &data);

    void set_fix_callback(FixCallback cb) { fix_callback_ = cb; }
    void set_imu_callback(ImuCallback cb) { imu_callback_ = cb; }
    void set_velocity_callback(VelocityCallback cb) { velocity_callback_ = cb; }
    void set_navigation_callback(NavigationCallback cb)
    {
      navigation_callback_ = cb;
    }
    void set_log_callback(LogCallback cb) { log_callback_ = cb; }

  private:
    void process_buffer();
    bool try_parse_frame(size_t start_idx);

    std::string frame_id_;

    std::vector<uint8_t> buffer_;

    FixCallback fix_callback_;
    ImuCallback imu_callback_;
    VelocityCallback velocity_callback_;
    NavigationCallback navigation_callback_;
    LogCallback log_callback_;
  };

} // namespace rtk_gps_driver_cpp

#endif // RTK_GPS_DRIVER_CPP_NMEA_PARSER_H
