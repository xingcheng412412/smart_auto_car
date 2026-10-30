#include "fw_mini_pro_slam3d/cloud_conversion.hpp"

#include <cmath>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace fw_mini_pro_slam3d {
namespace {
using sensor_msgs::msg::Imu;
using sensor_msgs::msg::PointCloud2;

bool later(const builtin_interfaces::msg::Time & now,
           const builtin_interfaces::msg::Time & last) {
  return now.sec > last.sec || (now.sec == last.sec && now.nanosec > last.nanosec);
}
bool valid_stamp(const builtin_interfaces::msg::Time & stamp) {
  return stamp.sec >= 0 && stamp.nanosec < 1000000000u;
}
bool valid_imu(const Imu & imu) {
  const auto & a = imu.linear_acceleration;
  const auto & g = imu.angular_velocity;
  return std::isfinite(a.x) && std::isfinite(a.y) && std::isfinite(a.z) &&
    std::isfinite(g.x) && std::isfinite(g.y) && std::isfinite(g.z) &&
    std::hypot(a.x, a.y, a.z) > 0.0;
}
}  // namespace

class FairyAdapter : public rclcpp::Node {
public:
  FairyAdapter() : Node("fairy_adapter") {
    const auto cloud_in = declare_parameter<std::string>("input_cloud_topic", "/rslidar_points");
    const auto imu_in = declare_parameter<std::string>("input_imu_topic", "/rslidar_imu_data");
    const auto cloud_out = declare_parameter<std::string>("output_cloud_topic", "/fairy/points");
    const auto imu_out = declare_parameter<std::string>("output_imu_topic", "/fairy/imu");
    max_scan_duration_ = declare_parameter<double>("max_scan_duration", 0.2);
    scan_lines_ = declare_parameter<int>("scan_lines", 128);
    imu_frame_ = declare_parameter<std::string>("imu_frame", "fairy_imu");
    if (cloud_in.empty() || imu_in.empty() || cloud_out.empty() || imu_out.empty() ||
        imu_frame_.empty() || cloud_in == cloud_out || imu_in == imu_out ||
        !std::isfinite(max_scan_duration_) || max_scan_duration_ <= 0.0 ||
        scan_lines_ < 1 || scan_lines_ > 65536) {
      throw std::invalid_argument("invalid fairy_adapter parameters");
    }
    cloud_pub_ = create_publisher<PointCloud2>(
      cloud_out, rclcpp::QoS(rclcpp::KeepLast(20)).reliable());
    imu_pub_ = create_publisher<Imu>(
      imu_out, rclcpp::QoS(rclcpp::KeepLast(400)).reliable());
    cloud_sub_ = create_subscription<PointCloud2>(
      cloud_in, rclcpp::SensorDataQoS().keep_last(20),
      std::bind(&FairyAdapter::on_cloud, this, std::placeholders::_1));
    imu_sub_ = create_subscription<Imu>(
      imu_in, rclcpp::SensorDataQoS().keep_last(400),
      std::bind(&FairyAdapter::on_imu, this, std::placeholders::_1));
  }

private:
  void on_cloud(const PointCloud2::SharedPtr input) {
    if (!valid_stamp(input->header.stamp)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Dropping cloud with invalid stamp");
      return;
    }
    if (have_cloud_stamp_ && !later(input->header.stamp, last_cloud_stamp_)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "Cloud timestamp moved backward or repeated; restart the pipeline to reset its clock");
      return;
    }
    try {
      auto output = convert_cloud(*input, max_scan_duration_, scan_lines_);
      cloud_pub_->publish(std::move(output));
      last_cloud_stamp_ = input->header.stamp;
      have_cloud_stamp_ = true;
    } catch (const std::exception & e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "Dropping invalid cloud: %s", e.what());
    }
  }

  void on_imu(const Imu::SharedPtr input) {
    if (!valid_stamp(input->header.stamp) || !valid_imu(*input)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "Dropping IMU sample with invalid stamp, gyro, or acceleration");
      return;
    }
    if (have_imu_stamp_ && !later(input->header.stamp, last_imu_stamp_)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "IMU timestamp moved backward or repeated; restart the pipeline to reset its clock");
      return;
    }
    auto output = *input;
    output.header.frame_id = imu_frame_;
    output.orientation_covariance[0] = -1.0;
    imu_pub_->publish(std::move(output));
    last_imu_stamp_ = input->header.stamp;
    have_imu_stamp_ = true;
  }

  double max_scan_duration_;
  int scan_lines_;
  std::string imu_frame_;
  bool have_cloud_stamp_ = false;
  bool have_imu_stamp_ = false;
  builtin_interfaces::msg::Time last_cloud_stamp_;
  builtin_interfaces::msg::Time last_imu_stamp_;
  rclcpp::Publisher<PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::Publisher<Imu>::SharedPtr imu_pub_;
  rclcpp::Subscription<PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<Imu>::SharedPtr imu_sub_;
};
}  // namespace fw_mini_pro_slam3d

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<fw_mini_pro_slam3d::FairyAdapter>());
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("fairy_adapter"), "Adapter failed: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
