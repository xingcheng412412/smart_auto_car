#ifndef RTK_GPS_DRIVER_CPP_POSE_LOCALIZER_H
#define RTK_GPS_DRIVER_CPP_POSE_LOCALIZER_H

#include <GeographicLib/LocalCartesian.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <array>

#include "rtk_gps_driver_cpp/nmea_parser.h"

namespace rtk_gps_driver_cpp
{

  struct PoseLocalizerConfig
  {
    double origin_latitude = 0.0;
    double origin_longitude = 0.0;
    double origin_altitude = 0.0;
    double status_hold_seconds = 5.0;
    double pose_timeout_seconds = 0.5;
    double max_speed_mps = 10.0;
    double process_noise = 0.01;
  };

  enum class PoseUpdateResult
  {
    WAITING_FOR_STATUS,
    ACCEPTED,
    REJECTED_OUTLIER,
    STATUS_RESET,
    INVALID_SAMPLE
  };

  class PoseLocalizer
  {
  public:
    explicit PoseLocalizer(const PoseLocalizerConfig &config);

    PoseUpdateResult update(const NavigationSample &sample,
                            double monotonic_seconds);
    bool get_pose(double monotonic_seconds, geometry_msgs::msg::Pose *pose);
    void require_fix_quality_check();

  private:
    bool valid_sample(const NavigationSample &sample,
                      double monotonic_seconds) const;
    void reset();
    void update_filter(const std::array<double, 3> &measurement,
                       const std::array<double, 3> &measurement_std,
                       double elapsed_seconds);

    PoseLocalizerConfig config_;
    GeographicLib::LocalCartesian local_cartesian_;
    bool tracking_status_ = false;
    bool eligible_ = false;
    bool filter_initialized_ = false;
    bool have_accepted_position_ = false;
    bool require_fix_quality_check_ = true;
    int32_t last_gps_diff_status_ = 0;
    double status_start_time_ = 0.0;
    double last_accepted_time_ = 0.0;
    std::array<double, 3> last_accepted_position_{{0.0, 0.0, 0.0}};
    std::array<double, 3> filtered_position_{{0.0, 0.0, 0.0}};
    std::array<double, 3> filter_covariance_{{1.0, 1.0, 1.0}};
    geometry_msgs::msg::Pose pose_;
  };

} // namespace rtk_gps_driver_cpp

#endif // RTK_GPS_DRIVER_CPP_POSE_LOCALIZER_H