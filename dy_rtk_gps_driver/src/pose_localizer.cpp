#include "rtk_gps_driver_cpp/pose_localizer.h"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace rtk_gps_driver_cpp
{
  namespace
  {

    constexpr double kMinimumMeasurementVariance = 1e-4;
    constexpr double kTimeEpsilon = 1e-9;

    bool finite_and_nonnegative(double value)
    {
      return std::isfinite(value) && value >= 0.0;
    }

  } // namespace

  PoseLocalizer::PoseLocalizer(const PoseLocalizerConfig &config)
      : config_(config),
        local_cartesian_(config.origin_latitude,
                         config.origin_longitude,
                         config.origin_altitude)
  {
    if (!std::isfinite(config_.origin_latitude) ||
        config_.origin_latitude < -90.0 || config_.origin_latitude > 90.0 ||
        !std::isfinite(config_.origin_longitude) ||
        config_.origin_longitude < -180.0 || config_.origin_longitude > 180.0 ||
        !std::isfinite(config_.origin_altitude) ||
        !std::isfinite(config_.status_hold_seconds) ||
        config_.status_hold_seconds < 0.0 ||
        !std::isfinite(config_.pose_timeout_seconds) ||
        config_.pose_timeout_seconds <= 0.0 ||
        !std::isfinite(config_.max_speed_mps) || config_.max_speed_mps <= 0.0 ||
        !finite_and_nonnegative(config_.process_noise))
    {
      throw std::invalid_argument("Invalid pose localizer configuration");
    }
    pose_.orientation.w = 1.0;
  }

  bool PoseLocalizer::valid_sample(const NavigationSample &sample,
                                   double monotonic_seconds) const
  {
    return std::isfinite(monotonic_seconds) &&
           std::isfinite(sample.latitude) && sample.latitude >= -90.0 &&
           sample.latitude <= 90.0 &&
           std::isfinite(sample.longitude) && sample.longitude >= -180.0 &&
           sample.longitude <= 180.0 &&
           std::isfinite(sample.altitude) &&
           std::isfinite(sample.heading_deg) &&
           std::isfinite(sample.pitch_deg) &&
           std::isfinite(sample.roll_deg) &&
           finite_and_nonnegative(sample.latitude_std) &&
           finite_and_nonnegative(sample.longitude_std) &&
           finite_and_nonnegative(sample.altitude_std);
  }

  void PoseLocalizer::reset()
  {
    tracking_status_ = false;
    eligible_ = false;
    filter_initialized_ = false;
    have_accepted_position_ = false;
    last_gps_diff_status_ = 0;
    status_start_time_ = 0.0;
    last_accepted_time_ = 0.0;
    last_accepted_position_ = {{0.0, 0.0, 0.0}};
    filtered_position_ = {{0.0, 0.0, 0.0}};
    filter_covariance_ = {{1.0, 1.0, 1.0}};
    pose_ = geometry_msgs::msg::Pose();
    pose_.orientation.w = 1.0;
  }

  void PoseLocalizer::require_fix_quality_check()
  {
    require_fix_quality_check_ = true;
    reset();
  }

  void PoseLocalizer::update_filter(
      const std::array<double, 3> &measurement,
      const std::array<double, 3> &measurement_std,
      double elapsed_seconds)
  {
    if (!filter_initialized_)
    {
      filtered_position_ = measurement;
      for (std::size_t axis = 0; axis < filter_covariance_.size(); ++axis)
      {
        filter_covariance_[axis] = std::max(
            measurement_std[axis] * measurement_std[axis],
            kMinimumMeasurementVariance);
      }
      filter_initialized_ = true;
      return;
    }

    const double prediction_elapsed = std::max(0.0, elapsed_seconds);
    for (std::size_t axis = 0; axis < filtered_position_.size(); ++axis)
    {
      filter_covariance_[axis] += config_.process_noise * prediction_elapsed;
      const double measurement_variance = std::max(
          measurement_std[axis] * measurement_std[axis],
          kMinimumMeasurementVariance);
      const double gain = filter_covariance_[axis] /
                          (filter_covariance_[axis] + measurement_variance);
      filtered_position_[axis] +=
          gain * (measurement[axis] - filtered_position_[axis]);
      filter_covariance_[axis] *= (1.0 - gain);
    }
  }

  PoseUpdateResult PoseLocalizer::update(const NavigationSample &sample,
                                         double monotonic_seconds)
  {
    if (!valid_sample(sample, monotonic_seconds))
    {
      return PoseUpdateResult::INVALID_SAMPLE;
    }

    if (have_accepted_position_ &&
        monotonic_seconds - last_accepted_time_ >
            config_.pose_timeout_seconds)
    {
      reset();
    }

    // Require INS fully aligned (fix_quality==4) AND GPS fixed solution
    // (gps_diff_status=48/49/50) to accept data for pose publishing,
    // but only during initial power-up or after reset_mode/initial_gnss_pose.
    // Once the system passes the quality gate, subsequent data is accepted
    // without quality checks.
    if (sample.gps_diff_status >= 0)
    {
      last_gps_diff_status_ = sample.gps_diff_status;
    }

    if (require_fix_quality_check_)
    {
      const bool ins_fully_aligned = (sample.fix_quality == 4);
      const bool gps_fixed = (last_gps_diff_status_ == 48 ||
                              last_gps_diff_status_ == 49 ||
                              last_gps_diff_status_ == 50);
      if (!ins_fully_aligned || !gps_fixed)
      {
        reset();
        return PoseUpdateResult::STATUS_RESET;
      }
    }

    if (!tracking_status_)
    {
      tracking_status_ = true;
      status_start_time_ = monotonic_seconds;
    }

    double east = 0.0;
    double north = 0.0;
    double up = 0.0;
    local_cartesian_.Forward(sample.latitude, sample.longitude, sample.altitude,
                             east, north, up);
    const std::array<double, 3> position{{north, -east, up}};

    double elapsed_since_accepted = 0.0;
    if (have_accepted_position_)
    {
      elapsed_since_accepted = monotonic_seconds - last_accepted_time_;
      if (elapsed_since_accepted < 0.0)
      {
        return PoseUpdateResult::INVALID_SAMPLE;
      }
    }

    const std::array<double, 3> position_std{{sample.latitude_std, sample.longitude_std, sample.altitude_std}};
    update_filter(position, position_std, elapsed_since_accepted);
    have_accepted_position_ = true;
    last_accepted_position_ = position;
    last_accepted_time_ = monotonic_seconds;

    pose_.position.x = filtered_position_[0];
    pose_.position.y = filtered_position_[1];
    pose_.position.z = filtered_position_[2];
    tf2::Quaternion quaternion;
    quaternion.setRPY(
        sample.roll_deg * M_PI / 180.0,
        -sample.pitch_deg * M_PI / 180.0,
        -sample.heading_deg * M_PI / 180.0);
    pose_.orientation = tf2::toMsg(quaternion);

    if (monotonic_seconds - status_start_time_ + kTimeEpsilon >=
        config_.status_hold_seconds)
    {
      eligible_ = true;
      require_fix_quality_check_ = false;
      return PoseUpdateResult::ACCEPTED;
    }
    return PoseUpdateResult::WAITING_FOR_STATUS;
  }

  bool PoseLocalizer::get_pose(double monotonic_seconds,
                               geometry_msgs::msg::Pose *pose)
  {
    if (pose == nullptr || !std::isfinite(monotonic_seconds) || !eligible_ ||
        !have_accepted_position_)
    {
      return false;
    }
    if (monotonic_seconds - last_accepted_time_ >
        config_.pose_timeout_seconds)
    {
      reset();
      return false;
    }
    *pose = pose_;
    return true;
  }

} // namespace rtk_gps_driver_cpp