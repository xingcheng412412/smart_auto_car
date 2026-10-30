#include "rtk_gps_driver_cpp/nmea_parser.h"

#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <stdexcept>

namespace rtk_gps_driver_cpp
{

  namespace
  {

    constexpr uint8_t kHeader0 = 0xBD;
    constexpr uint8_t kHeader1 = 0xDB;
    constexpr uint8_t kHeader2 = 0x0B;
    // Total frame size: 88 bytes (offset 0-87)
    constexpr size_t kFrameSize = 88;

    template <typename T>
    T read_lsb(const uint8_t *data)
    {
      T result = 0;
      for (size_t i = 0; i < sizeof(T); ++i)
      {
        result |= static_cast<T>(data[i]) << (i * 8);
      }
      return result;
    }

    int16_t read_i16(const uint8_t *data)
    {
      return static_cast<int16_t>(read_lsb<uint16_t>(data));
    }

    int32_t read_i32(const uint8_t *data)
    {
      return static_cast<int32_t>(read_lsb<uint32_t>(data));
    }

    int64_t read_i64(const uint8_t *data)
    {
      return static_cast<int64_t>(read_lsb<uint64_t>(data));
    }

    uint8_t compute_checksum(const uint8_t *data, size_t len)
    {
      uint8_t cs = 0;
      for (size_t i = 0; i < len; ++i)
      {
        cs ^= data[i];
      }
      return cs;
    }

    // INS status bits: bit0=position, bit1=velocity, bit2=attitude, bit3=heading
    // All 4 bits set (0x0F) means fully aligned
    int ins_status_to_fix_quality(uint8_t ins_status)
    {
      if ((ins_status & 0x0F) == 0x0F)
        return 4; // RTK fixed equivalent
      if ((ins_status & 0x01) != 0)
        return 1; // Position aligned
      return 0;   // No alignment
    }

  } // namespace

  Bddb0bParser::Bddb0bParser(const std::string &frame_id)
      : frame_id_(frame_id) {}

  void Bddb0bParser::parse_bytes(const std::vector<uint8_t> &data)
  {
    if (data.empty())
      return;

    static bool first_data_logged = false;
    if (!first_data_logged && log_callback_)
    {
      first_data_logged = true;
      char buf[256];
      int offset = std::snprintf(buf, sizeof(buf),
                                 "First serial data received (%zu bytes), first 16: ",
                                 data.size());
      const size_t preview = std::min(data.size(), size_t(16));
      for (size_t i = 0; i < preview; ++i)
      {
        offset += std::snprintf(buf + offset, sizeof(buf) - offset,
                                "%02X ", static_cast<unsigned int>(data[i]));
      }
      log_callback_("INFO", "BDDB0B_DATA", buf);
    }

    buffer_.insert(buffer_.end(), data.begin(), data.end());
    process_buffer();
  }

  void Bddb0bParser::process_buffer()
  {
    while (buffer_.size() >= kFrameSize)
    {
      // Find frame header 0xBD 0xDB 0x0B
      size_t pos = 0;
      bool found = false;
      while (pos + kFrameSize <= buffer_.size())
      {
        if (buffer_[pos] == kHeader0 && buffer_[pos + 1] == kHeader1 && buffer_[pos + 2] == kHeader2)
        {
          found = true;
          break;
        }
        ++pos;
      }

      if (!found)
      {
        // Keep last 2 bytes in case of partial header
        if (buffer_.size() > 2)
        {
          buffer_.erase(buffer_.begin(), buffer_.begin() + (buffer_.size() - 2));
        }
        break;
      }

      // Remove bytes before header
      if (pos > 0)
      {
        buffer_.erase(buffer_.begin(), buffer_.begin() + pos);
      }

      if (buffer_.size() < kFrameSize)
        break;

      // Try to parse the frame
      if (try_parse_frame(0))
      {
        buffer_.erase(buffer_.begin(), buffer_.begin() + kFrameSize);
      }
      else
      {
        // Invalid frame, skip first byte and try again
        if (log_callback_)
        {
          static int skip_count = 0;
          if (skip_count < 5)
          {
            skip_count++;
            char buf[64];
            std::snprintf(buf, sizeof(buf),
                          "Invalid BDDB0B frame at offset 0, skipping byte (count=%d)",
                          skip_count);
            log_callback_("WARN", "BDDB0B_PARSE", buf);
          }
        }
        buffer_.erase(buffer_.begin());
      }
    }
  }

  bool Bddb0bParser::try_parse_frame(size_t start_idx)
  {
    if (buffer_.size() < start_idx + kFrameSize)
      return false;

    const uint8_t *data = buffer_.data() + start_idx;

    // Verify header
    if (data[0] != kHeader0 || data[1] != kHeader1 || data[2] != kHeader2)
      return false;

    // Verify checksum at offset 57: XOR of bytes 0-56
    uint8_t cs1 = compute_checksum(data, 57);
    if (cs1 != data[57])
    {
      if (log_callback_)
      {
        char buf[128];
        std::snprintf(buf, sizeof(buf),
                      "Checksum1 mismatch: computed=0x%02X received=0x%02X",
                      static_cast<unsigned int>(cs1), static_cast<unsigned int>(data[57]));
        log_callback_("WARN", "BDDB0B_CHECKSUM", buf);
      }
      return false;
    }

    rclcpp::Time current_time = rclcpp::Clock().now();

    // ===== BDDB0B field parsing (per 字段定义表.csv) =====

    // Offset 3: Roll (I16), deg = value * 360/32768, [-90,90], right tilt positive
    double roll_deg = static_cast<double>(read_i16(data + 3)) * 360.0 / 32768.0;
    // Offset 5: Pitch (I16), deg = value * 360/32768, [-180,180], nose up positive
    double pitch_deg = static_cast<double>(read_i16(data + 5)) * 360.0 / 32768.0;
    // Offset 7: Heading (I16), deg = value * 360/32768, [-180,180], North=0, clockwise positive
    double heading_deg = static_cast<double>(read_i16(data + 7)) * 360.0 / 32768.0;

    // Offset 9: Gyro X (I16), deg/s = value * 300/32768
    double gyro_x = static_cast<double>(read_i16(data + 9)) * 300.0 / 32768.0;
    // Offset 11: Gyro Y (I16), deg/s = value * 300/32768
    double gyro_y = static_cast<double>(read_i16(data + 11)) * 300.0 / 32768.0;
    // Offset 13: Gyro Z (I16), deg/s = value * 300/32768, clockwise positive
    double gyro_z = static_cast<double>(read_i16(data + 13)) * 300.0 / 32768.0;

    // Offset 15: Accel X (I16), g = value * 12/32768
    double acc_x = static_cast<double>(read_i16(data + 15)) * 12.0 / 32768.0;
    // Offset 17: Accel Y (I16), g = value * 12/32768
    double acc_y = static_cast<double>(read_i16(data + 17)) * 12.0 / 32768.0;
    // Offset 19: Accel Z (I16), g = value * 12/32768
    double acc_z = static_cast<double>(read_i16(data + 19)) * 12.0 / 32768.0;

    // Offset 21: Latitude (I32), deg = value * 1e-7, [-90,90], positive=N
    double latitude = static_cast<double>(read_i32(data + 21)) * 1e-7;
    // Offset 25: Longitude (I32), deg = value * 1e-7, [-180,180], positive=E
    double longitude = static_cast<double>(read_i32(data + 25)) * 1e-7;
    // Offset 29: Altitude (I32), m = value * 1e-3
    double altitude = static_cast<double>(read_i32(data + 29)) * 1e-3;

    // Offset 33: North velocity (I16), m/s = value * 100/32768
    double velocity_north = static_cast<double>(read_i16(data + 33)) * 100.0 / 32768.0;
    // Offset 35: East velocity (I16), m/s = value * 100/32768
    double velocity_east = static_cast<double>(read_i16(data + 35)) * 100.0 / 32768.0;
    // Offset 37: Down velocity (I16), m/s = value * 100/32768
    double velocity_down = static_cast<double>(read_i16(data + 37)) * 100.0 / 32768.0;

    // Offset 39: INS status (U8)
    uint8_t ins_status = data[39];

    // Offset 40-45: Reserved (6 bytes, not defined in spec)

    // Offset 46: Data1 (I16, 2 bytes) - poll data
    int16_t poll_data1 = read_i16(data + 46);
    // Offset 48: Data2 (I16, 2 bytes) - poll data
    int16_t poll_data2 = read_i16(data + 48);
    // Offset 50: Data3 (I16, 2 bytes) - poll data
    int16_t poll_data3 = read_i16(data + 50);

    // Offset 52: Time (U32, 4 bytes), seconds = value * 2.5e-4, GPS time (ms within week)
    // uint32_t gps_time_ms = read_lsb<uint32_t>(data + 52);

    // Offset 56: Type (U8, 1 byte) - poll data type
    uint8_t poll_type = data[56];

    // Offset 57: Checksum1 (U8) - already verified above

    // Offset 58: GPS week (U32, 4 bytes)
    // uint32_t gps_week = read_lsb<uint32_t>(data + 58);

    // Offset 62: Checksum2 (U8) - XOR of bytes 0-61
    // uint8_t cs2 = compute_checksum(data, 62);
    // if (cs2 != data[62]) return false;

    // Offset 63: Latitude precise (I64, 8 bytes), deg = value * 1e-8 (mm precision)
    double latitude_precise = static_cast<double>(read_i64(data + 63)) * 1e-8;
    // Offset 71: Longitude precise (I64, 8 bytes), deg = value * 1e-8 (mm precision)
    double longitude_precise = static_cast<double>(read_i64(data + 71)) * 1e-8;

    // Offset 79: Checksum3 (U8) - XOR of bytes 0-78
    // uint8_t cs3 = compute_checksum(data, 79);
    // if (cs3 != data[79]) return false;

    // Offset 80: Gravity-free Accel X (I16, 2 bytes), g = value * 12/32768
    double acc_gf_x = static_cast<double>(read_i16(data + 80)) * 12.0 / 32768.0;
    // Offset 82: Gravity-free Accel Y (I16, 2 bytes), g = value * 12/32768
    double acc_gf_y = static_cast<double>(read_i16(data + 82)) * 12.0 / 32768.0;
    // Offset 84: Gravity-free Accel Z (I16, 2 bytes), g = value * 12/32768
    double acc_gf_z = static_cast<double>(read_i16(data + 84)) * 12.0 / 32768.0;

    // Offset 86: Checksum4 (U8) - XOR of bytes 0-85
    // uint8_t cs4 = compute_checksum(data, 86);
    // if (cs4 != data[86]) return false;

    // Offset 87: Reserved

    // Use precise lat/lon if available (mm-level precision)
    if (std::abs(latitude_precise) > 1e-12)
    {
      latitude = latitude_precise;
    }
    if (std::abs(longitude_precise) > 1e-12)
    {
      longitude = longitude_precise;
    }

    // Convert velocity: NED -> ENU (down -> up)
    double velocity_up = -velocity_down;

    // Determine fix quality from INS status
    int fix_quality = ins_status_to_fix_quality(ins_status);

    // Publish NavSatFix
    if (fix_callback_)
    {
      sensor_msgs::msg::NavSatFix fix_msg;
      fix_msg.header.stamp = current_time;
      fix_msg.header.frame_id = frame_id_;
      fix_msg.latitude = latitude;
      fix_msg.longitude = longitude;
      fix_msg.altitude = altitude;
      fix_msg.status.status = fix_quality > 0 ? 0 : -1;
      fix_msg.status.service = 0;
      double cov = (fix_quality >= 4) ? 0.01 : ((fix_quality >= 1) ? 1.0 : 100.0);
      fix_msg.position_covariance = {
          cov, 0.0, 0.0,
          0.0, cov, 0.0,
          0.0, 0.0, cov * 4.0};
      fix_msg.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_APPROXIMATED;
      fix_callback_(fix_msg);
    }

    // Publish TwistWithCovarianceStamped (velocity in ENU frame)
    if (velocity_callback_)
    {
      geometry_msgs::msg::TwistWithCovarianceStamped velocity_msg;
      velocity_msg.header.stamp = current_time;
      velocity_msg.header.frame_id = "enu";
      velocity_msg.twist.twist.linear.x = velocity_east;
      velocity_msg.twist.twist.linear.y = velocity_north;
      velocity_msg.twist.twist.linear.z = velocity_up;
      velocity_msg.twist.covariance[0] = 0.01;
      velocity_msg.twist.covariance[7] = 0.01;
      velocity_msg.twist.covariance[14] = 0.01;
      velocity_callback_(velocity_msg);
    }

    // Publish IMU
    if (imu_callback_)
    {
      sensor_msgs::msg::Imu imu_msg;
      imu_msg.header.stamp = current_time;
      imu_msg.header.frame_id = "imu";

      // Roll about X, Pitch about Y, Heading about Z
      tf2::Quaternion tf_q;
      tf_q.setRPY(
          roll_deg * M_PI / 180.0,
          pitch_deg * M_PI / 180.0,
          heading_deg * M_PI / 180.0);
      imu_msg.orientation = tf2::toMsg(tf_q);

      // Angular velocity (deg/s -> rad/s)
      imu_msg.angular_velocity.x = gyro_x * M_PI / 180.0;
      imu_msg.angular_velocity.y = gyro_y * M_PI / 180.0;
      imu_msg.angular_velocity.z = gyro_z * M_PI / 180.0;

      // Use gravity-free acceleration for linear_acceleration if available
      // (excludes gravity component, more useful for navigation)
      if (acc_gf_x != 0.0 || acc_gf_y != 0.0 || acc_gf_z != 0.0)
      {
        imu_msg.linear_acceleration.x = acc_gf_x * 9.80665;
        imu_msg.linear_acceleration.y = acc_gf_y * 9.80665;
        imu_msg.linear_acceleration.z = acc_gf_z * 9.80665;
      }
      else
      {
        // Fallback to raw acceleration (includes gravity)
        imu_msg.linear_acceleration.x = acc_x * 9.80665;
        imu_msg.linear_acceleration.y = acc_y * 9.80665;
        imu_msg.linear_acceleration.z = acc_z * 9.80665;
      }

      imu_callback_(imu_msg);
    }

    // Publish NavigationSample for pose localizer
    if (navigation_callback_)
    {
      NavigationSample sample;
      sample.stamp = current_time;
      sample.latitude = latitude;
      sample.longitude = longitude;
      sample.altitude = altitude;
      sample.heading_deg = heading_deg;
      sample.pitch_deg = pitch_deg;
      sample.roll_deg = roll_deg;
      sample.velocity_east = velocity_east;
      sample.velocity_north = velocity_north;
      sample.velocity_up = velocity_up;
      sample.chc_status = ins_status;
      sample.fix_quality = fix_quality;
      sample.num_satellites = 0; // Not available in BDDB0B
      sample.poll_type = poll_type;
      sample.poll_data1 = poll_data1;
      sample.poll_data2 = poll_data2;
      sample.poll_data3 = poll_data3;
      sample.gps_diff_status = (poll_type == 32) ? static_cast<int32_t>(poll_data3) : -1;
      navigation_callback_(sample);
    }

    return true;
  }

} // namespace rtk_gps_driver_cpp
