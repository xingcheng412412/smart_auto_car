#include "fw_mini_pro_slam3d/cloud_conversion.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include <sensor_msgs/msg/point_field.hpp>

namespace fw_mini_pro_slam3d {
namespace {
using sensor_msgs::msg::PointCloud2;
using sensor_msgs::msg::PointField;

bool host_big_endian() {
  const uint16_t one = 1;
  return *reinterpret_cast<const uint8_t *>(&one) == 0;
}
template<typename T> T read(const uint8_t * p, bool big) {
  uint8_t bytes[sizeof(T)];
  std::memcpy(bytes, p, sizeof(T));
  if (big != host_big_endian()) std::reverse(bytes, bytes + sizeof(T));
  T value;
  std::memcpy(&value, bytes, sizeof(T));
  return value;
}
template<typename T> void write(uint8_t * p, T value) {
  uint8_t bytes[sizeof(T)];
  std::memcpy(bytes, &value, sizeof(T));
  if (host_big_endian()) std::reverse(bytes, bytes + sizeof(T));
  std::memcpy(p, bytes, sizeof(T));
}
uint32_t required(const PointCloud2 & c, const char * name, uint8_t type, uint32_t bytes) {
  const PointField * found = nullptr;
  for (const auto & f : c.fields) {
    if (f.name == name) {
      if (found) throw std::invalid_argument(std::string("duplicate field: ") + name);
      found = &f;
    }
  }
  if (!found || found->datatype != type || found->count != 1 ||
      found->offset > c.point_step || bytes > c.point_step - found->offset)
    throw std::invalid_argument(std::string("invalid or missing field: ") + name);
  return found->offset;
}
PointField field(const char * name, uint32_t offset, uint8_t type) {
  PointField f;
  f.name = name; f.offset = offset; f.datatype = type; f.count = 1;
  return f;
}
struct Point {
  float x, y, z, intensity;
  double timestamp;
  uint16_t ring;
};
}  // namespace

PointCloud2 convert_cloud(const PointCloud2 & input, double max_scan_duration, int scan_lines) {
  if (!std::isfinite(max_scan_duration) || max_scan_duration <= 0.0)
    throw std::invalid_argument("max_scan_duration must be finite and positive");
  if (scan_lines < 1 || scan_lines > 65536)
    throw std::invalid_argument("scan_lines must be in [1, 65536]");
  if (input.header.stamp.sec < 0 || input.header.stamp.nanosec >= 1000000000u)
    throw std::invalid_argument("cloud header stamp is invalid");
  const double start = static_cast<double>(input.header.stamp.sec) +
    static_cast<double>(input.header.stamp.nanosec) * 1e-9;
  const uint64_t count = static_cast<uint64_t>(input.width) * input.height;
  if (count == 0 || count > std::numeric_limits<uint32_t>::max())
    throw std::invalid_argument("cloud is empty or too large");
  const uint64_t minimum_row_step = static_cast<uint64_t>(input.width) * input.point_step;
  const uint64_t payload_size = static_cast<uint64_t>(input.row_step) * input.height;
  if (!input.point_step || input.row_step < minimum_row_step || payload_size > input.data.size())
    throw std::invalid_argument("invalid row_step or payload");
  const uint32_t x = required(input, "x", PointField::FLOAT32, 4);
  const uint32_t y = required(input, "y", PointField::FLOAT32, 4);
  const uint32_t z = required(input, "z", PointField::FLOAT32, 4);
  const uint32_t intensity = required(input, "intensity", PointField::FLOAT32, 4);
  const uint32_t ring = required(input, "ring", PointField::UINT16, 2);
  const uint32_t timestamp = required(input, "timestamp", PointField::FLOAT64, 8);

  std::vector<Point> points;
  points.reserve(static_cast<size_t>(count));
  for (uint32_t row = 0; row < input.height; ++row) {
    for (uint32_t column = 0; column < input.width; ++column) {
      const uint64_t at = static_cast<uint64_t>(row) * input.row_step +
        static_cast<uint64_t>(column) * input.point_step;
      const uint8_t * p = input.data.data() + static_cast<size_t>(at);
      Point v{read<float>(p+x, input.is_bigendian),
              read<float>(p+y, input.is_bigendian),
              read<float>(p+z, input.is_bigendian),
              read<float>(p+intensity, input.is_bigendian),
              read<double>(p+timestamp, input.is_bigendian),
              read<uint16_t>(p+ring, input.is_bigendian)};
      if (std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z) &&
          std::isfinite(v.intensity) && std::isfinite(v.timestamp) && v.ring < scan_lines)
        points.push_back(v);
    }
  }
  if (points.empty()) throw std::invalid_argument("cloud has no valid points");
  std::stable_sort(points.begin(), points.end(),
    [](const Point & a, const Point & b) { return a.timestamp < b.timestamp; });
  constexpr double negative_tolerance = 1e-6;
  points.erase(std::remove_if(points.begin(), points.end(),
    [start, max_scan_duration](const Point & p) {
      const double t = p.timestamp - start;
      return t < -negative_tolerance || t > max_scan_duration;
    }), points.end());
  if (points.size() < 2 || points.back().timestamp <= std::max(start, points.front().timestamp))
    throw std::invalid_argument("cloud has zero scan duration after filtering");

  PointCloud2 output;
  output.header = input.header;
  output.height = 1;
  output.width = static_cast<uint32_t>(points.size());
  output.is_bigendian = false;
  output.is_dense = true;
  output.point_step = 24;
  const uint64_t size = static_cast<uint64_t>(output.width) * output.point_step;
  if (size > std::numeric_limits<uint32_t>::max())
    throw std::invalid_argument("converted cloud too large");
  output.row_step = static_cast<uint32_t>(size);
  output.fields = {field("x", 0, PointField::FLOAT32),
                   field("y", 4, PointField::FLOAT32),
                   field("z", 8, PointField::FLOAT32),
                   field("intensity", 12, PointField::FLOAT32),
                   field("time", 16, PointField::FLOAT32),
                   field("ring", 20, PointField::UINT16)};
  output.data.assign(static_cast<size_t>(size), 0);
  for (size_t i = 0; i < points.size(); ++i) {
    uint8_t * p = output.data.data() + i * output.point_step;
    const auto & v = points[i];
    const double t = v.timestamp - start;
    write(p, v.x); write(p+4, v.y); write(p+8, v.z); write(p+12, v.intensity);
    write(p+16, static_cast<float>(t < 0.0 ? 0.0 : t));
    write(p+20, v.ring);
  }
  return output;
}
}  // namespace fw_mini_pro_slam3d
