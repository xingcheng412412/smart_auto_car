#include "fw_mini_pro_slam3d/cloud_conversion.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include <sensor_msgs/msg/point_field.hpp>

namespace {

using sensor_msgs::msg::PointCloud2;
using sensor_msgs::msg::PointField;

PointField field(const std::string & name, uint32_t offset, uint8_t datatype)
{
  PointField result;
  result.name = name;
  result.offset = offset;
  result.datatype = datatype;
  result.count = 1;
  return result;
}

template<typename T>
void put(std::vector<uint8_t> & data, size_t offset, T value, bool big_endian = false)
{
  uint8_t bytes[sizeof(T)];
  std::memcpy(bytes, &value, sizeof(T));
  const uint16_t one = 1;
  const bool host_big_endian = *reinterpret_cast<const uint8_t *>(&one) == 0;
  if (big_endian != host_big_endian) {
    std::reverse(bytes, bytes + sizeof(T));
  }
  std::memcpy(data.data() + offset, bytes, sizeof(T));
}

template<typename T>
T get(const std::vector<uint8_t> & data, size_t offset)
{
  T value;
  std::memcpy(&value, data.data() + offset, sizeof(T));
  return value;
}

PointCloud2 cloud(uint32_t width = 2, uint32_t height = 1, uint32_t row_padding = 0,
  bool big_endian = false)
{
  PointCloud2 result;
  result.header.frame_id = "fairy_lidar";
  result.header.stamp.sec = 1700000000;
  result.header.stamp.nanosec = 0;
  result.width = width;
  result.height = height;
  result.is_bigendian = big_endian;
  result.point_step = 40;
  result.row_step = width * result.point_step + row_padding;
  result.data.resize(static_cast<size_t>(result.row_step) * height, 0xAA);
  // Deliberately use an unusual field order and gaps.
  result.fields = {
    field("timestamp", 0, PointField::FLOAT64),
    field("ring", 8, PointField::UINT16),
    field("intensity", 12, PointField::FLOAT32),
    field("x", 16, PointField::FLOAT32),
    field("z", 24, PointField::FLOAT32),
    field("y", 20, PointField::FLOAT32)};
  return result;
}

void point(PointCloud2 & c, uint32_t row, uint32_t column, double timestamp,
  uint16_t ring = 3, float x = 1.0f, float y = 2.0f, float z = 3.0f,
  float intensity = 4.0f)
{
  const size_t at = static_cast<size_t>(row) * c.row_step + column * c.point_step;
  put(c.data, at + 0, timestamp, c.is_bigendian);
  put(c.data, at + 8, ring, c.is_bigendian);
  put(c.data, at + 12, intensity, c.is_bigendian);
  put(c.data, at + 16, x, c.is_bigendian);
  put(c.data, at + 20, y, c.is_bigendian);
  put(c.data, at + 24, z, c.is_bigendian);
}

TEST(CloudConversion, UsesAbsoluteEpochTimeAndSortsRelativeOffsets)
{
  auto input = cloud(3);
  point(input, 0, 0, 1700000000.12, 4, 12.0f);
  point(input, 0, 1, 1700000000.02, 5, 2.0f);
  point(input, 0, 2, 1700000000.07, 6, 7.0f);
  const auto output = fw_mini_pro_slam3d::convert_cloud(input);
  EXPECT_EQ(output.header.frame_id, input.header.frame_id);
  EXPECT_EQ(output.header.stamp, input.header.stamp);
  EXPECT_EQ(output.width, 3u);
  EXPECT_EQ(output.height, 1u);
  EXPECT_EQ(output.point_step, 24u);
  EXPECT_EQ(output.row_step, 72u);
  EXPECT_FALSE(output.is_bigendian);
  ASSERT_EQ(output.fields.size(), 6u);
  EXPECT_EQ(output.fields[4].name, "time");
  EXPECT_EQ(output.fields[4].offset, 16u);
  EXPECT_EQ(output.fields[4].datatype, PointField::FLOAT32);
  EXPECT_EQ(output.fields[5].name, "ring");
  EXPECT_EQ(output.fields[5].offset, 20u);
  EXPECT_EQ(get<float>(output.data, 0), 2.0f);
  EXPECT_EQ(get<uint16_t>(output.data, 20), 5u);
  EXPECT_NEAR(get<float>(output.data, 16), 0.02f, 1e-5f);
  EXPECT_EQ(get<float>(output.data, 24), 7.0f);
  EXPECT_NEAR(get<float>(output.data, 40), 0.07f, 1e-5f);
  EXPECT_EQ(get<float>(output.data, 48), 12.0f);
  EXPECT_NEAR(get<float>(output.data, 64), 0.12f, 1e-5f);
}

TEST(CloudConversion, ReadsOrganizedRowsWithPaddingAndBigEndianFields)
{
  auto input = cloud(2, 2, 13, true);
  point(input, 0, 0, 1700000000.01, 1, 1.0f);
  point(input, 0, 1, 1700000000.04, 2, 2.0f);
  point(input, 1, 0, 1700000000.02, 3, 3.0f);
  point(input, 1, 1, 1700000000.03, 4, 4.0f);
  const auto output = fw_mini_pro_slam3d::convert_cloud(input);
  ASSERT_EQ(output.width, 4u);
  EXPECT_EQ(get<float>(output.data, 0), 1.0f);
  EXPECT_EQ(get<float>(output.data, 24), 3.0f);
  EXPECT_EQ(get<float>(output.data, 48), 4.0f);
  EXPECT_EQ(get<float>(output.data, 72), 2.0f);
  EXPECT_EQ(get<uint16_t>(output.data, 72 + 20), 2u);
}

TEST(CloudConversion, DropsNonfiniteValuesAndOutOfRangePoints)
{
  auto input = cloud(6);
  input.header.stamp.sec = 100;
  point(input, 0, 0, 100.0, 1);
  point(input, 0, 1, 100.1, 2);
  point(input, 0, 2, std::numeric_limits<double>::quiet_NaN());
  point(input, 0, 3, 100.03, 3, std::numeric_limits<float>::infinity());
  point(input, 0, 4, 101.0);
  point(input, 0, 5, 100.04, 4, 1, 2, 3,
    std::numeric_limits<float>::quiet_NaN());
  const auto output = fw_mini_pro_slam3d::convert_cloud(input, 0.2);
  EXPECT_EQ(output.width, 2u);
  EXPECT_NEAR(get<float>(output.data, 24 + 16), 0.1f, 1e-5f);
}

TEST(CloudConversion, RejectsInvalidRingPoints)
{
  auto input = cloud(3);
  input.header.stamp.sec = 100;
  point(input, 0, 0, 100.0, 127);
  point(input, 0, 1, 100.05, 128);
  point(input, 0, 2, 100.1, 2);
  const auto output = fw_mini_pro_slam3d::convert_cloud(input, 0.2, 128);
  ASSERT_EQ(output.width, 2u);
  EXPECT_EQ(get<uint16_t>(output.data, 20), 127u);
  EXPECT_EQ(get<uint16_t>(output.data, 44), 2u);
}

TEST(CloudConversion, RejectsMalformedSchemaAndTruncatedPayload)
{
  auto input = cloud();
  input.header.stamp.sec = 100;
  point(input, 0, 0, 100.0);
  point(input, 0, 1, 100.1);
  input.fields[0].datatype = PointField::FLOAT32;
  EXPECT_THROW(fw_mini_pro_slam3d::convert_cloud(input), std::invalid_argument);
  input.fields[0].datatype = PointField::FLOAT64;
  input.fields[0].count = 2;
  EXPECT_THROW(fw_mini_pro_slam3d::convert_cloud(input), std::invalid_argument);
  input.fields[0].count = 1;
  input.fields[1].offset = 39;
  EXPECT_THROW(fw_mini_pro_slam3d::convert_cloud(input), std::invalid_argument);
  input.fields[1].offset = 8;
  input.row_step = 79;
  EXPECT_THROW(fw_mini_pro_slam3d::convert_cloud(input), std::invalid_argument);
  input.row_step = 80;
  input.data.pop_back();
  EXPECT_THROW(fw_mini_pro_slam3d::convert_cloud(input), std::invalid_argument);
}

TEST(CloudConversion, RejectsEmptyAndZeroDurationClouds)
{
  EXPECT_THROW(fw_mini_pro_slam3d::convert_cloud(cloud(0)), std::invalid_argument);
  auto input = cloud();
  input.header.stamp.sec = 100;
  point(input, 0, 0, 100.0);
  point(input, 0, 1, 100.0);
  EXPECT_THROW(fw_mini_pro_slam3d::convert_cloud(input), std::invalid_argument);
}


TEST(CloudConversion, UsesHeaderTimeEvenWhenEarlierPointIsInvalid)
{
  auto input = cloud(3);
  input.header.stamp.sec = 100;
  point(input, 0, 0, 100.0, 1, std::numeric_limits<float>::quiet_NaN());
  point(input, 0, 1, 100.02, 2, 2.0f);
  point(input, 0, 2, 100.09, 3, 9.0f);
  const auto output = fw_mini_pro_slam3d::convert_cloud(input);
  ASSERT_EQ(output.width, 2u);
  EXPECT_NEAR(get<float>(output.data, 16), 0.02f, 1e-5f);
  EXPECT_NEAR(get<float>(output.data, 40), 0.09f, 1e-5f);
}

TEST(CloudConversion, ClampsTinyNegativeOffsetAndDropsLargerNegativeOffset)
{
  auto input = cloud(4);
  input.header.stamp.sec = 100;
  point(input, 0, 0, 99.999998, 1);
  point(input, 0, 1, 99.9999995, 2);
  point(input, 0, 2, 100.04, 3);
  point(input, 0, 3, 100.21, 4);
  const auto output = fw_mini_pro_slam3d::convert_cloud(input);
  ASSERT_EQ(output.width, 2u);
  EXPECT_EQ(get<uint16_t>(output.data, 20), 2u);
  EXPECT_EQ(get<float>(output.data, 16), 0.0f);
  EXPECT_NEAR(get<float>(output.data, 40), 0.04f, 1e-5f);
}

TEST(CloudConversion, RejectsInvalidHeaderStamp)
{
  auto input = cloud();
  point(input, 0, 0, 100.0);
  point(input, 0, 1, 100.1);
  input.header.stamp.sec = -1;
  EXPECT_THROW(fw_mini_pro_slam3d::convert_cloud(input), std::invalid_argument);
  input.header.stamp.sec = 100;
  input.header.stamp.nanosec = 1000000000u;
  EXPECT_THROW(fw_mini_pro_slam3d::convert_cloud(input), std::invalid_argument);
}

}  // namespace
