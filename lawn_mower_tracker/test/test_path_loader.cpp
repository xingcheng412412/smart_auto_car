#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <string>

#include "lawn_mower_tracker/path_loader.hpp"

namespace {

class PathLoaderTest : public ::testing::Test {
protected:
  void SetUp() override
  {
    directory_ = std::filesystem::temp_directory_path() /
      ("lawn_mower_tracker_" + std::to_string(::testing::UnitTest::GetInstance()->random_seed()));
    std::filesystem::create_directories(directory_);
  }

  void TearDown() override { std::filesystem::remove_all(directory_); }

  std::filesystem::path write(const std::string & contents)
  {
    const auto path = directory_ / "path.csv";
    std::ofstream(path) << contents;
    return path;
  }

  std::filesystem::path directory_;
};

TEST_F(PathLoaderTest, LoadsThreeFinitePointsByColumnName)
{
  const auto path = write("pose_x,pose_y,pose_z\n0,0,1\n1,0,2\n2,1,3\n");
  lawn_mower_tracker::CsvColumns columns{"pose_x", "pose_y", "pose_z", true};
  const auto result = lawn_mower_tracker::load_path_csv(path, columns);
  ASSERT_TRUE(result.ok()) << result.error;
  ASSERT_EQ(result.points.size(), 3U);
  EXPECT_DOUBLE_EQ(result.points.back().z, 3.0);
  EXPECT_EQ(result.points.front().direction, 0);
  EXPECT_FALSE(result.points.front().has_path_id);
}

TEST_F(PathLoaderTest, LoadsOptionalVelocityDirectionAndPathId)
{
  const auto path = write(
    "pose_x,pose_y,pose_z,linear_velocity,path_id\n"
    "0,0,0,0.5,1\n"
    "1,0,0,0.005,1\n"
    "2,0,0,-0.4,2\n");
  lawn_mower_tracker::CsvColumns columns{"pose_x", "pose_y", "pose_z", true};
  const auto result = lawn_mower_tracker::load_path_csv(path, columns);
  ASSERT_TRUE(result.ok()) << result.error;
  ASSERT_EQ(result.points.size(), 3U);
  EXPECT_EQ(result.points[0].direction, 1);
  EXPECT_EQ(result.points[1].direction, 0);
  EXPECT_EQ(result.points[2].direction, -1);
  EXPECT_EQ(result.points[0].path_id, 1);
  EXPECT_EQ(result.points[1].path_id, 1);
  EXPECT_EQ(result.points[2].path_id, 2);
  EXPECT_TRUE(result.points[0].has_path_id);
  EXPECT_TRUE(result.points[1].has_path_id);
  EXPECT_TRUE(result.points[2].has_path_id);
}

TEST_F(PathLoaderTest, ReportsMalformedOptionalVelocity)
{
  const auto path = write(
    "x,y,linear_velocity\n"
    "0,0,0.5\n"
    "1,0,bad\n"
    "2,0,-0.5\n");
  const auto result = lawn_mower_tracker::load_path_csv(
    path, {"x", "y", "z", false});
  EXPECT_FALSE(result.ok());
  EXPECT_NE(result.error.find("row 3"), std::string::npos);
  EXPECT_NE(result.error.find("linear_velocity"), std::string::npos);
}

TEST_F(PathLoaderTest, ReportsMalformedOptionalPathId)
{
  const auto path = write(
    "x,y,path_id\n"
    "0,0,1\n"
    "1,0,not-an-integer\n"
    "2,0,2\n");
  const auto result = lawn_mower_tracker::load_path_csv(
    path, {"x", "y", "z", false});
  EXPECT_FALSE(result.ok());
  EXPECT_NE(result.error.find("row 3"), std::string::npos);
  EXPECT_NE(result.error.find("path_id"), std::string::npos);
}

TEST_F(PathLoaderTest, ReportsMissingFile)
{
  const auto result = lawn_mower_tracker::load_path_csv(
    directory_ / "missing.csv", {"x", "y", "z", false});
  EXPECT_FALSE(result.ok());
  EXPECT_NE(result.error.find("file"), std::string::npos);
}

TEST_F(PathLoaderTest, ReportsMissingRequiredColumn)
{
  const auto path = write("x,z\n0,0\n1,0\n2,0\n");
  const auto result = lawn_mower_tracker::load_path_csv(path, {"x", "y", "z", false});
  EXPECT_FALSE(result.ok());
  EXPECT_NE(result.error.find("columns"), std::string::npos);
}

TEST_F(PathLoaderTest, ReportsMalformedNumericRow)
{
  const auto path = write("x,y\n0,0\nbad,1\n2,2\n");
  const auto result = lawn_mower_tracker::load_path_csv(path, {"x", "y", "z", false});
  EXPECT_FALSE(result.ok());
  EXPECT_NE(result.error.find("row 3"), std::string::npos);
}

TEST_F(PathLoaderTest, RequiresAtLeastThreePoints)
{
  const auto path = write("x,y\n0,0\n1,1\n");
  const auto result = lawn_mower_tracker::load_path_csv(path, {"x", "y", "z", false});
  EXPECT_FALSE(result.ok());
  EXPECT_NE(result.error.find("points"), std::string::npos);
}

TEST_F(PathLoaderTest, LoadsOptionalRecordedYaw)
{
  const auto path = write(
    "pose_x,pose_y,pose_z,yaw\n0,0,0,0.1\n1,0,0,0.2\n2,0,0,0.3\n");
  const auto result = lawn_mower_tracker::load_path_csv(path, {});
  ASSERT_TRUE(result.ok()) << result.error;
  EXPECT_TRUE(result.points.back().has_yaw);
  EXPECT_DOUBLE_EQ(result.points.back().yaw, 0.3);
}

TEST_F(PathLoaderTest, RejectsMalformedRecordedYaw)
{
  const auto path = write(
    "pose_x,pose_y,pose_z,yaw\n0,0,0,0.1\n1,0,0,bad\n2,0,0,0.3\n");
  const auto result = lawn_mower_tracker::load_path_csv(path, {});
  EXPECT_FALSE(result.ok());
  EXPECT_NE(result.error.find("row 3"), std::string::npos);
  EXPECT_NE(result.error.find("yaw"), std::string::npos);
}

TEST_F(PathLoaderTest, AllowsLegacyCsvWithoutYaw)
{
  const auto path = write("pose_x,pose_y,pose_z\n0,0,0\n1,0,0\n2,0,0\n");
  const auto result = lawn_mower_tracker::load_path_csv(path, {});
  ASSERT_TRUE(result.ok()) << result.error;
  EXPECT_FALSE(result.points.back().has_yaw);
}

}  // namespace
