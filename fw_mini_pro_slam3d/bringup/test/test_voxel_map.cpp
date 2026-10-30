#include <gtest/gtest.h>
#include <cmath>
#include <filesystem>
#include <fstream>
#include "fw_mini_pro_slam3d/voxel_map.hpp"
using namespace fw_mini_pro_slam3d;
TEST(VoxelMap, MergesVoxelCentroidsAndKeepsNegativeCells) {
  VoxelMap map(1.0, 10);
  map.add({0.1f, 0.2f, 0.3f, 1.f});
  map.add({0.3f, 0.4f, 0.5f, 3.f});
  map.add({-0.1f, 0.2f, 0.3f, 5.f});
  EXPECT_EQ(map.size(), 2u);
  for (auto p : map.points()) if (p.x > 0) {
    EXPECT_NEAR(p.x, .2, 1e-6); EXPECT_NEAR(p.intensity, 2., 1e-6);
  }
}
TEST(VoxelMap, CapacityRejectsNewCellsAndReportsTruncation) {
  VoxelMap map(1.0, 1);
  map.add({0.f, 0.f, 0.f, 1.f});
  map.add({2.f, 0.f, 0.f, 1.f});
  EXPECT_EQ(map.size(), 1u); EXPECT_TRUE(map.truncated());
  map.add({0.5f, 0.f, 0.f, 3.f});
  EXPECT_NEAR(map.points()[0].x, .25f, 1e-6);
}
TEST(VoxelMap, InvalidValuesAndConfiguration) {
  EXPECT_THROW(VoxelMap(0., 1), std::invalid_argument);
  EXPECT_THROW(VoxelMap(1., 0), std::invalid_argument);
  VoxelMap map(.1, 10);
  map.add({NAN, 0.f, 0.f, 1.f});
  map.add({INFINITY, 0.f, 0.f, 1.f});
  map.add({1e30f, 0.f, 0.f, 1.f});
  EXPECT_EQ(map.size(), 0u);
}
TEST(VoxelMap, SaveNonemptyBinaryAndReportErrors) {
  const auto dir = std::filesystem::temp_directory_path() / "fairy_voxel_map_test";
  std::filesystem::create_directories(dir);
  VoxelMap map(.1, 10);
  EXPECT_THROW(map.save((dir / "empty.pcd").string()), std::runtime_error);
  map.add({1.f, 2.f, 3.f, 4.f});
  const auto file = dir / "map.pcd";
  map.save(file.string());
  std::ifstream input(file, std::ios::binary);
  std::string line; bool found = false;
  while (std::getline(input,line)) if (line == "DATA binary") { found = true; break; }
  ASSERT_TRUE(found);
  MapPoint p{}; input.read(reinterpret_cast<char*>(&p), sizeof(p));
  ASSERT_TRUE(input); EXPECT_FLOAT_EQ(p.x,1.f); EXPECT_FLOAT_EQ(p.intensity,4.f);
  EXPECT_THROW(map.save(dir.string()), std::exception);
  std::filesystem::remove_all(dir);
}
