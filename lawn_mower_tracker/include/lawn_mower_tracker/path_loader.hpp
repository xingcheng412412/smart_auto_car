#ifndef LAWN_MOWER_TRACKER__PATH_LOADER_HPP_
#define LAWN_MOWER_TRACKER__PATH_LOADER_HPP_

#include <filesystem>
#include <string>
#include <vector>

#include "lawn_mower_tracker/types.hpp"

namespace lawn_mower_tracker
{

struct CsvColumns
{
  std::string x{"pose_x"};
  std::string y{"pose_y"};
  std::string z{"pose_z"};
  bool z_enabled{true};
  std::string velocity{"linear_velocity"};
  std::string path_id{"path_id"};
  double stationary_velocity_threshold{0.01};
  std::string yaw{"yaw"};
};

struct PathLoadResult
{
  std::vector<PathPoint> points;
  std::string error;

  bool ok() const noexcept { return error.empty() && points.size() >= 3U; }
};

PathLoadResult load_path_csv(
  const std::filesystem::path & path, const CsvColumns & columns);

}  // namespace lawn_mower_tracker

#endif  // LAWN_MOWER_TRACKER__PATH_LOADER_HPP_
