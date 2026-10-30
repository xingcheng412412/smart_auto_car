#ifndef LAWN_MOWER_TRACKER__TYPES_HPP_
#define LAWN_MOWER_TRACKER__TYPES_HPP_

#include <cstdint>
#include <string>
#include <vector>

namespace lawn_mower_tracker
{

struct PathPoint
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
  int direction{0};
  std::int64_t path_id{0};
  bool has_path_id{false};
  double yaw{0.0};
  bool has_yaw{false};
};

struct Pose2D
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double yaw{0.0};
};

}  // namespace lawn_mower_tracker

#endif  // LAWN_MOWER_TRACKER__TYPES_HPP_
