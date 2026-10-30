#ifndef LAWN_MOWER_TRACKER__OBSTACLE_PLANNER_HPP_
#define LAWN_MOWER_TRACKER__OBSTACLE_PLANNER_HPP_
#include <vector>
#include "lawn_mower_tracker/types.hpp"
namespace lawn_mower_tracker
{
struct ObstaclePoint {double x, y, z;};
struct ObstacleParams
{
  double optical_height{0.518976069};
  // Includes wheels, body, support and lidar, with a small self-return tolerance.
  double body_half_length{0.30}, body_half_width{0.19};
  double body_min_z{-0.09}, body_max_z{0.57};
  double min_height{-0.04}, max_height{0.55};
  double safety_margin{0.12}, resolution{0.12}, range{5.0};
  double wheelbase{0.36}, max_steering{0.942477796};
  int max_expansions{18000};
};
struct AvoidanceResult
{
  bool safe{false};
  double steering{0};
  std::vector<Pose2D> path;
};
class ObstaclePlanner
{
public:
  explicit ObstaclePlanner(ObstacleParams params = {});
  std::vector<ObstaclePoint> filter(const std::vector<ObstaclePoint> & points) const;
  AvoidanceResult plan(const std::vector<ObstaclePoint> & obstacles,
    const Pose2D & goal, double direction, const Pose2D & start = {}) const;
  AvoidanceResult check_motion(const std::vector<ObstaclePoint> & obstacles,
    const Pose2D & start, double speed, double steering, double horizon = 0) const;
private:
  ObstacleParams params_;
};
}
#endif
