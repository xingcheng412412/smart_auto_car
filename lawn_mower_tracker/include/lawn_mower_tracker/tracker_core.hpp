#ifndef LAWN_MOWER_TRACKER__TRACKER_CORE_HPP_
#define LAWN_MOWER_TRACKER__TRACKER_CORE_HPP_

#include <string>
#include <vector>

#include "lawn_mower_tracker/types.hpp"

namespace lawn_mower_tracker
{

enum class TrackerState {kIdle, kTracking, kPaused, kStopped, kCompleted, kError};

struct TrackerParams
{
  double wheelbase{1.45};
  double lookahead_distance{2.0};
  double min_velocity{0.3};
  double max_velocity{1.2};
  double max_steering_angle{0.942477796};
  double slowdown_distance{2.0};
  double stop_distance{0.2};
  std::size_t search_back_points{5U};
  std::size_t search_forward_points{30U};
  double steering_gain{1.0};
  double max_slope_tangent{0.3};
  double direction_switch_distance{0.5};
  double initial_search_distance{10.0};
  std::size_t initial_search_points{10U};
  double max_start_distance{3.0};
  double terminal_alignment_distance{3.0};
  double terminal_yaw_tolerance{0.174532925};
};

struct ControlResult
{
  double linear_velocity{0.0};
  double steering_angle{0.0};
  bool completed{false};
  std::string error;
};

class TrackerCore
{
public:
  explicit TrackerCore(TrackerParams params = {});

  std::string start(std::vector<PathPoint> path);
  bool update_pose(const Pose2D & pose);
  ControlResult step();
  Pose2D local_goal(double distance) const;
  void pause();
  bool resume();
  void stop();
  TrackerState state() const noexcept { return state_; }
  std::size_t progress_index() const noexcept { return progress_index_; }

private:
  struct DirectionSwitchRange
  {
    std::size_t first{0U};
    std::size_t last{0U};
  };

  std::size_t direction_bounded_search_last(std::size_t unconstrained_last) const;
  std::size_t closest_index() const;
  double remaining_arc_length() const;
  bool near_direction_switch(std::size_t index) const;
  bool can_cross_direction_switch(std::size_t index, std::size_t boundary_index) const;
  std::size_t lookahead_index(std::size_t closest) const;
  bool resolve_terminal_yaw();

  TrackerParams params_;
  std::vector<PathPoint> path_;
  Pose2D pose_;
  bool pose_valid_{false};
  TrackerState state_{TrackerState::kIdle};
  std::size_t progress_index_{0U};
  std::size_t initial_search_last_index_{0U};
  bool progress_initialized_{false};
  int travel_direction_{0};
  std::vector<int> resolved_directions_;
  std::vector<DirectionSwitchRange> direction_switch_ranges_;
  double terminal_yaw_{0.0};
};

}  // namespace lawn_mower_tracker

#endif  // LAWN_MOWER_TRACKER__TRACKER_CORE_HPP_
