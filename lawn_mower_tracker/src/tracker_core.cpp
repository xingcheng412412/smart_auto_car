#include "lawn_mower_tracker/tracker_core.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace lawn_mower_tracker
{
namespace
{
double distance(const PathPoint & a, const PathPoint & b)
{
  return std::hypot(a.x - b.x, a.y - b.y);
}

bool finite_pose(const Pose2D & pose)
{
  return std::isfinite(pose.x) && std::isfinite(pose.y) && std::isfinite(pose.z) &&
         std::isfinite(pose.yaw);
}

double normalize_angle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}
}  // namespace

TrackerCore::TrackerCore(TrackerParams params) : params_(params)
{
  if (!std::isfinite(params_.wheelbase) || !std::isfinite(params_.lookahead_distance) ||
    !std::isfinite(params_.min_velocity) || !std::isfinite(params_.max_velocity) ||
    !std::isfinite(params_.max_steering_angle) || !std::isfinite(params_.slowdown_distance) ||
    !std::isfinite(params_.stop_distance) || !(params_.wheelbase > 0.0) ||
    !(params_.lookahead_distance > 0.0) || !(params_.max_velocity > 0.0) ||
    !(params_.max_steering_angle > 0.0) || params_.min_velocity < 0.0 ||
    params_.min_velocity > params_.max_velocity || !(params_.stop_distance > 0.0) ||
    params_.slowdown_distance < params_.stop_distance || params_.search_forward_points == 0U ||
    !std::isfinite(params_.steering_gain) || params_.steering_gain <= 0.0 ||
    !std::isfinite(params_.max_slope_tangent) || params_.max_slope_tangent < 0.0 ||
    !std::isfinite(params_.direction_switch_distance) || params_.direction_switch_distance <= 0.0 ||
    !std::isfinite(params_.initial_search_distance) || params_.initial_search_distance <= 0.0 ||
    params_.initial_search_points == 0U || !std::isfinite(params_.max_start_distance) ||
    params_.max_start_distance <= 0.0 ||
    !std::isfinite(params_.terminal_alignment_distance) ||
    params_.terminal_alignment_distance <= params_.stop_distance ||
    !std::isfinite(params_.terminal_yaw_tolerance) ||
    params_.terminal_yaw_tolerance <= 0.0 || params_.terminal_yaw_tolerance > M_PI)
  {
    throw std::invalid_argument("invalid tracker parameters");
  }
}

std::string TrackerCore::start(std::vector<PathPoint> path)
{
  if (path.size() < 3U) {
    state_ = TrackerState::kError;
    return "path requires at least three points";
  }
  for (const auto & point : path) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z) ||
      (point.has_yaw && !std::isfinite(point.yaw)))
    {
      state_ = TrackerState::kError;
      return "path contains a non-finite point";
    }
  }
  path_ = std::move(path);
  initial_search_last_index_ =
    std::min(path_.size() - 1U, params_.initial_search_points - 1U);
  double initial_arc_length = 0.0;
  for (std::size_t index = 0U; index + 1U < path_.size(); ++index) {
    initial_arc_length += distance(path_[index], path_[index + 1U]);
    if (initial_arc_length > params_.initial_search_distance) {
      break;
    }
    initial_search_last_index_ = std::max(initial_search_last_index_, index + 1U);
  }
  resolved_directions_.clear();
  direction_switch_ranges_.clear();

  const bool has_direction_metadata = std::any_of(
    path_.begin(), path_.end(), [](const PathPoint & point) {return point.direction != 0;});
  if (has_direction_metadata) {
    struct DirectionRun
    {
      std::size_t first;
      std::size_t last;
      int direction;
    };
    std::vector<DirectionRun> runs;
    std::size_t run_first = path_.size();
    int run_direction = 0;
    for (std::size_t index = 0U; index < path_.size(); ++index) {
      const int direction = path_[index].direction;
      if (direction == 0) {
        continue;
      }
      if (run_first == path_.size()) {
        run_first = index;
        run_direction = direction;
      } else if (direction != run_direction) {
        runs.push_back({run_first, index - 1U, run_direction});
        run_first = index;
        run_direction = direction;
      }
    }
    runs.push_back({run_first, path_.size() - 1U, run_direction});

    resolved_directions_.assign(path_.size(), runs.front().direction);
    int accepted_direction = runs.front().direction;
    std::size_t previous_accepted_first = runs.front().first;
    for (std::size_t run_index = 0U; run_index < runs.size(); ++run_index) {
      const auto & run = runs[run_index];
      double run_distance = 0.0;
      for (std::size_t index = run.first; index < run.last; ++index) {
        run_distance += distance(path_[index], path_[index + 1U]);
      }
      const bool accepted =
        run_index == 0U || run_distance >= params_.direction_switch_distance;
      if (accepted && run.direction != accepted_direction) {
        std::size_t switch_first = run.first > 0U ? run.first - 1U : 0U;
        std::size_t switch_last = run.first;
        for (std::size_t index = previous_accepted_first + 1U;
          index <= run.first; ++index)
        {
          if (path_[index - 1U].has_path_id && path_[index].has_path_id &&
            path_[index - 1U].path_id != path_[index].path_id &&
            distance(path_[index], path_[run.first]) <= params_.direction_switch_distance)
          {
            switch_first = std::min(switch_first, index - 1U);
            switch_last = std::max(switch_last, index);
          }
        }
        direction_switch_ranges_.push_back({switch_first, switch_last});
        accepted_direction = run.direction;
        previous_accepted_first = run.first;
      } else if (accepted) {
        previous_accepted_first = run.first;
      }
      std::fill(
        resolved_directions_.begin() + run.first,
        resolved_directions_.begin() + run.last + 1U,
        accepted_direction);
    }
    if (runs.front().first > 0U) {
      std::fill(
        resolved_directions_.begin(),
        resolved_directions_.begin() + runs.front().first,
        runs.front().direction);
      }
  } else {
    constexpr double kMinimumSegmentLength = 1e-6;
    std::size_t index = 1U;
    while (index + 1U < path_.size()) {
      std::size_t first = index;
      while (first > 0U && distance(path_[first], path_[first - 1U]) <= kMinimumSegmentLength) {
        --first;
      }
      std::size_t last = index;
      while (last + 1U < path_.size() &&
        distance(path_[last], path_[last + 1U]) <= kMinimumSegmentLength)
      {
        ++last;
      }
      if (first > 0U && last + 1U < path_.size()) {
        const double ax = path_[first].x - path_[first - 1U].x;
        const double ay = path_[first].y - path_[first - 1U].y;
        const double bx = path_[last + 1U].x - path_[last].x;
        const double by = path_[last + 1U].y - path_[last].y;
        const double denominator = std::hypot(ax, ay) * std::hypot(bx, by);
        if (denominator > 1e-9 && (ax * bx + ay * by) / denominator < -0.8) {
          direction_switch_ranges_.push_back({first - 1U, last + 1U});
        }
      }
      index = std::max(index + 1U, last + 1U);
    }
  }
  if (!resolve_terminal_yaw()) {
    state_ = TrackerState::kError;
    return "terminal yaw is unavailable";
  }
  pose_valid_ = false;
  progress_index_ = 0U;
  progress_initialized_ = false;
  travel_direction_ = 0;
  state_ = TrackerState::kTracking;
  return {};
}

bool TrackerCore::resolve_terminal_yaw()
{
  if (path_.back().has_yaw) {
    terminal_yaw_ = normalize_angle(path_.back().yaw);
    return true;
  }
  constexpr double kMinimumSegmentLength = 1e-6;
  for (std::size_t last = path_.size() - 1U; last > 0U; --last) {
    const double dx = path_[last].x - path_[last - 1U].x;
    const double dy = path_[last].y - path_[last - 1U].y;
    if (std::hypot(dx, dy) <= kMinimumSegmentLength) {
      continue;
    }
    double yaw = std::atan2(dy, dx);
    const int final_direction = resolved_directions_.empty() ? 0 : resolved_directions_.back();
    if (final_direction < 0) {
      yaw += M_PI;
    }
    terminal_yaw_ = normalize_angle(yaw);
    return true;
  }
  return false;
}

bool TrackerCore::update_pose(const Pose2D & pose)
{
  pose_valid_ = finite_pose(pose);
  if (pose_valid_) {
    pose_ = pose;
  }
  return pose_valid_;
}

std::size_t TrackerCore::direction_bounded_search_last(
  std::size_t unconstrained_last) const
{
  if (resolved_directions_.empty() || progress_index_ >= unconstrained_last) {
    return unconstrained_last;
  }

  int search_direction = resolved_directions_[progress_index_];
  bool crossed_direction_switch = false;
  for (std::size_t index = progress_index_ + 1U; index <= unconstrained_last; ++index) {
    if (resolved_directions_[index] == search_direction) {
      continue;
    }
    const bool switch_already_executed =
      travel_direction_ == resolved_directions_[index];
    if (crossed_direction_switch ||
      (!switch_already_executed && !can_cross_direction_switch(progress_index_, index)))
    {
      return index - 1U;
    }
    crossed_direction_switch = true;
    search_direction = resolved_directions_[index];
  }
  return unconstrained_last;
}

std::size_t TrackerCore::closest_index() const
{
  const std::size_t first = progress_initialized_ ?
    (progress_index_ > params_.search_back_points ? progress_index_ - params_.search_back_points : 0U) : 0U;
  const std::size_t unconstrained_last = progress_initialized_ ?
    std::min(path_.size() - 1U, progress_index_ + params_.search_forward_points) :
    initial_search_last_index_;
  const std::size_t last = progress_initialized_ ?
    direction_bounded_search_last(unconstrained_last) : unconstrained_last;
  std::size_t best = first;
  double best_distance = std::numeric_limits<double>::infinity();
  for (std::size_t index = first; index <= last; ++index) {
    const double candidate = std::hypot(path_[index].x - pose_.x, path_[index].y - pose_.y);
    if (candidate < best_distance) {
      best = index;
      best_distance = candidate;
    }
  }
  return best;
}

double TrackerCore::remaining_arc_length() const
{
  double remaining = std::hypot(
    path_[progress_index_].x - pose_.x, path_[progress_index_].y - pose_.y);
  for (std::size_t index = progress_index_; index + 1U < path_.size(); ++index) {
    remaining += distance(path_[index], path_[index + 1U]);
  }
  return remaining;
}

bool TrackerCore::near_direction_switch(std::size_t index) const
{
  for (const auto & range : direction_switch_ranges_) {
    if (index >= range.first && index <= range.last) {
      return true;
    }

    double arc_to_switch = 0.0;
    if (index < range.first) {
      for (std::size_t point_index = index; point_index < range.first; ++point_index) {
        arc_to_switch += distance(path_[point_index], path_[point_index + 1U]);
        if (arc_to_switch > params_.direction_switch_distance) {
          break;
        }
      }
    } else {
      for (std::size_t point_index = range.last; point_index < index; ++point_index) {
        arc_to_switch += distance(path_[point_index], path_[point_index + 1U]);
        if (arc_to_switch > params_.direction_switch_distance) {
          break;
        }
      }
    }
    if (arc_to_switch > params_.direction_switch_distance) {
      continue;
    }

    for (std::size_t point_index = range.first; point_index <= range.last; ++point_index) {
      if (std::hypot(path_[point_index].x - pose_.x, path_[point_index].y - pose_.y) <=
        params_.direction_switch_distance)
      {
        return true;
      }
    }
  }
  return false;
}

bool TrackerCore::can_cross_direction_switch(
  std::size_t index, std::size_t boundary_index) const
{
  for (const auto & range : direction_switch_ranges_) {
    if (boundary_index < range.first || boundary_index > range.last) {
      continue;
    }
    if (index >= range.first && index <= range.last) {
      return true;
    }
    if (index > range.last) {
      return false;
    }

    double arc_to_switch = 0.0;
    for (std::size_t point_index = index; point_index < range.first; ++point_index) {
      arc_to_switch += distance(path_[point_index], path_[point_index + 1U]);
      if (arc_to_switch > params_.direction_switch_distance) {
        return false;
      }
    }
    for (std::size_t point_index = range.first; point_index <= range.last; ++point_index) {
      if (std::hypot(path_[point_index].x - pose_.x, path_[point_index].y - pose_.y) <=
        params_.direction_switch_distance)
      {
        return true;
      }
    }
    return false;
  }
  return false;
}

std::size_t TrackerCore::lookahead_index(std::size_t closest) const
{
  double arc = 0.0;
  int lookahead_direction = resolved_directions_.empty() ? 0 : resolved_directions_[closest];
  bool crossed_direction_switch = false;
  for (std::size_t index = closest; index + 1U < path_.size(); ++index) {
    if (!resolved_directions_.empty() &&
      resolved_directions_[index + 1U] != lookahead_direction)
    {
      if (crossed_direction_switch ||
        !can_cross_direction_switch(closest, index + 1U))
      {
        return index;
      }
      crossed_direction_switch = true;
      lookahead_direction = resolved_directions_[index + 1U];
    }
    arc += distance(path_[index], path_[index + 1U]);
    if (arc >= params_.lookahead_distance) {
      return index + 1U;
    }
  }
  return path_.size() - 1U;
}

ControlResult TrackerCore::step()
{
  if (state_ != TrackerState::kTracking) {
    return {};
  }
  if (!pose_valid_) {
    return {0.0, 0.0, false, "pose is unavailable or invalid"};
  }

  const auto candidate = closest_index();
  if (!progress_initialized_ && std::hypot(
      path_[candidate].x - pose_.x, path_[candidate].y - pose_.y) > params_.max_start_distance)
  {
    return {0.0, 0.0, false, "RTK pose is too far from the path start"};
  }
  progress_index_ = progress_initialized_ ? std::max(progress_index_, candidate) : candidate;
  progress_initialized_ = true;
  const auto & endpoint = path_.back();
  const double endpoint_distance = std::hypot(endpoint.x - pose_.x, endpoint.y - pose_.y);
  const double remaining = remaining_arc_length();
  if (endpoint_distance <= params_.stop_distance && remaining <= params_.stop_distance) {
    const double terminal_yaw_error = normalize_angle(terminal_yaw_ - pose_.yaw);
    if (std::abs(terminal_yaw_error) <= params_.terminal_yaw_tolerance) {
      state_ = TrackerState::kCompleted;
      return {0.0, 0.0, true, {}};
    }
    state_ = TrackerState::kError;
    return {0.0, 0.0, false, "endpoint reached but terminal yaw is outside tolerance"};
  }

  const auto target_index = lookahead_index(progress_index_);
  const auto & target = path_[target_index];
  const double dx = target.x - pose_.x;
  const double dy = target.y - pose_.y;
  const double local_x = std::cos(pose_.yaw) * dx + std::sin(pose_.yaw) * dy;
  const int geometry_direction = local_x >= 0.0 ? 1 : -1;
  const bool at_direction_switch = near_direction_switch(progress_index_);
  int metadata_direction = resolved_directions_.empty() ?
    0 : resolved_directions_[progress_index_];
  if (at_direction_switch && !resolved_directions_.empty() &&
    resolved_directions_[target_index] != metadata_direction)
  {
    metadata_direction = resolved_directions_[target_index];
  }
  const int direction_sign = metadata_direction == 0 ? geometry_direction : metadata_direction;

  const double path_heading_error = normalize_angle(std::atan2(dy, dx) - pose_.yaw);
  // Pure pursuit blends directions of travel. Recorded terminal_yaw_ is
  // body orientation, so reverse travel must use its opposite direction.
  // The completion check above deliberately keeps the recorded body yaw.
  const double terminal_travel_yaw = terminal_yaw_ +
    (direction_sign < 0 ? std::acos(-1.0) : 0.0);
  const double terminal_heading_error = normalize_angle(terminal_travel_yaw - pose_.yaw);
  const double terminal_weight = std::clamp(
    1.0 - remaining / params_.terminal_alignment_distance, 0.0, 1.0);
  const double alpha = normalize_angle(
    path_heading_error + terminal_weight *
    normalize_angle(terminal_heading_error - path_heading_error));
  const double target_distance = std::max(std::hypot(dx, dy), 1e-6);
  double steering = params_.steering_gain *
    std::atan2(2.0 * params_.wheelbase * std::sin(alpha), target_distance);
  steering = std::clamp(steering, -params_.max_steering_angle, params_.max_steering_angle);

  const bool changing_direction =
    travel_direction_ != 0 && direction_sign != travel_direction_;
  if (changing_direction && !at_direction_switch) {
    return {0.0, steering, false, "direction change requested away from a path reversal"};
  }
  // Retire the old run when a legal cross-boundary direction is selected,
  // including initial selection near a reversal. Moving out of the switch
  // radius must not restore the old run's direction on the next cycle.
  if (!resolved_directions_.empty() &&
    resolved_directions_[progress_index_] != direction_sign)
  {
    for (std::size_t index = progress_index_ + 1U; index <= target_index; ++index) {
      if (resolved_directions_[index] == direction_sign) {
        progress_index_ = index;
        break;
      }
    }
  }
  if (changing_direction) {
    travel_direction_ = direction_sign;
    return {0.0, steering, false, {}};
  }
  travel_direction_ = direction_sign;
  double speed = params_.max_velocity;
  if (remaining < params_.slowdown_distance) {
    const double ratio = remaining / params_.slowdown_distance;
    speed = std::max(params_.min_velocity, params_.max_velocity * ratio);
  }
  const double horizontal = std::max(std::hypot(dx, dy), 1e-6);
  if (std::abs(target.z - pose_.z) / horizontal > params_.max_slope_tangent) {
    speed = params_.min_velocity;
  }
  return {static_cast<double>(direction_sign) * speed, steering, false, {}};
}

Pose2D TrackerCore::local_goal(double max_distance) const
{
  if (path_.empty() || !pose_valid_ || !std::isfinite(max_distance) || max_distance <= 0) {
    return {};
  }
  PathPoint target = path_[progress_index_];
  double arc = 0.0;
  double heading = pose_.yaw;
  for (std::size_t i = progress_index_; i + 1 < path_.size(); ++i) {
    if ((!resolved_directions_.empty() && resolved_directions_[i + 1] != resolved_directions_[i]) ||
      (resolved_directions_.empty() && near_direction_switch(i) && i > progress_index_))
    {
      break;
    }
    const double segment = distance(path_[i], path_[i + 1]);
    if (segment <= 1e-6) {continue;}
    const double fraction = std::min(1.0, (max_distance - arc) / segment);
    target.x = path_[i].x + fraction * (path_[i + 1].x - path_[i].x);
    target.y = path_[i].y + fraction * (path_[i + 1].y - path_[i].y);
    heading = std::atan2(path_[i + 1].y - path_[i].y, path_[i + 1].x - path_[i].x) +
      (travel_direction_ < 0 ? M_PI : 0.0);
    arc += fraction * segment;
    if (arc >= max_distance) {break;}
  }
  const double dx = target.x - pose_.x, dy = target.y - pose_.y;
  return {std::cos(pose_.yaw) * dx + std::sin(pose_.yaw) * dy,
    -std::sin(pose_.yaw) * dx + std::cos(pose_.yaw) * dy,
    0, normalize_angle(heading - pose_.yaw)};
}

void TrackerCore::pause()
{
  if (state_ == TrackerState::kTracking) {
    state_ = TrackerState::kPaused;
  }
}

bool TrackerCore::resume()
{
  if (state_ != TrackerState::kPaused) {
    return false;
  }
  state_ = TrackerState::kTracking;
  return true;
}

void TrackerCore::stop()
{
  path_.clear();
  resolved_directions_.clear();
  direction_switch_ranges_.clear();
  pose_valid_ = false;
  progress_initialized_ = false;
  travel_direction_ = 0;
  state_ = TrackerState::kStopped;
}

}  // namespace lawn_mower_tracker
