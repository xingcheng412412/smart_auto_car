#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <vector>

#include "lawn_mower_tracker/path_loader.hpp"
#include "lawn_mower_tracker/tracker_core.hpp"

using lawn_mower_tracker::PathPoint;
using lawn_mower_tracker::Pose2D;
using lawn_mower_tracker::TrackerCore;
using lawn_mower_tracker::TrackerState;

namespace {

std::vector<PathPoint> straight_path()
{
  return {{0, 0, 0}, {1, 0, 0}, {2, 0, 0}, {3, 0, 0}, {4, 0, 0}};
}

PathPoint point_with_yaw(double x, double y, double yaw, int direction = 1)
{
  PathPoint point;
  point.x = x;
  point.y = y;
  point.yaw = yaw;
  point.has_yaw = true;
  point.direction = direction;
  return point;
}

TEST(TrackerCoreTest, TracksStraightPathWithoutSteering)
{
  TrackerCore core;
  ASSERT_TRUE(core.start(straight_path()).empty());
  core.update_pose({0, 0, 0, 0});
  const auto result = core.step();
  EXPECT_GT(result.linear_velocity, 0.0);
  EXPECT_NEAR(result.steering_angle, 0.0, 1e-9);
  EXPECT_FALSE(result.completed);
}

TEST(TrackerCoreTest, SteersTowardLeftAndRightLookaheadPoints)
{
  TrackerCore left;
  ASSERT_TRUE(left.start({{0, 0, 0}, {1, 0.2, 0}, {2, 1, 0}, {3, 2, 0}}).empty());
  left.update_pose({0, 0, 0, 0});
  EXPECT_GT(left.step().steering_angle, 0.0);

  TrackerCore right;
  ASSERT_TRUE(right.start({{0, 0, 0}, {1, -0.2, 0}, {2, -1, 0}, {3, -2, 0}}).empty());
  right.update_pose({0, 0, 0, 0});
  EXPECT_LT(right.step().steering_angle, 0.0);
}

TEST(TrackerCoreTest, PauseResumeAndStopAreExplicitStates)
{
  TrackerCore core;
  ASSERT_TRUE(core.start(straight_path()).empty());
  core.pause();
  EXPECT_EQ(core.state(), TrackerState::kPaused);
  EXPECT_DOUBLE_EQ(core.step().linear_velocity, 0.0);
  EXPECT_TRUE(core.resume());
  EXPECT_EQ(core.state(), TrackerState::kTracking);
  core.stop();
  EXPECT_EQ(core.state(), TrackerState::kStopped);
  EXPECT_DOUBLE_EQ(core.step().linear_velocity, 0.0);
}

TEST(TrackerCoreTest, RejectsShortPath)
{
  TrackerCore core;
  EXPECT_FALSE(core.start({{0, 0, 0}, {1, 0, 0}}).empty());
  EXPECT_EQ(core.state(), TrackerState::kError);
}

TEST(TrackerCoreTest, CompletesInsideStopDistance)
{
  TrackerCore core;
  ASSERT_TRUE(core.start(straight_path()).empty());
  core.update_pose({3.9, 0, 0, 0});
  const auto result = core.step();
  EXPECT_TRUE(result.completed);
  EXPECT_DOUBLE_EQ(result.linear_velocity, 0.0);
  EXPECT_EQ(core.state(), TrackerState::kCompleted);
}

TEST(TrackerCoreTest, CompletesOnlyWhenRecordedTerminalYawIsWithinTolerance)
{
  TrackerCore core;
  ASSERT_TRUE(core.start({
    point_with_yaw(0, 0, 0), point_with_yaw(1, 0, 0), point_with_yaw(2, 0, 0)}).empty());
  core.update_pose({2.0, 0.0, 0.0, 0.15});
  EXPECT_TRUE(core.step().completed);
}

TEST(TrackerCoreTest, StopsWithoutCompletionWhenTerminalYawIsOutsideTolerance)
{
  TrackerCore core;
  ASSERT_TRUE(core.start({
    point_with_yaw(0, 0, 0), point_with_yaw(1, 0, 0), point_with_yaw(2, 0, 0)}).empty());
  core.update_pose({2.0, 0.0, 0.0, 0.4});
  const auto result = core.step();
  EXPECT_FALSE(result.completed);
  EXPECT_DOUBLE_EQ(result.linear_velocity, 0.0);
  EXPECT_DOUBLE_EQ(result.steering_angle, 0.0);
  EXPECT_EQ(result.error, "endpoint reached but terminal yaw is outside tolerance");
  EXPECT_EQ(core.state(), TrackerState::kError);
}

TEST(TrackerCoreTest, NormalizesTerminalYawAcrossPiBoundary)
{
  TrackerCore core;
  ASSERT_TRUE(core.start({
    point_with_yaw(0, 0, 3.124139361), point_with_yaw(1, 0, 3.124139361),
    point_with_yaw(2, 0, 3.124139361)}).empty());
  core.update_pose({2.0, 0.0, 0.0, -3.124139361});
  EXPECT_TRUE(core.step().completed);
}

TEST(TrackerCoreTest, LegacyTerminalYawSkipsStationaryDuplicates)
{
  TrackerCore core;
  ASSERT_TRUE(core.start({{0, 0, 0}, {1, 1, 0}, {2, 2, 0}, {2, 2, 0}, {2, 2, 0}}).empty());
  core.update_pose({2, 2, 0, 0.7853981633974483});
  EXPECT_TRUE(core.step().completed);
}

TEST(TrackerCoreTest, LegacyReverseTerminalYawUsesVehicleBodyOrientation)
{
  TrackerCore core;
  ASSERT_TRUE(core.start({
    {2, 0, 0, -1}, {1, 0, 0, -1}, {0, 0, 0, -1}}).empty());
  core.update_pose({0, 0, 0, 0});
  EXPECT_TRUE(core.step().completed);
}

TEST(TrackerCoreTest, RejectsInvalidTerminalYawParameters)
{
  lawn_mower_tracker::TrackerParams params;
  params.terminal_yaw_tolerance = 0.0;
  EXPECT_THROW(TrackerCore invalid_tolerance(params), std::invalid_argument);
  params.terminal_yaw_tolerance = 0.174532925;
  params.terminal_alignment_distance = params.stop_distance;
  EXPECT_THROW(TrackerCore invalid_distance(params), std::invalid_argument);
}

TEST(TrackerCoreTest, IncreasesTerminalYawSteeringDuringFinalApproach)
{
  lawn_mower_tracker::TrackerParams params;
  params.lookahead_distance = 0.5;
  params.terminal_alignment_distance = 3.0;
  TrackerCore core(params);
  ASSERT_TRUE(core.start({
    point_with_yaw(0, 0, 0), point_with_yaw(1, 0, 0),
    point_with_yaw(2, 0, 0), point_with_yaw(3, 0, 0),
    point_with_yaw(4, 0, 0), point_with_yaw(5, 0, 0),
    point_with_yaw(6, 0, 1.0)}).empty());

  core.update_pose({2.0, 0, 0, 0});
  EXPECT_NEAR(core.step().steering_angle, 0.0, 1e-9);
  core.update_pose({4.0, 0, 0, 0});
  const double early = core.step().steering_angle;
  core.update_pose({5.0, 0, 0, 0});
  const double late = core.step().steering_angle;
  EXPECT_GT(early, 0.0);
  EXPECT_GT(late, early);
}

TEST(TrackerCoreTest, DoesNotCompleteNearEndpointBeforeFinalSegment)
{
  TrackerCore core;
  ASSERT_TRUE(core.start({
    {0, 0, 0}, {2, 0, 0}, {2, 2, 0}, {0, 2, 0}, {0.1, 0.1, 0}}).empty());
  core.update_pose({0, 0, 0, 0});
  EXPECT_FALSE(core.step().completed);
  EXPECT_EQ(core.state(), TrackerState::kTracking);
}

TEST(TrackerCoreTest, CompletesWithTrailingStationaryDuplicatePoints)
{
  TrackerCore core;
  ASSERT_TRUE(core.start({
    {0, 0, 0},
    {1, 0, 0},
    {2, 0, 0},
    {2, 0, 0},
    {2, 0, 0},
    {2, 0, 0}}).empty());
  core.update_pose({2, 0, 0, 0});
  const auto result = core.step();
  EXPECT_TRUE(result.completed);
  EXPECT_TRUE(result.error.empty()) << result.error;
  EXPECT_EQ(core.state(), TrackerState::kCompleted);
}

TEST(TrackerCoreTest, RetainsProgressAtAPathCrossing)
{
  lawn_mower_tracker::TrackerParams params;
  params.search_back_points = 1;
  params.search_forward_points = 2;
  TrackerCore core(params);
  ASSERT_TRUE(core.start({
    {-2, -2, 0}, {-1, -1, 0}, {0, 0, 0}, {1, 1, 0}, {2, 2, 0},
    {1, -1, 0}, {0, 0, 0}, {-1, 1, 0}, {-2, 2, 0}}).empty());
  core.update_pose({-1, -1, 0, 0.785398163});
  core.step();
  const auto before = core.progress_index();
  core.update_pose({0.05, 0.05, 0, 0.785398163});
  core.step();
  EXPECT_LE(core.progress_index(), before + params.search_forward_points);
  EXPECT_LT(core.progress_index(), 6U);
}

TEST(TrackerCoreTest, RejectsNonFinitePose)
{
  TrackerCore core;
  ASSERT_TRUE(core.start(straight_path()).empty());
  EXPECT_FALSE(core.update_pose({NAN, 0, 0, 0}));
  EXPECT_FALSE(core.step().error.empty());
}

TEST(TrackerCoreTest, RejectsNonFiniteRecordedPathYaw)
{
  auto path = straight_path();
  path.back().yaw = NAN;
  path.back().has_yaw = true;
  TrackerCore core;
  EXPECT_FALSE(core.start(path).empty());
  EXPECT_EQ(core.state(), TrackerState::kError);
}

TEST(TrackerCoreTest, StopsForOneCycleBeforeChangingDirection)
{
  TrackerCore core;
  ASSERT_TRUE(core.start({{0, 0, 0}, {1, 0, 0}, {2, 0, 0}, {1, 0, 0}, {0, 0, 0}}).empty());
  core.update_pose({0, 0, 0, 0});
  EXPECT_GT(core.step().linear_velocity, 0.0);
  core.update_pose({2, 0, 0, 0});
  EXPECT_DOUBLE_EQ(core.step().linear_velocity, 0.0);
  EXPECT_LT(core.step().linear_velocity, 0.0);
}

TEST(TrackerCoreTest, UsesMetadataToReverseAcrossStationaryDuplicatePoints)
{
  lawn_mower_tracker::TrackerParams params;
  params.lookahead_distance = 0.5;
  TrackerCore core(params);
  ASSERT_TRUE(core.start({
    {0, 0, 0, 1, 1, true},
    {1, 0, 0, 1, 1, true},
    {2, 0, 0, 1, 1, true},
    {2, 0, 0, 0, 1, true},
    {2, 0, 0, 0, 2, true},
    {2, 0, 0, 0, 2, true},
    {1.5, 0, 0, -1, 2, true},
    {1, 0, 0, -1, 2, true},
    {0, 0, 0, -1, 2, true}}).empty());

  core.update_pose({0, 0, 0, 0});
  EXPECT_GT(core.step().linear_velocity, 0.0);
  core.update_pose({2, 0, 0, 0});
  const auto switch_result = core.step();
  EXPECT_TRUE(switch_result.error.empty()) << switch_result.error;
  EXPECT_DOUBLE_EQ(switch_result.linear_velocity, 0.0);
  const auto reverse_result = core.step();
  EXPECT_TRUE(reverse_result.error.empty()) << reverse_result.error;
  EXPECT_LT(reverse_result.linear_velocity, 0.0);
}

TEST(TrackerCoreTest, IgnoresShortDirectionNoiseNearAStationaryPoint)
{
  lawn_mower_tracker::TrackerParams params;
  params.lookahead_distance = 2.0;
  params.direction_switch_distance = 0.5;
  TrackerCore core(params);
  ASSERT_TRUE(core.start({
    {0, 0, 0, 1, 1, true},
    {1, 0, 0, 1, 1, true},
    {2, 0, 0, -1, 1, true},
    {2.01, 0, 0, -1, 1, true},
    {2.02, 0, 0, 1, 1, true},
    {3, 0, 0, 1, 1, true},
    {4, 0, 0, 1, 1, true}}).empty());

  core.update_pose({0, 0, 0, 0});
  const auto result = core.step();
  EXPECT_TRUE(result.error.empty()) << result.error;
  EXPECT_GT(result.linear_velocity, 0.0);
}

TEST(TrackerCoreTest, DoesNotAdoptLookaheadDirectionBeforeTheSwitchRange)
{
  lawn_mower_tracker::TrackerParams params;
  params.lookahead_distance = 4.5;
  params.direction_switch_distance = 0.5;
  TrackerCore core(params);
  ASSERT_TRUE(core.start({
    {0, 0, 0, 1, 1, true},
    {1, 0, 0, 1, 1, true},
    {2, 0, 0, 1, 1, true},
    {2, 0, 0, 0, 1, true},
    {2, 0, 0, 0, 2, true},
    {1.5, 0, 0, -1, 2, true},
    {1, 0, 0, -1, 2, true},
    {0, 0, 0, -1, 2, true},
    {-1, 0, 0, -1, 2, true},
    {-2, 0, 0, -1, 2, true}}).empty());

  core.update_pose({0, 0, 0, 0});
  const auto result = core.step();
  EXPECT_TRUE(result.error.empty()) << result.error;
  EXPECT_GT(result.linear_velocity, 0.0);
}

TEST(TrackerCoreTest, IgnoresSpatiallyCloseFutureSwitchUntilPathProgressReachesIt)
{
  lawn_mower_tracker::TrackerParams params;
  params.lookahead_distance = 4.0;
  params.direction_switch_distance = 0.5;
  TrackerCore core(params);
  ASSERT_TRUE(core.start({
    {0.0, 0.0, 0.0, 1, 1, true},
    {1.0, 0.0, 0.0, 1, 1, true},
    {2.0, 0.0, 0.0, 1, 1, true},
    {2.0, 1.0, 0.0, 1, 1, true},
    {1.0, 1.0, 0.0, 1, 1, true},
    {1.0, 0.2, 0.0, -1, 1, true},
    {1.0, 1.0, 0.0, -1, 1, true},
    {1.0, 1.8, 0.0, -1, 1, true}}).empty());

  core.update_pose({1.0, 0.0, 0.0, 0.0});
  const auto early = core.step();
  EXPECT_TRUE(early.error.empty()) << early.error;
  EXPECT_GT(early.linear_velocity, 0.0);

  core.update_pose({2.0, 0.0, 0.0, 0.0});
  ASSERT_TRUE(core.step().error.empty());
  core.update_pose({2.0, 1.0, 0.0, 0.0});
  ASSERT_TRUE(core.step().error.empty());
  core.update_pose({1.0, 1.0, 0.0, 0.0});
  const auto switching = core.step();
  EXPECT_TRUE(switching.error.empty()) << switching.error;
  EXPECT_DOUBLE_EQ(switching.linear_velocity, 0.0);
  core.update_pose({1.0, 0.2, 0.0, 0.0});
  const auto reversed = core.step();
  EXPECT_TRUE(reversed.error.empty()) << reversed.error;
  EXPECT_LT(reversed.linear_velocity, 0.0);
}

TEST(TrackerCoreTest, KeepsRecentSwitchActiveWhileReverseHeadingSettles)
{
  lawn_mower_tracker::TrackerParams params;
  params.lookahead_distance = 0.5;
  params.direction_switch_distance = 0.5;
  TrackerCore core(params);
  ASSERT_TRUE(core.start({
    {0.0, 0.0, 0.0, 1, 1, true},
    {1.0, 0.0, 0.0, 1, 1, true},
    {2.0, 0.0, 0.0, 1, 1, true},
    {1.6, 0.0, 0.0, -1, 1, true},
    {1.2, 0.0, 0.0, -1, 1, true},
    {0.8, 0.0, 0.0, -1, 1, true},
    {0.4, 0.0, 0.0, -1, 1, true},
    {0.0, 0.0, 0.0, -1, 1, true}}).empty());

  core.update_pose({0.0, 0.0, 0.0, 0.0});
  EXPECT_GT(core.step().linear_velocity, 0.0);
  core.update_pose({1.6, 0.0, 0.0, 0.0});
  EXPECT_DOUBLE_EQ(core.step().linear_velocity, 0.0);
  EXPECT_LT(core.step().linear_velocity, 0.0);

  core.update_pose({1.2, 0.0, 0.0, 3.14159265358979323846});
  const auto settling = core.step();
  EXPECT_TRUE(settling.error.empty()) << settling.error;
  EXPECT_LT(settling.linear_velocity, 0.0);
}

TEST(TrackerCoreTest, DoesNotSkipReverseRunWhenLookaheadSpansTwoSwitches)
{
  lawn_mower_tracker::TrackerParams params;
  params.lookahead_distance = 2.0;
  params.direction_switch_distance = 0.5;
  TrackerCore core(params);
  ASSERT_TRUE(core.start({
    {0.0, 0.0, 0.0, 1, 1, true},
    {1.0, 0.0, 0.0, 1, 1, true},
    {2.0, 0.0, 0.0, 1, 1, true},
    {1.8, 0.0, 0.0, -1, 1, true},
    {1.4, 0.0, 0.0, -1, 1, true},
    {1.0, 0.0, 0.0, -1, 1, true},
    {1.4, 0.0, 0.0, 1, 1, true},
    {1.8, 0.0, 0.0, 1, 1, true},
    {2.2, 0.0, 0.0, 1, 1, true}}).empty());

  core.update_pose({0.0, 0.0, 0.0, 0.0});
  EXPECT_GT(core.step().linear_velocity, 0.0);
  core.update_pose({1.8, 0.0, 0.0, 0.0});
  EXPECT_DOUBLE_EQ(core.step().linear_velocity, 0.0);
  EXPECT_LT(core.step().linear_velocity, 0.0);
}

TEST(TrackerCoreTest, TrustsForwardMetadataThroughATightTurn)
{
  lawn_mower_tracker::TrackerParams params;
  params.lookahead_distance = 2.0;
  TrackerCore core(params);
  ASSERT_TRUE(core.start({
    {0.0, 0.0, 0.0, 1, 1, true},
    {1.0, 0.0, 0.0, 1, 1, true},
    {2.0, 0.0, 0.0, 1, 1, true},
    {2.0, 1.0, 0.0, 1, 1, true},
    {1.0, 1.0, 0.0, 1, 1, true},
    {0.0, 1.0, 0.0, 1, 1, true}}).empty());

  core.update_pose({2.0, 0.0, 0.0, 0.0});
  const auto result = core.step();
  EXPECT_TRUE(result.error.empty()) << result.error;
  EXPECT_GT(result.linear_velocity, 0.0);
}

TEST(TrackerCoreTest, GeometryFallbackSkipsStationaryDuplicatePoints)
{
  lawn_mower_tracker::TrackerParams params;
  params.lookahead_distance = 0.5;
  TrackerCore core(params);
  ASSERT_TRUE(core.start({
    {0, 0, 0},
    {1, 0, 0},
    {2, 0, 0},
    {2, 0, 0},
    {2, 0, 0},
    {2, 0, 0},
    {1.5, 0, 0},
    {1, 0, 0},
    {0, 0, 0}}).empty());

  core.update_pose({0, 0, 0, 0});
  EXPECT_GT(core.step().linear_velocity, 0.0);
  core.update_pose({2, 0, 0, 0});
  const auto switch_result = core.step();
  EXPECT_TRUE(switch_result.error.empty()) << switch_result.error;
  EXPECT_DOUBLE_EQ(switch_result.linear_velocity, 0.0);
  const auto reverse_result = core.step();
  EXPECT_TRUE(reverse_result.error.empty()) << reverse_result.error;
  EXPECT_LT(reverse_result.linear_velocity, 0.0);
}

TEST(TrackerCoreTest, RejectsNonFiniteConfiguration)
{
  lawn_mower_tracker::TrackerParams params;
  params.max_steering_angle = NAN;
  EXPECT_THROW(TrackerCore core(params), std::invalid_argument);
  params = {};
  params.slowdown_distance = NAN;
  EXPECT_THROW(TrackerCore core(params), std::invalid_argument);
}

TEST(TrackerCoreTest, InitializesOnlyWithinConfiguredStartWindow)
{
  lawn_mower_tracker::TrackerParams params;
  params.initial_search_points = 4U;
  params.max_start_distance = 2.0;
  std::vector<PathPoint> path;
  for (int index = 0; index < 20; ++index) {
    path.push_back({static_cast<double>(index), 0, 0});
  }
  path.push_back({0.01, 0, 0});
  TrackerCore core(params);
  ASSERT_TRUE(core.start(path).empty());
  core.update_pose({0.01, 0, 0, 0});
  ASSERT_TRUE(core.step().error.empty());
  EXPECT_LT(core.progress_index(), params.initial_search_points);
}

TEST(TrackerCoreTest, InitializesAtNearestPointWithinStartDistancePrefix)
{
  lawn_mower_tracker::TrackerParams params;
  params.initial_search_points = 3U;
  params.initial_search_distance = 5.0;
  params.max_start_distance = 1.0;
  std::vector<PathPoint> path;
  for (int index = 0; index <= 20; ++index) {
    path.push_back({0.5 * index, 0.0, 0.0});
  }

  TrackerCore core(params);
  ASSERT_TRUE(core.start(path).empty());
  core.update_pose({4.1, 0.1, 0.0, 0.0});
  const auto result = core.step();

  EXPECT_TRUE(result.error.empty()) << result.error;
  EXPECT_EQ(core.progress_index(), 8U);
}

TEST(TrackerCoreTest, DoesNotInitializeOnOverlappingPointBeyondStartDistancePrefix)
{
  lawn_mower_tracker::TrackerParams params;
  params.initial_search_points = 3U;
  params.initial_search_distance = 5.0;
  params.max_start_distance = 1.0;
  std::vector<PathPoint> path;
  for (int index = 0; index <= 20; ++index) {
    path.push_back({0.5 * index, 0.0, 0.0});
  }
  path.push_back({4.11, 0.1, 0.0});

  TrackerCore core(params);
  ASSERT_TRUE(core.start(path).empty());
  core.update_pose({4.1, 0.1, 0.0, 0.0});
  const auto result = core.step();

  EXPECT_TRUE(result.error.empty()) << result.error;
  EXPECT_EQ(core.progress_index(), 8U);
  EXPECT_NE(core.progress_index(), path.size() - 1U);
}

TEST(TrackerCoreTest, InitialSearchPointsRemainAMinimumForSparsePaths)
{
  lawn_mower_tracker::TrackerParams params;
  params.initial_search_points = 4U;
  params.initial_search_distance = 1.0;
  params.max_start_distance = 1.0;
  TrackerCore core(params);
  ASSERT_TRUE(core.start({
    {0, 0, 0}, {2, 0, 0}, {4, 0, 0}, {6, 0, 0}, {8, 0, 0}}).empty());
  core.update_pose({5.9, 0.0, 0.0, 0.0});

  const auto result = core.step();

  EXPECT_TRUE(result.error.empty()) << result.error;
  EXPECT_EQ(core.progress_index(), 3U);
}

TEST(TrackerCoreTest, RejectsPoseTooFarFromInitialSearchPrefix)
{
  lawn_mower_tracker::TrackerParams params;
  params.initial_search_distance = 5.0;
  params.max_start_distance = 1.0;
  TrackerCore core(params);
  ASSERT_TRUE(core.start({
    {0, 0, 0}, {1, 0, 0}, {2, 0, 0}, {3, 0, 0},
    {4, 0, 0}, {5, 0, 0}, {6, 0, 0}}).empty());
  core.update_pose({4.0, 2.0, 0.0, 0.0});

  const auto result = core.step();

  EXPECT_EQ(result.error, "RTK pose is too far from the path start");
  EXPECT_DOUBLE_EQ(result.linear_velocity, 0.0);
}

TEST(TrackerCoreTest, RejectsInvalidInitialSearchDistance)
{
  lawn_mower_tracker::TrackerParams params;
  params.initial_search_distance = 0.0;
  EXPECT_THROW(TrackerCore core(params), std::invalid_argument);
  params.initial_search_distance = -1.0;
  EXPECT_THROW(TrackerCore core(params), std::invalid_argument);
  params.initial_search_distance = NAN;
  EXPECT_THROW(TrackerCore core(params), std::invalid_argument);
}

TEST(TrackerCoreTest, ClosestPointDoesNotSkipAnUnreachedDirectionBoundary)
{
  lawn_mower_tracker::TrackerParams params;
  params.lookahead_distance = 0.4;
  params.direction_switch_distance = 0.5;
  params.search_forward_points = 30U;
  params.initial_search_distance = 0.1;
  params.initial_search_points = 1U;
  TrackerCore core(params);
  ASSERT_TRUE(core.start({
    {0.0, 0.0, 0.0, -1, 11, true},
    {-1.0, 0.0, 0.0, -1, 11, true},
    {-2.0, 0.0, 0.0, -1, 11, true},
    {-3.0, 0.0, 0.0, -1, 11, true},
    {-2.5, 0.0, 0.0, 1, 11, true},
    {-2.0, 0.0, 0.0, 1, 11, true},
    {-1.01, 0.0, 0.0, 1, 11, true},
    {0.0, 0.0, 0.0, 1, 11, true},
    {-0.5, 0.0, 0.0, -1, 11, true},
    {-1.0, 0.0, 0.0, -1, 11, true}}).empty());

  core.update_pose({0.0, 0.0, 0.0, 0.0});
  ASSERT_LT(core.step().linear_velocity, 0.0);
  core.update_pose({-1.01, 0.0, 0.0, 0.0});
  const auto result = core.step();

  EXPECT_TRUE(result.error.empty()) << result.error;
  EXPECT_EQ(core.progress_index(), 1U);
  EXPECT_LT(result.linear_velocity, 0.0);
}

TEST(TrackerCoreTest, ExecutesOverlappingDirectionRunsInPathOrder)
{
  lawn_mower_tracker::TrackerParams params;
  params.lookahead_distance = 0.4;
  params.direction_switch_distance = 0.5;
  params.search_forward_points = 30U;
  params.initial_search_distance = 0.1;
  params.initial_search_points = 1U;
  TrackerCore core(params);
  ASSERT_TRUE(core.start({
    {0.0, 0.0, 0.0, -1, 11, true},
    {-1.0, 0.0, 0.0, -1, 11, true},
    {-2.0, 0.0, 0.0, -1, 11, true},
    {-3.0, 0.0, 0.0, -1, 11, true},
    {-2.5, 0.0, 0.0, 1, 11, true},
    {-2.0, 0.0, 0.0, 1, 11, true},
    {-1.01, 0.0, 0.0, 1, 11, true},
    {0.0, 0.0, 0.0, 1, 11, true},
    {-0.5, 0.0, 0.0, -1, 11, true},
    {-1.0, 0.0, 0.0, -1, 11, true}}).empty());

  core.update_pose({0.0, 0.0, 0.0, 0.0});
  ASSERT_LT(core.step().linear_velocity, 0.0);

  core.update_pose({-3.0, 0.0, 0.0, 0.0});
  const auto first_switch = core.step();
  ASSERT_TRUE(first_switch.error.empty()) << first_switch.error;
  EXPECT_DOUBLE_EQ(first_switch.linear_velocity, 0.0);
  const auto forward = core.step();
  ASSERT_TRUE(forward.error.empty()) << forward.error;
  EXPECT_GT(forward.linear_velocity, 0.0);

  core.update_pose({-1.01, 0.0, 0.0, 0.0});
  const auto middle = core.step();
  ASSERT_TRUE(middle.error.empty()) << middle.error;
  EXPECT_EQ(core.progress_index(), 6U);
  EXPECT_GT(middle.linear_velocity, 0.0);

  core.update_pose({0.0, 0.0, 0.0, 0.0});
  const auto second_switch = core.step();
  ASSERT_TRUE(second_switch.error.empty()) << second_switch.error;
  EXPECT_DOUBLE_EQ(second_switch.linear_velocity, 0.0);
  const auto reverse = core.step();
  ASSERT_TRUE(reverse.error.empty()) << reverse.error;
  EXPECT_LT(reverse.linear_velocity, 0.0);
}

TEST(TrackerCoreTest, ReplaysMap01WithoutSkippingDirectionRuns)
{
  const auto map = std::filesystem::path(LAWN_MOWER_WORKSPACE_ROOT) /
    "map_01" / "lawn_mower_records.csv";
  const auto loaded = lawn_mower_tracker::load_path_csv(map, {});
  ASSERT_TRUE(loaded.ok()) << loaded.error;

  TrackerCore core;
  ASSERT_TRUE(core.start(loaded.points).empty());
  bool completed = false;
  for (const auto & point : loaded.points) {
    ASSERT_TRUE(point.has_yaw);
    ASSERT_TRUE(core.update_pose({point.x, point.y, point.z, point.yaw}));
    auto result = core.step();
    ASSERT_TRUE(result.error.empty()) <<
      "index=" << core.progress_index() << ": " << result.error;
    if (result.completed) {
      completed = true;
      break;
    }
    if (result.linear_velocity == 0.0) {
      result = core.step();
      ASSERT_TRUE(result.error.empty()) <<
        "index=" << core.progress_index() << ": " << result.error;
      if (result.completed) {
        completed = true;
        break;
      }
    }
  }

  EXPECT_TRUE(completed);
  EXPECT_EQ(core.state(), TrackerState::kCompleted);
}

TEST(TrackerCoreTest, AlignedReverseApproachDoesNotIntroduceTerminalSteering)
{
  lawn_mower_tracker::TrackerParams params;
  params.wheelbase = 0.36;
  params.lookahead_distance = 0.6;
  params.terminal_alignment_distance = 0.8;
  TrackerCore core(params);
  std::vector<PathPoint> path;
  for (int i = 0; i <= 60; ++i) {
    path.push_back(point_with_yaw(-0.05 * i, 0.0, 0.0, -1));
  }
  ASSERT_TRUE(core.start(path).empty());
  core.update_pose({-2.5, 0.0, 0.0, 0.0});
  const auto result = core.step();
  EXPECT_LT(result.linear_velocity, 0.0);
  EXPECT_NEAR(result.steering_angle, 0.0, 1e-9);
}

TEST(TrackerCoreTest, ReverseTerminalYawCorrectionTurnsBodyTowardRecordedYaw)
{
  lawn_mower_tracker::TrackerParams params;
  params.wheelbase = 0.36;
  params.lookahead_distance = 0.6;
  params.terminal_alignment_distance = 0.8;
  for (double target_yaw : {-0.3, 0.3}) {
    TrackerCore core(params);
    std::vector<PathPoint> path;
    for (int i = 0; i <= 60; ++i) {
      path.push_back(point_with_yaw(-0.05 * i, 0.0, target_yaw, -1));
    }
    ASSERT_TRUE(core.start(path).empty());
    core.update_pose({-2.5, 0.0, 0.0, 0.0});
    const auto result = core.step();
    const double body_yaw_rate = result.linear_velocity *
      std::tan(result.steering_angle) / params.wheelbase;
    EXPECT_GT(body_yaw_rate * target_yaw, 0.0);
  }
}

TEST(TrackerCoreTest, CommittedReverseDoesNotRevertAfterLeavingSwitchRadius)
{
  lawn_mower_tracker::TrackerParams params;
  params.lookahead_distance = 0.6;
  params.initial_search_distance = 0.1;
  params.initial_search_points = 1U;
  TrackerCore core(params);
  std::vector<PathPoint> path;
  for (int i = 0; i <= 20; ++i) {
    path.push_back(point_with_yaw(i * 0.1, 0.0, 0.0, 1));
  }
  for (int i = 0; i < 20; ++i) {
    path.push_back(point_with_yaw(1.99 - i * 0.1, 0.1, 0.0, -1));
  }
  ASSERT_TRUE(core.start(path).empty());
  core.update_pose({0.0, 0.0, 0.0, 0.0});
  ASSERT_GT(core.step().linear_velocity, 0.0);
  core.update_pose({1.51, 0.0, 0.0, 0.0});
  ASSERT_DOUBLE_EQ(core.step().linear_velocity, 0.0);
  EXPECT_EQ(core.progress_index(), 21U);
  ASSERT_LT(core.step().linear_velocity, 0.0);
  core.update_pose({1.48, 0.0, 0.0, 0.0});
  const auto result = core.step();
  EXPECT_TRUE(result.error.empty()) << result.error;
  EXPECT_LT(result.linear_velocity, 0.0);
}

TEST(TrackerCoreTest, TracksMap01WithFeedbackGeneratedFromItsOwnCommands)
{
  const auto map = std::filesystem::path(LAWN_MOWER_WORKSPACE_ROOT) /
    "map_01" / "lawn_mower_records.csv";
  lawn_mower_tracker::CsvColumns columns;
  columns.z_enabled = false;
  const auto loaded = lawn_mower_tracker::load_path_csv(map, columns);
  ASSERT_TRUE(loaded.ok()) << loaded.error;
  lawn_mower_tracker::TrackerParams params;
  params.wheelbase = 0.36;
  params.lookahead_distance = 0.6;
  params.min_velocity = 0.15;
  params.max_velocity = 0.4;
  params.slowdown_distance = 0.7;
  params.terminal_alignment_distance = 0.8;
  TrackerCore core(params);
  ASSERT_TRUE(core.start(loaded.points).empty());
  const auto & first = loaded.points.front();
  Pose2D pose{first.x, first.y, 0.0, first.yaw};
  bool completed = false;
  for (int tick = 0; tick < 40000; ++tick) {
    ASSERT_TRUE(core.update_pose(pose));
    const auto result = core.step();
    ASSERT_TRUE(result.error.empty()) << "tick=" << tick <<
      " index=" << core.progress_index() << " " << result.error;
    if (result.completed) {
      completed = true;
      break;
    }
    const double yaw_rate = std::clamp(result.linear_velocity *
      std::tan(result.steering_angle) / params.wheelbase, -1.5, 1.5);
    pose.x += 0.02 * result.linear_velocity * std::cos(pose.yaw);
    pose.y += 0.02 * result.linear_velocity * std::sin(pose.yaw);
    pose.yaw = std::remainder(pose.yaw + 0.02 * yaw_rate, 2.0 * std::acos(-1.0));
  }
  EXPECT_TRUE(completed) << "last index=" << core.progress_index();
  EXPECT_EQ(core.state(), TrackerState::kCompleted);
}

TEST(TrackerCoreTest, InitialSelectionAcrossReversalCommitsTheNewRun)
{
  lawn_mower_tracker::TrackerParams params;
  params.lookahead_distance = 0.6;
  TrackerCore core(params);
  std::vector<PathPoint> path;
  for (int i = 0; i <= 20; ++i) {
    path.push_back(point_with_yaw(i * 0.1, 0.0, 0.0, 1));
  }
  for (int i = 0; i < 20; ++i) {
    path.push_back(point_with_yaw(1.99 - i * 0.1, 0.1, 0.0, -1));
  }
  ASSERT_TRUE(core.start(path).empty());
  core.update_pose({1.51, 0.0, 0.0, 0.0});
  ASSERT_LT(core.step().linear_velocity, 0.0);
  EXPECT_EQ(core.progress_index(), 21U);
  core.update_pose({1.48, 0.0, 0.0, 0.0});
  const auto result = core.step();
  EXPECT_TRUE(result.error.empty()) << result.error;
  EXPECT_LT(result.linear_velocity, 0.0);
}

}  // namespace

TEST(TrackerCoreAvoidance, LocalGoalFollowsPathAndStopsAtDirectionBoundary)
{
  lawn_mower_tracker::TrackerCore core;
  std::vector<lawn_mower_tracker::PathPoint> path;
  for (int i=0;i<=10;++i) {
    lawn_mower_tracker::PathPoint p;
    p.x=i*.2;p.direction=1;path.push_back(p);
  }
  for (int i=1;i<=10;++i) {
    lawn_mower_tracker::PathPoint p;
    p.x=2-i*.2;p.direction=-1;path.push_back(p);
  }
  ASSERT_TRUE(core.start(path).empty());
  ASSERT_TRUE(core.update_pose({0,0,0,0}));
  core.step();
  const auto goal=core.local_goal(3.0);
  EXPECT_NEAR(goal.x,2.0,.01);
  EXPECT_NEAR(goal.y,0,.01);
}
TEST(TrackerCoreAvoidance, InterpolatesSparseReferenceSegments)
{
  lawn_mower_tracker::TrackerCore core;
  ASSERT_TRUE(core.start({{0,0,0},{10,0,0},{20,0,0}}).empty());
  ASSERT_TRUE(core.update_pose({0,0,0,0}));
  core.step();
  EXPECT_NEAR(core.local_goal(3).x,3,.01);
}
