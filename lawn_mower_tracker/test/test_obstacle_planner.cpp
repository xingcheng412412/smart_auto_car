#include <gtest/gtest.h>
#include <cmath>
#include <limits>
#include "lawn_mower_tracker/obstacle_planner.hpp"
#include "lawn_mower_tracker/tracker_core.hpp"
using namespace lawn_mower_tracker;

TEST(ObstaclePlanner, FiltersBodyGroundAndInvalidReturns)
{
  ObstaclePlanner planner;
  auto points = planner.filter({{0,0,-0.4}, {.26,.15,-.5}, {1,0,-.60},
    {1,0,.5}, {1,0,-.3}, {NAN,0,0}, {0,INFINITY,0}});
  ASSERT_EQ(points.size(), 1U);
  EXPECT_DOUBLE_EQ(points.front().x, 1);
}
TEST(ObstaclePlanner, ValidatesGeometry)
{
  ObstacleParams p;
  p.resolution = 0;
  EXPECT_THROW(ObstaclePlanner{p}, std::invalid_argument);
  p = {}; p.max_steering = 1.6;
  EXPECT_THROW(ObstaclePlanner{p}, std::invalid_argument);
}
TEST(ObstaclePlanner, StraightAndReversePaths)
{
  ObstaclePlanner planner;
  for (double direction : {1.0,-1.0}) {
    auto result = planner.plan({}, {3*direction,0,0,0}, direction);
    ASSERT_TRUE(result.safe);
    EXPECT_NEAR(result.steering, 0, .01);
    EXPECT_GT(result.path.size(), 3U);
  }
}
TEST(ObstaclePlanner, ChoosesOpenSideAndRespectsSteering)
{
  ObstaclePlanner planner;
  for (double side : {1.0,-1.0}) {
    std::vector<ObstaclePoint> wall;
    for (double y = 0; y <= 2; y += .08) {wall.push_back({1.5, side*y, -.3});}
    auto result = planner.plan(wall, {3,0,0,0}, 1);
    ASSERT_TRUE(result.safe);
    double excursion = 0;
    for (const auto & p : result.path) {
      excursion = std::min(excursion, side*p.y);
      for (const auto & obstacle : wall) {
        EXPECT_GT(std::hypot(p.x-obstacle.x,p.y-obstacle.y), .4);
      }
    }
    EXPECT_LT(excursion, -.45);
    EXPECT_LE(std::abs(result.steering), .942477796);
  }
}
TEST(ObstaclePlanner, StopsForBlockedGoalAndSurroundedVehicle)
{
  ObstaclePlanner planner;
  EXPECT_FALSE(planner.plan({{3,0,-.3}}, {3,0,0,0}, 1).safe);
  EXPECT_FALSE(planner.plan({{.35,0,-.3}}, {3,0,0,0}, 1).safe);
  std::vector<ObstaclePoint> wall;
  for (double y=-5; y<=5; y+=.1) {wall.push_back({1.3,y,-.3});}
  EXPECT_FALSE(planner.plan(wall,{3,0,0,0},1).safe);
}
TEST(ObstaclePlanner, ClosedLoopBypassesAndReturnsToReference)
{
  ObstaclePlanner planner;
  Pose2D pose;
  double max_deviation = 0;
  for (int step=0; step<220 && pose.x<3.7; ++step) {
    const double c=std::cos(pose.yaw), s=std::sin(pose.yaw);
    std::vector<ObstaclePoint> obstacles;
    for (double y=-.2; y<=.2; y+=.05) {
      double dx=1.6-pose.x, dy=y-pose.y;
      obstacles.push_back({c*dx+s*dy,-s*dx+c*dy,-.3});
    }
    const double dx=4-pose.x, dy=-pose.y;
    auto result=planner.plan(obstacles,{c*dx+s*dy,-s*dx+c*dy,0,-pose.yaw},1);
    ASSERT_TRUE(result.safe) << step << " pose " << pose.x << "," << pose.y;
    const double ds=.04;
    const double curvature=std::tan(result.steering)/.36;
    pose.x+=ds*std::cos(pose.yaw+ds*curvature/2);
    pose.y+=ds*std::sin(pose.yaw+ds*curvature/2);
    pose.yaw+=ds*curvature;
    max_deviation=std::max(max_deviation,std::abs(pose.y));
    EXPECT_GT(std::hypot(pose.x-1.6,std::max(0.0,std::abs(pose.y)-.2)),.4);
  }
  EXPECT_GT(pose.x,3.7);
  EXPECT_GT(max_deviation,.6);
  EXPECT_LT(std::abs(pose.y),.3);
}
TEST(ObstaclePlanner, ChecksNominalTerminalMotionAndStaleCloudPose)
{
  ObstaclePlanner planner;
  EXPECT_TRUE(planner.check_motion({}, {0,0,0,0}, .2, .1).safe);
  EXPECT_FALSE(planner.check_motion({{.6,0,-.3}}, {0,0,0,0}, .2, 0).safe);
  EXPECT_FALSE(planner.plan({{1,0,-.3}}, {3,0,0,0}, 1, {.6,0,0,0}).safe);
}
TEST(ObstaclePlanner, NominalLookaheadDetectsObstacleBeforeBrakingHorizon)
{
  ObstaclePlanner planner;
  EXPECT_TRUE(planner.check_motion({{1.2,0,-.3}}, {}, .15, 0).safe);
  EXPECT_FALSE(planner.check_motion({{1.2,0,-.3}}, {}, .15, 0, 1.5).safe);
}

TEST(ObstaclePlanner, TrackerDetourRejoinsAndCompletesWithTerminalYaw)
{
  TrackerParams params;
  params.wheelbase=.36;params.lookahead_distance=.6;
  params.min_velocity=.15;params.max_velocity=.4;
  params.slowdown_distance=.7;params.terminal_alignment_distance=.8;
  TrackerCore core(params);
  std::vector<PathPoint> reference;
  for (int i=0;i<=84;++i) {
    PathPoint p;p.x=i*.05;p.direction=1;p.has_yaw=true;
    reference.push_back(p);
  }
  ASSERT_TRUE(core.start(reference).empty());
  ObstaclePlanner planner;
  Pose2D pose;
  bool completed=false;
  double excursion=0;
  for (int iteration=0;iteration<1200;++iteration) {
    core.update_pose(pose);
    auto command=core.step();
    ASSERT_TRUE(command.error.empty()) << command.error << " at " << pose.x << "," << pose.y
      << " yaw " << pose.yaw;
    if (command.completed) {completed=true;break;}
    const double c=std::cos(pose.yaw),s=std::sin(pose.yaw);
    std::vector<ObstaclePoint> points;
    for (double y=-.3;y<=.3;y+=.05) {
      for (double x : {1.55,2.05}) {
        const double dx=x-pose.x,dy=y-pose.y;
        points.push_back({c*dx+s*dy,-s*dx+c*dy,-.3});
      }
    }
    command.linear_velocity=std::copysign(std::min(std::abs(command.linear_velocity),.15),
      command.linear_velocity);
    auto plan=planner.check_motion(points,{},command.linear_velocity,command.steering_angle,1.5);
    if (!plan.safe) {
      plan=planner.plan(points,core.local_goal(3.6),command.linear_velocity);
    }
    ASSERT_TRUE(plan.safe) << iteration;
    double ds=command.linear_velocity*.05;
    const double curvature=std::tan(plan.steering)/.36;
    pose.x+=ds*std::cos(pose.yaw+ds*curvature/2);
    pose.y+=ds*std::sin(pose.yaw+ds*curvature/2);
    pose.yaw+=ds*curvature;
    excursion=std::max(excursion,std::abs(pose.y));
    EXPECT_GT(std::hypot(std::max(std::abs(pose.x-1.8)-.25,0.0),
      std::max(std::abs(pose.y)-.3,0.0)),.45);
  }
  EXPECT_TRUE(completed);
  EXPECT_GT(excursion,.7);
  EXPECT_LT(std::abs(pose.yaw),params.terminal_yaw_tolerance);
}

TEST(ObstaclePlanner, RawAndOpticalOriginsFilterTheSamePhysicalGeometry)
{
  ObstacleParams raw_params;
  raw_params.optical_height=.483976069;
  ObstaclePlanner raw(raw_params), optical;
  // Same body return, ground return and obstacle expressed in each sensor frame.
  auto raw_points=raw.filter({{0,0,.1-.483976069},{1,0,-.07-.483976069},
    {1,0,.1-.483976069}});
  auto optical_points=optical.filter({{0,0,.1-.518976069},{1,0,-.07-.518976069},
    {1,0,.1-.518976069}});
  ASSERT_EQ(raw_points.size(),1U);
  ASSERT_EQ(optical_points.size(),1U);
  EXPECT_DOUBLE_EQ(raw_points[0].x,optical_points[0].x);
  EXPECT_NEAR(raw_points[0].z-optical_points[0].z,.035,1e-9);
}
