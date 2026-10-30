#include "lawn_mower_tracker/obstacle_planner.hpp"
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <queue>
#include <stdexcept>
#include <unordered_map>
namespace lawn_mower_tracker
{
ObstaclePlanner::ObstaclePlanner(ObstacleParams params) : params_(params)
{
  for (double value : {params.optical_height, params.body_half_length, params.body_half_width,
      params.safety_margin, params.resolution, params.range, params.wheelbase, params.max_steering})
  {
    if (!std::isfinite(value) || value <= 0) {
      throw std::invalid_argument("avoidance geometry must be finite and positive");
    }
  }
  if (!std::isfinite(params.min_height) || !std::isfinite(params.max_height) ||
    !std::isfinite(params.body_min_z) || !std::isfinite(params.body_max_z) ||
    params.min_height >= params.max_height || params.body_min_z >= params.body_max_z ||
    params.max_steering >= 1.5 || params.max_expansions < 1 || params.max_expansions > 100000 ||
    params.range > 20 || params.resolution < .04 || params.range / params.resolution > 250)
  {
    throw std::invalid_argument("invalid avoidance search/height limits");
  }
}
std::vector<ObstaclePoint> ObstaclePlanner::filter(const std::vector<ObstaclePoint> & points) const
{
  std::vector<ObstaclePoint> result;
  for (const auto & p : points) {
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {continue;}
    const double height = p.z + params_.optical_height;
    const bool self = std::abs(p.x) <= params_.body_half_length &&
      std::abs(p.y) <= params_.body_half_width &&
      height >= params_.body_min_z && height <= params_.body_max_z;
    if (!self && height >= params_.min_height && height <= params_.max_height &&
      std::hypot(p.x,p.y) <= params_.range)
    {
      result.push_back(p);
    }
  }
  return result;
}
AvoidanceResult ObstaclePlanner::check_motion(const std::vector<ObstaclePoint> & obstacles,
  const Pose2D & start, double speed, double steering, double horizon) const
{
  if (!std::isfinite(speed) || !std::isfinite(steering) ||
    !std::isfinite(horizon) || horizon<0 || horizon>params_.range ||
    !std::isfinite(start.x) || !std::isfinite(start.y) || !std::isfinite(start.yaw) ||
    std::abs(steering)>params_.max_steering) {return {};}
  const double radius=std::hypot(params_.body_half_length,params_.body_half_width)+
    params_.safety_margin+params_.resolution/6;
  AvoidanceResult result;
  result.steering=steering;
  Pose2D pose=start;
  const double length=std::max({.15,std::abs(speed)*.8,horizon});
  const int samples=static_cast<int>(std::ceil(length/(params_.resolution/6)));
  const double ds=std::copysign(length/samples,speed);
  const double curvature=std::tan(steering)/params_.wheelbase;
  for (int i=0;i<=samples;++i) {
    if (std::hypot(pose.x,pose.y)+radius>=params_.range) {return {};}
    for (const auto & p : obstacles) {
      if (!std::isfinite(p.x) || !std::isfinite(p.y) ||
        std::hypot(p.x-pose.x,p.y-pose.y)<=radius) {return {};}
    }
    result.path.push_back(pose);
    pose.x+=ds*std::cos(pose.yaw+ds*curvature/2);
    pose.y+=ds*std::sin(pose.yaw+ds*curvature/2);
    pose.yaw+=ds*curvature;
  }
  result.safe=true;
  return result;
}
AvoidanceResult ObstaclePlanner::plan(const std::vector<ObstaclePoint> & obstacles,
  const Pose2D & goal, double direction, const Pose2D & start) const
{
  if (!std::isfinite(goal.x) || !std::isfinite(goal.y) || !std::isfinite(goal.yaw) ||
    !std::isfinite(start.x) || !std::isfinite(start.y) || !std::isfinite(start.yaw) ||
    !std::isfinite(direction) || direction == 0) {return {};}
  direction = direction > 0 ? 1 : -1;
  const double res = params_.resolution;
  const int half = static_cast<int>(std::ceil(params_.range / res));
  const int width = 2*half+1;
  const auto cell = [=](double x) {return static_cast<int>(std::lround(x/res))+half;};
  // Keep actual point positions in spatial buckets. Quantizing obstacles to
  // occupied cells made the inflated boundary jump as the sensor moved.
  std::vector<std::vector<ObstaclePoint>> buckets(width*width);
  const double radius = std::hypot(params_.body_half_length,params_.body_half_width)+
    params_.safety_margin+res/6;
  const int inflation = static_cast<int>(std::ceil(radius/res))+1;
  for (const auto & p : obstacles) {
    if (!std::isfinite(p.x) || !std::isfinite(p.y)) {return {};}
    if (std::hypot(p.x,p.y)>params_.range+radius) {continue;}
    const int cx=cell(p.x),cy=cell(p.y);
    if (cx>=0 && cy>=0 && cx<width && cy<width) {buckets[cy*width+cx].push_back(p);}
  }
  const auto free = [&](double x,double y) {
      if (std::hypot(x,y)+radius>=params_.range) {return false;}
      const int cx=cell(x),cy=cell(y);
      for (int iy=std::max(0,cy-inflation);iy<=std::min(width-1,cy+inflation);++iy) {
        for (int ix=std::max(0,cx-inflation);ix<=std::min(width-1,cx+inflation);++ix) {
          for (const auto & p : buckets[iy*width+ix]) {
            if ((p.x-x)*(p.x-x)+(p.y-y)*(p.y-y)<=radius*radius) {return false;}
          }
        }
      }
      return true;
    };
  if (!free(start.x,start.y) || !free(goal.x,goal.y)) {return {};}
  constexpr int headings=64;
  const auto key = [&](const Pose2D & pose) -> int {
      const double angle=std::atan2(std::sin(pose.yaw),std::cos(pose.yaw));
      const int h=static_cast<int>(std::lround((angle+M_PI)*headings/(2*M_PI)))%headings;
      return (cell(pose.y)*width+cell(pose.x))*headings+h;
    };
  struct SearchNode {Pose2D pose; double cost; int parent; double steering;};
  using Entry=std::pair<double,int>;
  std::priority_queue<Entry,std::vector<Entry>,std::greater<Entry>> open;
  std::vector<SearchNode> nodes;
  nodes.reserve(params_.max_expansions*3);
  std::unordered_map<int,double> costs;
  const auto heuristic = [&](const Pose2D & p) {return std::hypot(goal.x-p.x,goal.y-p.y);};
  nodes.push_back({start,0,-1,0});
  costs[key(start)]=0;
  open.push({heuristic(start),0});
  const double length = res*2;
  int finish=-1;
  const auto search_started=std::chrono::steady_clock::now();
  for (int expansions=0;!open.empty() && expansions<params_.max_expansions;) {
    if (std::chrono::steady_clock::now()-search_started>std::chrono::milliseconds(30)) {
      break;
    }
    const int index=open.top().second;
    open.pop();
    const auto node=nodes[index];
    if (node.cost>costs[key(node.pose)]+1e-9) {continue;}
    ++expansions;
    const double yaw_error=std::atan2(std::sin(goal.yaw-node.pose.yaw),
      std::cos(goal.yaw-node.pose.yaw));
    if (heuristic(node.pose)<res*2 && std::abs(yaw_error)<.2) {finish=index;break;}
    for (double ratio : {0.0,-.5,.5,-1.0,1.0}) {
      const double steering=ratio*params_.max_steering;
      const double curvature=std::tan(steering)/params_.wheelbase;
      Pose2D p=node.pose;
      bool valid=true;
      for (int sample=0;sample<6;++sample) {
        const double ds=direction*length/6;
        p.x+=ds*std::cos(p.yaw+ds*curvature/2);
        p.y+=ds*std::sin(p.yaw+ds*curvature/2);
        p.yaw+=ds*curvature;
        if (!free(p.x,p.y)) {valid=false;break;}
      }
      if (!valid) {continue;}
      const double cost=node.cost+length*(1+.10*std::abs(ratio))+
        .04*std::abs(steering-node.steering);
      const int k=key(p);
      auto found=costs.find(k);
      if (found!=costs.end() && found->second<=cost) {continue;}
      costs[k]=cost;
      nodes.push_back({p,cost,index,steering});
      open.push({cost+1.12*heuristic(p),static_cast<int>(nodes.size()-1)});
    }
  }
  if (finish<0) {return {};}
  AvoidanceResult result;
  result.safe=true;
  for (int index=finish;index>=0;index=nodes[index].parent) {
    result.path.push_back(nodes[index].pose);
    if (nodes[index].parent==0) {result.steering=nodes[index].steering;}
  }
  std::reverse(result.path.begin(),result.path.end());
  return result;
}
}
