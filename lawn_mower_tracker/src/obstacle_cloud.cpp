#include "lawn_mower_tracker/tracker_node.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <sensor_msgs/point_cloud2_iterator.hpp>
namespace lawn_mower_tracker
{
void TrackerNode::cloud_callback(sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  have_cloud_ = false;
  const auto reject = [this]() {
      publish_stop();
      RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),2000,"Invalid obstacle cloud; stopping");
    };
  if (msg->header.stamp.sec<0 || msg->header.stamp.nanosec>=1000000000U) {
    reject();return;
  }
  const double age = (now()-rclcpp::Time(msg->header.stamp,get_clock()->get_clock_type())).seconds();
  const uint64_t count=uint64_t(msg->width)*msg->height;
  if (!have_rtk_ || msg->header.frame_id!=cloud_frame_ || msg->is_bigendian ||
    count==0 || count>300000 || msg->point_step<12 ||
    uint64_t(msg->row_step)<uint64_t(msg->width)*msg->point_step ||
    uint64_t(msg->data.size())<uint64_t(msg->row_step)*msg->height ||
    age<-.05 || age>cloud_timeout_)
  {reject();return;}
  std::array<uint32_t,3> offsets{};
  const std::array<std::string,3> names={"x","y","z"};
  for (std::size_t axis=0;axis<3;++axis) {
    const auto field=std::find_if(msg->fields.begin(),msg->fields.end(),
      [&](const auto & f) {return f.name==names[axis];});
    if (field==msg->fields.end() || field->datatype!=sensor_msgs::msg::PointField::FLOAT32 ||
      field->count!=1 || uint64_t(field->offset)+4>msg->point_step)
    {reject();return;}
    offsets[axis]=field->offset;
  }
  std::vector<ObstaclePoint> points;
  points.reserve(count);
  for (uint32_t row=0;row<msg->height;++row) {
    for (uint32_t col=0;col<msg->width;++col) {
      const auto * data=msg->data.data()+uint64_t(row)*msg->row_step+uint64_t(col)*msg->point_step;
      std::array<float,3> p;
      for (std::size_t i=0;i<3;++i) {std::memcpy(&p[i],data+offsets[i],4);}
      if (std::isfinite(p[0]) && std::isfinite(p[1]) && std::isfinite(p[2])) {
        points.push_back({p[0],p[1],p[2]});
      }
    }
  }
  if (points.empty()) {reject();return;}
  obstacles_=obstacle_planner_->filter(points);
  cloud_pose_=current_pose_;
  cloud_received_=std::chrono::steady_clock::now();
  cloud_stamp_=rclcpp::Time(msg->header.stamp,get_clock()->get_clock_type());
  have_cloud_=true;
  need_plan_=true;
  sensor_msgs::msg::PointCloud2 filtered;
  filtered.header=msg->header;
  sensor_msgs::PointCloud2Modifier modifier(filtered);
  modifier.setPointCloud2FieldsByString(1,"xyz");
  modifier.resize(obstacles_.size());
  sensor_msgs::PointCloud2Iterator<float> x(filtered,"x"),y(filtered,"y"),z(filtered,"z");
  for (const auto & p : obstacles_) {*x=p.x;*y=p.y;*z=p.z;++x;++y;++z;}
  filtered_publisher_->publish(filtered);
}

bool TrackerNode::cloud_fresh() const
{
  if (!have_cloud_) {return false;}
  const double received_age=std::chrono::duration<double>(
    std::chrono::steady_clock::now()-cloud_received_).count();
  const double stamp_age=(now()-cloud_stamp_).seconds();
  return received_age<=cloud_timeout_ && stamp_age<=cloud_timeout_ && stamp_age>=-.05;
}

bool TrackerNode::apply_avoidance(ControlResult & result)
{
  if (!cloud_fresh()) {
    RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),2000,"Obstacle lidar missing/stale; stopping");
    return false;
  }
  // Only a few path poses are expressed in the last cloud's axes; points stay unchanged.
  const double c=std::cos(cloud_pose_.yaw), s=std::sin(cloud_pose_.yaw);
  const double dx=current_pose_.x-cloud_pose_.x,dy=current_pose_.y-cloud_pose_.y;
  const Pose2D start{c*dx+s*dy,-s*dx+c*dy,0,current_pose_.yaw-cloud_pose_.yaw};
  const double cs=std::cos(start.yaw),sn=std::sin(start.yaw);
  result.linear_velocity=std::copysign(
    std::min(std::abs(result.linear_velocity),avoidance_speed_),result.linear_velocity);
  AvoidanceResult plan;
  const bool terminal = std::hypot(core_.local_goal(planning_distance_).x,
    core_.local_goal(planning_distance_).y)<.6;
  if (!need_plan_ && !terminal && planned_direction_*result.linear_velocity>0) {
    if (!previous_plan_.safe || !obstacle_planner_->check_motion(obstacles_,start,
        result.linear_velocity,previous_plan_.steering).safe) {return false;}
    result.steering_angle=previous_plan_.steering;
    return true;
  }
  // Return control early once the original tracker's swept motion is clear.
  // This gives its terminal heading blend enough distance to converge.
  plan=obstacle_planner_->check_motion(obstacles_,start,result.linear_velocity,
    result.steering_angle,std::min(1.5,planning_distance_));
  const auto search_started=std::chrono::steady_clock::now();
  for (double distance=planning_distance_;!plan.safe && distance>=.5;distance-=.6) {
    if (!cloud_fresh() || std::chrono::steady_clock::now()-search_started>
      std::chrono::milliseconds(60)) {break;}
    const auto target=core_.local_goal(distance);
    // Preserve the tracker's terminal yaw convergence and direction-switch behavior.
    if (std::hypot(target.x,target.y)<.6) {
      plan=obstacle_planner_->check_motion(obstacles_,start,
        result.linear_velocity,result.steering_angle);
      break;
    }
    const Pose2D goal{start.x+cs*target.x-sn*target.y,
      start.y+sn*target.x+cs*target.y,0,start.yaw+target.yaw};
    plan=obstacle_planner_->plan(obstacles_,goal,result.linear_velocity,start);
    if (plan.safe) {break;}
  }
  planned_direction_=result.linear_velocity;
  previous_plan_=plan;
  need_plan_=false;
  if (!plan.safe) {
    RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),2000,"No collision-free local route; stopping");
    return false;
  }
  result.steering_angle=plan.steering;
  nav_msgs::msg::Path debug;
  debug.header.frame_id=cloud_frame_;
  debug.header.stamp=cloud_stamp_;
  for (const auto & p : plan.path) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header=debug.header;
    pose.pose.position.x=p.x;
    pose.pose.position.y=p.y;
    pose.pose.position.z=-get_parameter("lidar_origin_height").as_double();
    pose.pose.orientation.z=std::sin(p.yaw/2);
    pose.pose.orientation.w=std::cos(p.yaw/2);
    debug.poses.push_back(pose);
  }
  local_path_publisher_->publish(debug);
  return true;
}
}
