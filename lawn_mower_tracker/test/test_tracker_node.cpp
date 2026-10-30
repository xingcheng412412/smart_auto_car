#include <gtest/gtest.h>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <future>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include <can_client/msg/vehicle_cmd.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include "lawn_mower_tracker/srv/mower_track_control.hpp"
#include "lawn_mower_tracker/tracker_node.hpp"

using namespace std::chrono_literals;

namespace {

class RosEnvironment : public ::testing::Environment
{
public:
  void SetUp() override {rclcpp::init(0, nullptr);}
  void TearDown() override {rclcpp::shutdown();}
};
const auto ros_environment = ::testing::AddGlobalTestEnvironment(new RosEnvironment);

class TrackerNodeSafetyTest : public ::testing::Test {
protected:
  virtual bool use_avoidance() const {return false;}
  virtual bool raw_mount() const {return false;}

  void SetUp() override
  {
    path_ = std::filesystem::temp_directory_path() / "tracker_node_path.csv";
    std::ofstream(path_) << "pose_x,pose_y,pose_z\n0,0,0\n1,0,0\n2,0,0\n3,0,0\n";
    rclcpp::NodeOptions options;
    options.parameter_overrides({
      {"path_file", path_.string()}, {"control_rate_hz", 50.0},
      {"rtk_timeout_sec", 0.15}, {"z_enabled", true},
      {"csv_yaw_column", "recorded_yaw"}, {"terminal_alignment_distance", 4.0},
      {"terminal_yaw_tolerance", 0.1}, {"avoidance_enabled", use_avoidance()},
      {"wheelbase", .36}, {"obstacle_timeout_sec", .1},
      {"obstacle_cloud_frame", raw_mount() ? "rslidar" : "fairy_optical"},
      {"lidar_origin_height", raw_mount() ? .483976069 : .518976069}});
    tracker_ = std::make_shared<lawn_mower_tracker::TrackerNode>(options);
    probe_ = std::make_shared<rclcpp::Node>("tracker_safety_probe");
    command_subscription_ = probe_->create_subscription<can_client::msg::VehicleCmd>(
      "/vehicle_cmd", rclcpp::QoS(20).reliable(),
      [this](can_client::msg::VehicleCmd::SharedPtr message) {
        std::lock_guard<std::mutex> lock(commands_mutex_);
        commands_.push_back(*message);
      });
    pose_publisher_ = probe_->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/gnss_pose", rclcpp::SensorDataQoS());
    cloud_publisher_ = probe_->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/rslidar_points",rclcpp::SensorDataQoS());
    client_ = probe_->create_client<lawn_mower_tracker::srv::MowerTrackControl>(
      "/mowerTrackControl");
    executor_.add_node(tracker_);
    executor_.add_node(probe_);
    spin_thread_ = std::thread([this]() {executor_.spin();});
    ASSERT_TRUE(client_->wait_for_service(2s));
  }

  void TearDown() override
  {
    executor_.cancel();
    if (spin_thread_.joinable()) {
      spin_thread_.join();
    }
    executor_.remove_node(probe_);
    executor_.remove_node(tracker_);
    probe_.reset();
    tracker_.reset();
    std::filesystem::remove(path_);
  }

  bool wait_for(bool enabled, std::chrono::milliseconds timeout = 500ms)
  {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
      {
        std::lock_guard<std::mutex> lock(commands_mutex_);
        if (!commands_.empty() && commands_.back().enable == enabled) {
          return true;
        }
      }
      std::this_thread::sleep_for(5ms);
    }
    return false;
  }

  void clear_commands()
  {
    std::lock_guard<std::mutex> lock(commands_mutex_);
    commands_.clear();
  }

  bool any_enabled() const
  {
    std::lock_guard<std::mutex> lock(commands_mutex_);
    for (const auto & command : commands_) {
      if (command.enable) {return true;}
    }
    return false;
  }

  std::shared_ptr<lawn_mower_tracker::srv::MowerTrackControl::Response> request(
    const std::string & state)
  {
    auto request = std::make_shared<lawn_mower_tracker::srv::MowerTrackControl::Request>();
    request->track_state = state;
    request->line_id = 7U;
    auto future = client_->async_send_request(request);
    EXPECT_EQ(future.wait_for(1s), std::future_status::ready);
    return future.get();
  }

  void publish_pose(bool valid)
  {
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = 0.0;
    pose.pose.orientation.w = valid ? 1.0 : 0.0;
    pose_publisher_->publish(pose);
  }

  void publish_cloud(const std::string & frame="fairy_optical", float obstacle_x=4, bool invalid_stamp=false)
  {
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.frame_id=frame;
    cloud.header.stamp=probe_->now();
    if (invalid_stamp) {cloud.header.stamp.sec=-1;}
    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2FieldsByString(1,"xyz");
    modifier.resize(1);
    sensor_msgs::PointCloud2Iterator<float> x(cloud,"x"),y(cloud,"y"),z(cloud,"z");
    *x=obstacle_x;*y=0;*z=-.3;
    cloud_publisher_->publish(cloud);
  }
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_publisher_;
  std::filesystem::path path_;
  std::shared_ptr<lawn_mower_tracker::TrackerNode> tracker_;
  std::shared_ptr<rclcpp::Node> probe_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::thread spin_thread_;
  rclcpp::Subscription<can_client::msg::VehicleCmd>::SharedPtr command_subscription_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
  rclcpp::Client<lawn_mower_tracker::srv::MowerTrackControl>::SharedPtr client_;
  mutable std::mutex commands_mutex_;
  std::vector<can_client::msg::VehicleCmd> commands_;
};

class TrackerNodeAvoidanceTest : public TrackerNodeSafetyTest
{
  bool use_avoidance() const override {return true;}
};

TEST_F(TrackerNodeAvoidanceTest, MissingLidarNeverEnablesDrive)
{
  ASSERT_TRUE(request("start")->success);
  for (int i=0;i<10;++i) {publish_pose(true);std::this_thread::sleep_for(20ms);}
  EXPECT_FALSE(any_enabled());
}

TEST_F(TrackerNodeAvoidanceTest, LidarTimeoutStopsEvenWithFreshPoses)
{
  ASSERT_TRUE(request("start")->success);
  publish_pose(true);
  std::this_thread::sleep_for(30ms);
  publish_cloud();
  ASSERT_TRUE(wait_for(true));
  for (int i=0;i<10;++i) {publish_pose(true);std::this_thread::sleep_for(20ms);}
  EXPECT_TRUE(wait_for(false));
  clear_commands();
  for (int i=0;i<5;++i) {publish_pose(true);std::this_thread::sleep_for(20ms);}
  EXPECT_FALSE(any_enabled());
}

TEST_F(TrackerNodeAvoidanceTest, InvalidFrameOrNearbyObstacleStops)
{
  ASSERT_TRUE(request("start")->success);
  publish_pose(true);
  std::this_thread::sleep_for(30ms);
  publish_cloud();
  ASSERT_TRUE(wait_for(true));
  clear_commands();
  publish_cloud("base_link");
  ASSERT_TRUE(wait_for(false));
  std::this_thread::sleep_for(40ms);
  EXPECT_FALSE(any_enabled());
  publish_pose(true);
  publish_cloud("fairy_optical",.4);
  std::this_thread::sleep_for(60ms);
  EXPECT_FALSE(any_enabled());
}

TEST_F(TrackerNodeAvoidanceTest, InvalidTimestampStopsAndCanRecover)
{
  ASSERT_TRUE(request("start")->success);
  publish_pose(true);
  std::this_thread::sleep_for(30ms);
  publish_cloud();
  ASSERT_TRUE(wait_for(true));
  publish_cloud("fairy_optical",4,true);
  ASSERT_TRUE(wait_for(false));
  publish_pose(true);
  std::this_thread::sleep_for(20ms);
  publish_cloud();
  EXPECT_TRUE(wait_for(true));
}

class TrackerNodeRawLidarTest : public TrackerNodeSafetyTest
{
  bool use_avoidance() const override {return true;}
  bool raw_mount() const override {return true;}
};

TEST_F(TrackerNodeRawLidarTest, RealRawFrameDrivesAndOpticalFrameIsRejected)
{
  EXPECT_NEAR(tracker_->get_parameter("lidar_origin_height").as_double(),.483976069,1e-9);
  ASSERT_TRUE(request("start")->success);
  publish_pose(true);
  std::this_thread::sleep_for(30ms);
  publish_cloud("rslidar");
  ASSERT_TRUE(wait_for(true));
  clear_commands();
  publish_cloud("fairy_optical");
  ASSERT_TRUE(wait_for(false));
  std::this_thread::sleep_for(40ms);
  EXPECT_FALSE(any_enabled());
}

TEST_F(TrackerNodeSafetyTest, AvoidanceConfigurationIsAvailable)
{
  EXPECT_TRUE(tracker_->has_parameter("avoidance_enabled"));
  EXPECT_TRUE(tracker_->has_parameter("obstacle_cloud_topic"));
}

TEST_F(TrackerNodeSafetyTest, InvalidRtkDurablyStopsActiveTracking)
{
  ASSERT_TRUE(request("start")->success);
  publish_pose(true);
  ASSERT_TRUE(wait_for(true));
  clear_commands();
  publish_pose(false);
  ASSERT_TRUE(wait_for(false));
  std::this_thread::sleep_for(100ms);
  EXPECT_FALSE(any_enabled());
}

TEST_F(TrackerNodeSafetyTest, InvalidServiceCommandDurablyStopsActiveTracking)
{
  ASSERT_TRUE(request("start")->success);
  publish_pose(true);
  ASSERT_TRUE(wait_for(true));
  clear_commands();
  EXPECT_FALSE(request("invalid")->success);
  ASSERT_TRUE(wait_for(false));
  std::this_thread::sleep_for(100ms);
  EXPECT_FALSE(any_enabled());
}

TEST_F(TrackerNodeSafetyTest, RtkTimeoutStopsAndDoesNotResumeWithoutNewPose)
{
  ASSERT_TRUE(request("start")->success);
  publish_pose(true);
  ASSERT_TRUE(wait_for(true));
  clear_commands();
  ASSERT_TRUE(wait_for(false, 400ms));
  clear_commands();
  std::this_thread::sleep_for(100ms);
  EXPECT_FALSE(any_enabled());
}

TEST_F(TrackerNodeSafetyTest, RejectsPauseWhenNoTaskIsTracking)
{
  EXPECT_EQ(tracker_->get_parameter("csv_yaw_column").as_string(), "recorded_yaw");
  EXPECT_DOUBLE_EQ(tracker_->get_parameter("terminal_alignment_distance").as_double(), 4.0);
  EXPECT_DOUBLE_EQ(tracker_->get_parameter("terminal_yaw_tolerance").as_double(), 0.1);
  EXPECT_FALSE(request("pause")->success);
  EXPECT_TRUE(wait_for(false));
  EXPECT_FALSE(any_enabled());
}

}  // namespace
