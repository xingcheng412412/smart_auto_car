#include <algorithm>
#include <chrono>
#include <cstring>
#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_srvs/srv/trigger.hpp>
#include "fw_mini_pro_slam3d/voxel_map.hpp"
using namespace std::chrono_literals;
namespace fw_mini_pro_slam3d {
class MapAccumulator : public rclcpp::Node {
 public:
  MapAccumulator() : Node("fairy_map") {
    const double voxel=declare_parameter<double>("voxel_size", .15);
    const int limit=declare_parameter<int>("max_voxels", 1000000);
    if (limit<=0) throw std::invalid_argument("max_voxels must be positive");
    map_=std::make_unique<VoxelMap>(voxel,static_cast<size_t>(limit));
    path_=declare_parameter<std::string>("map_file_path", "");
    frame_=declare_parameter<std::string>("map_frame", "camera_init");
    if (path_.empty() || !std::filesystem::path(path_).is_absolute())
      throw std::invalid_argument("Set map_file_path to an absolute .pcd path");
    auto input=declare_parameter<std::string>("cloud_topic", "/slam3d/registered");
    publisher_=create_publisher<sensor_msgs::msg::PointCloud2>("/slam3d/map",rclcpp::QoS(1).transient_local());
    subscriber_=create_subscription<sensor_msgs::msg::PointCloud2>(input,rclcpp::SensorDataQoS().keep_last(5),
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) { receive(*msg); });
    service_=create_service<std_srvs::srv::Trigger>("/slam3d/save_map",
      [this](std_srvs::srv::Trigger::Request::SharedPtr, std_srvs::srv::Trigger::Response::SharedPtr response) {
        try {
          map_->save(path_);
          response->success=!map_->truncated();
          response->message=(map_->truncated()?"INCOMPLETE map (voxel limit reached) saved: ":"Map saved: ")+path_+
            " ("+std::to_string(map_->size())+" voxels)";
        } catch (const std::exception& e) {response->success=false;response->message=e.what();}
        RCLCPP_INFO(get_logger(),"%s",response->message.c_str());
      });
    timer_=create_wall_timer(1s,[this] { publish(); });
  }
 private:
  void receive(const sensor_msgs::msg::PointCloud2& msg) {
    if (msg.header.frame_id!=frame_) {
      RCLCPP_ERROR_THROTTLE(get_logger(),*get_clock(),5000,"Reject cloud frame %s; expected %s",msg.header.frame_id.c_str(),frame_.c_str()); return;
    }
    const int64_t stamp=rclcpp::Time(msg.header.stamp).nanoseconds();
    if (last_stamp_>=stamp && last_stamp_>=0) {
      RCLCPP_ERROR_THROTTLE(get_logger(),*get_clock(),5000,"Non-increasing registered timestamp; restart entire mapping pipeline after sensor reset");return;
    }
    if (msg.is_bigendian || msg.point_step==0 || msg.row_step<static_cast<uint64_t>(msg.width)*msg.point_step ||
        msg.data.size()<static_cast<uint64_t>(msg.row_step)*msg.height) {
      RCLCPP_ERROR(get_logger(),"Invalid registered cloud layout");return;
    }
    std::array<size_t,4> offsets{};
    const std::array<std::string,4> names{"x","y","z","intensity"};
    for (size_t i=0;i<4;++i) {
      auto f=std::find_if(msg.fields.begin(),msg.fields.end(),[&](const auto& f){return f.name==names[i];});
      if (f==msg.fields.end() || f->datatype!=sensor_msgs::msg::PointField::FLOAT32 || f->count!=1 ||
          static_cast<uint64_t>(f->offset)+4>msg.point_step) {
        RCLCPP_ERROR(get_logger(),"Missing/invalid registered field %s",names[i].c_str());return;
      }
      offsets[i]=f->offset;
    }
    for (size_t row=0;row<msg.height;++row) for (size_t col=0;col<msg.width;++col) {
      const auto* point=msg.data.data()+row*msg.row_step+col*msg.point_step;
      std::array<float,4> values;
      for (size_t i=0;i<4;++i) std::memcpy(&values[i],point+offsets[i],4);
      map_->add({values[0],values[1],values[2],values[3]});
    }
    last_stamp_=stamp;header_=msg.header;dirty_=true;
    if (map_->truncated()) RCLCPP_ERROR_THROTTLE(get_logger(),*get_clock(),10000,
      "Map voxel limit reached: new areas omitted. Save and restart with larger max_voxels.");
  }
  void publish() {
    if (!dirty_ || map_->size()==0) return;
    sensor_msgs::msg::PointCloud2 msg;msg.header=header_;
    sensor_msgs::PointCloud2Modifier modifier(msg);
    modifier.setPointCloud2Fields(4,"x",1,sensor_msgs::msg::PointField::FLOAT32,
      "y",1,sensor_msgs::msg::PointField::FLOAT32,"z",1,sensor_msgs::msg::PointField::FLOAT32,
      "intensity",1,sensor_msgs::msg::PointField::FLOAT32);
    auto points=map_->points();modifier.resize(points.size());
    std::memcpy(msg.data.data(),points.data(),points.size()*sizeof(MapPoint));
    msg.is_dense=true;publisher_->publish(msg);dirty_=false;
  }
  std::unique_ptr<VoxelMap> map_;std::string path_,frame_;
  std_msgs::msg::Header header_;int64_t last_stamp_=-1;bool dirty_=false;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscriber_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}
int main(int argc,char** argv) {
  rclcpp::init(argc,argv);
  try { rclcpp::spin(std::make_shared<fw_mini_pro_slam3d::MapAccumulator>()); }
  catch(const std::exception& e) {RCLCPP_FATAL(rclcpp::get_logger("fairy_map"),"%s",e.what());rclcpp::shutdown();return 1;}
  rclcpp::shutdown();return 0;
}
