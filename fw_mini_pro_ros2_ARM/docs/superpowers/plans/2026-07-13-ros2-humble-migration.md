# can_client ROS2 Humble Migration Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Migrate `can_client` from ROS1 Noetic to a buildable and runnable ROS2 Humble package without changing its CAN protocol or public topic/message interfaces.

**Architecture:** Keep one `ament_cmake` package containing rosidl interfaces, one `rclcpp` vehicle node, and two installed `rclpy` tools. Preserve the CAN codec and command behavior while replacing ROS1 node handles, callbacks, timing, launch, build, and CLI integration with their ROS2 equivalents.

**Tech Stack:** Ubuntu 22.04, ROS2 Humble, `ament_cmake`, `rosidl`, `rclcpp`, `rclpy`, Python launch, SocketCAN, C++17

## Global Constraints

- Keep the package name `can_client`.
- Keep `/vehicle_cmd`, `/vehicle_feedback`, `/fault_reset`, `/can_tx`, and `/can_rx` unchanged.
- Keep every custom message field, constant, unit, and sign convention unchanged.
- Keep `can_iface=can0`, `update_rate=20.0`, `max_linear_speed=1.5`, and `max_angular_speed=3.0` as launch defaults.
- Do not alter CAN IDs, bit fields, BCC calculation, gear values, or safety behavior.
- A real CAN interface and VCU are not required for software verification.
- This directory is not a Git repository, so task checkpoints are verified but not committed.

---

## File Map

- `package.xml`: ROS2 package metadata and runtime/build dependencies.
- `CMakeLists.txt`: rosidl generation, C++ build, Python/launch/header installation, lint configuration.
- `msg/CANFrame.msg` and `msg/VehicleFeedback.msg`: qualify `std_msgs/Header` for ROS2 rosidl.
- `include/fw_mini_pro.hpp`: ROS2 message aliases, node-owned publishers/subscriptions, thread lifecycle declarations.
- `src/vehicle_core.cpp`: ROS2 callbacks, parameters, timer, logging, and shutdown; existing CAN codec remains in place.
- `launch/can_client.launch.py`: ROS2 launch arguments and vehicle node declaration.
- `launch/can_client.launch`: removed ROS1 launch file.
- `script/socketcan_bridge.py`: `rclpy` SocketCAN monitor node.
- `upper_computer/app.py`: `rclpy` curses controller.
- `upper_computer/run.sh`: Humble/workspace environment setup.
- `README.md`: ROS2 Humble build, launch, topic, and troubleshooting commands.

### Task 1: ROS2 Build Metadata and Interfaces

**Files:**
- Modify: `package.xml`
- Modify: `CMakeLists.txt`

**Interfaces:**
- Consumes: existing `msg/VehicleCmd.msg`, `msg/VehicleFeedback.msg`, and `msg/CANFrame.msg`
- Produces: generated C++ headers under `can_client/msg/*.hpp`, generated Python modules under `can_client.msg`, and an installable `ament_cmake` package

- [ ] **Step 1: Record the ROS1 baseline failure**

Run:

```bash
bash -lc 'source /opt/ros/humble/setup.bash && colcon --log-base /tmp/can_client_baseline_log build --packages-select can_client --build-base /tmp/can_client_baseline_build --install-base /tmp/can_client_baseline_install'
```

Expected: FAIL because the current project requests catkin/ROS1 dependencies or includes ROS1 headers.

- [ ] **Step 2: Replace package metadata with ROS2 dependencies**

Use package format 3 and declare these exact dependency categories:

```xml
<buildtool_depend>ament_cmake</buildtool_depend>
<build_depend>rosidl_default_generators</build_depend>
<depend>rclcpp</depend>
<depend>rclpy</depend>
<depend>std_msgs</depend>
<exec_depend>rosidl_default_runtime</exec_depend>
<exec_depend>launch</exec_depend>
<exec_depend>launch_ros</exec_depend>
<member_of_group>rosidl_interface_packages</member_of_group>
```

Remove all `catkin`, `roscpp`, `rospy`, `message_generation`, and `message_runtime` declarations. Preserve the current description, maintainer, and license metadata because licensing changes are outside this migration.

- [ ] **Step 3: Replace catkin CMake with ament and rosidl**

The central build declarations must be:

```cmake
cmake_minimum_required(VERSION 3.8)
project(can_client)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/VehicleCmd.msg"
  "msg/VehicleFeedback.msg"
  "msg/CANFrame.msg"
  DEPENDENCIES std_msgs
)

add_executable(vehicle_core src/vehicle_core.cpp)
target_include_directories(vehicle_core PRIVATE include)
ament_target_dependencies(vehicle_core rclcpp std_msgs)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(vehicle_core "${cpp_typesupport_target}")
```

Install `vehicle_core` to `lib/${PROJECT_NAME}`, headers to `include`, launch files to `share/${PROJECT_NAME}/launch`, and Python scripts to `lib/${PROJECT_NAME}`. Export `rosidl_default_runtime` and end with `ament_package()`.

- [ ] **Step 4: Validate metadata before source migration**

Run:

```bash
bash -lc 'source /opt/ros/humble/setup.bash && python3 -c "import xml.etree.ElementTree as E; E.parse(\"package.xml\")" && grep -q ament_package CMakeLists.txt && ! grep -E "catkin|roscpp|rospy" package.xml CMakeLists.txt'
```

Expected: exit code 0. A full build may still fail until Task 2 migrates the C++ source.

### Task 2: Migrate the C++ Vehicle Node

**Files:**
- Modify: `include/fw_mini_pro.hpp`
- Modify: `src/vehicle_core.cpp`

**Interfaces:**
- Consumes: `can_client::msg::VehicleCmd`, `can_client::msg::VehicleFeedback`, `std_msgs::msg::UInt8`, and a shared `rclcpp::Node`
- Produces: executable `vehicle_core`, topic publisher/subscriptions, four declared parameters, and a periodic CAN timer

- [ ] **Step 1: Add a source-level migration guard and verify failure**

Run:

```bash
rg -n '#include <ros/|ros::|ROS_(INFO|WARN|ERROR)|can_client::Vehicle|std_msgs::UInt8' include src
```

Expected: matches exist, proving ROS1 API remains before the migration.

- [ ] **Step 2: Define ROS2 types and ownership in the header**

Replace ROS1 includes with:

```cpp
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <can_client/msg/vehicle_cmd.hpp>
#include <can_client/msg/vehicle_feedback.hpp>
```

Change the driver interface to:

```cpp
virtual void vehicle_cmd(const can_client::msg::VehicleCmd & msg) = 0;
```

Construct `FwMiniProVehicle` with `rclcpp::Node & node` and store `rclcpp::Node * node_`. The driver is owned by the node and destroyed before the node, so this avoids both a reference cycle and `shared_from_this()` during construction. Declare exact ROS2 handles:

```cpp
rclcpp::Publisher<can_client::msg::VehicleFeedback>::SharedPtr feedback_pub_;
rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr fault_reset_sub_;
```

Change `feedback_` to `can_client::msg::VehicleFeedback` and the reset callback to `const std_msgs::msg::UInt8 &`.

- [ ] **Step 3: Migrate driver construction, parameters, callbacks, and logging**

Use declared parameters exactly once:

```cpp
can_iface_ = node_->declare_parameter<std::string>("can_iface", "can0");
max_linear_speed_ = node_->declare_parameter<double>("max_linear_speed", 1.5);
max_angular_speed_ = node_->declare_parameter<double>("max_angular_speed", 3.0);
```

Create publisher/subscription with depth 10, preserve relative topic names, and stamp feedback with:

```cpp
feedback_.header.stamp = node_->get_clock()->now();
```

Replace every ROS1 log macro with `RCLCPP_INFO`, `RCLCPP_WARN`, or `RCLCPP_ERROR` using `node_->get_logger()`.

- [ ] **Step 4: Make the CAN receive thread joinable**

Do not detach the receive thread. Destruction must follow this order:

```cpp
running_ = false;
if (can_sock_ >= 0) {
  ::shutdown(can_sock_, SHUT_RDWR);
  ::close(can_sock_);
  can_sock_ = -1;
}
if (recv_thread_.joinable()) {
  recv_thread_.join();
}
```

This guarantees `recv_loop()` cannot access destroyed members.

- [ ] **Step 5: Replace the ROS1 loop with a ROS2 timer node**

`VehicleCore` must derive from `rclcpp::Node`, create its driver in the constructor with `std::make_unique<FwMiniProVehicle>(*this)`, and own:

```cpp
rclcpp::Subscription<can_client::msg::VehicleCmd>::SharedPtr vehicle_cmd_sub_;
rclcpp::TimerBase::SharedPtr timer_;
std::unique_ptr<VehicleInferface> vehicle_;
```

Declare `update_rate` with default `20.0`; if it is non-positive, warn and use `20.0`. The timer duration is `std::chrono::duration<double>(1.0 / update_rate)`. The callback invokes `vehicle_->run()`.

Use this ROS2 main lifecycle:

```cpp
rclcpp::init(argc, argv);
auto node = std::make_shared<VehicleCore>();
rclcpp::spin(node);
node->shutdown_vehicle();
rclcpp::shutdown();
return 0;
```

- [ ] **Step 6: Build the C++ node and interfaces**

Run:

```bash
bash -lc 'source /opt/ros/humble/setup.bash && colcon --log-base /tmp/can_client_log build --packages-select can_client --build-base /tmp/can_client_build --install-base /tmp/can_client_install --event-handlers console_direct+'
```

Expected: package finishes successfully and `/tmp/can_client_install/lib/can_client/vehicle_core` exists.

- [ ] **Step 7: Verify no ROS1 C++ API remains**

Run:

```bash
! rg -n '#include <ros/|ros::|ROS_(INFO|WARN|ERROR)|can_client::Vehicle|std_msgs::UInt8' include src
```

Expected: exit code 0 with no matches.

### Task 3: ROS2 Launch File

**Files:**
- Delete: `launch/can_client.launch`
- Create: `launch/can_client.launch.py`

**Interfaces:**
- Consumes: installed `can_client/vehicle_core`
- Produces: `ros2 launch can_client can_client.launch.py` with four overridable arguments

- [ ] **Step 1: Verify the ROS1 launch file is incompatible**

Run:

```bash
bash -lc 'source /opt/ros/humble/setup.bash && ros2 launch ./launch/can_client.launch'
```

Expected: FAIL because the file uses ROS1 XML node attributes.

- [ ] **Step 2: Create the Python launch description**

Implement these exact declarations and parameter mappings:

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument('can_iface', default_value='can0'),
        DeclareLaunchArgument('update_rate', default_value='20.0'),
        DeclareLaunchArgument('max_linear_speed', default_value='1.5'),
        DeclareLaunchArgument('max_angular_speed', default_value='3.0'),
    ]
    vehicle_core = Node(
        package='can_client',
        executable='vehicle_core',
        name='vehicle_core',
        output='screen',
        parameters=[{
            'can_iface': LaunchConfiguration('can_iface'),
            'update_rate': ParameterValue(LaunchConfiguration('update_rate'), value_type=float),
            'max_linear_speed': ParameterValue(LaunchConfiguration('max_linear_speed'), value_type=float),
            'max_angular_speed': ParameterValue(LaunchConfiguration('max_angular_speed'), value_type=float),
        }],
    )
    return LaunchDescription(arguments + [vehicle_core])
```

The explicit `ParameterValue` wrappers ensure Humble passes the three numeric arguments as doubles instead of strings.

- [ ] **Step 3: Validate and install the launch file**

Run:

```bash
python3 -m py_compile launch/can_client.launch.py
bash -lc 'source /opt/ros/humble/setup.bash && colcon --log-base /tmp/can_client_log build --packages-select can_client --build-base /tmp/can_client_build --install-base /tmp/can_client_install && test -f /tmp/can_client_install/share/can_client/launch/can_client.launch.py'
```

Expected: both commands exit 0.

### Task 4: Migrate the SocketCAN Debug Tool

**Files:**
- Modify: `script/socketcan_bridge.py`

**Interfaces:**
- Consumes: `can_client.msg.CANFrame`, parameter `can_iface`, topic `/can_tx`
- Produces: topic `/can_rx` and installed executable `socketcan_bridge.py`

- [ ] **Step 1: Verify the ROS1 dependency exists**

Run:

```bash
rg -n 'rospy' script/socketcan_bridge.py
```

Expected: matches exist.

- [ ] **Step 2: Convert the class to `rclpy.node.Node`**

Use this node skeleton while preserving the current CAN packing logic:

```python
import rclpy
from rclpy.node import Node
from can_client.msg import CANFrame


class SocketCanBridge(Node):
    def __init__(self):
        super().__init__('socketcan_bridge')
        self.declare_parameter('can_iface', 'can0')
        self.iface = self.get_parameter('can_iface').value
        self.pub = self.create_publisher(CANFrame, '/can_rx', 100)
        self.sub = self.create_subscription(CANFrame, '/can_tx', self.tx_callback, 100)
```

In `rx_loop`, loop on `rclpy.ok()`, set `msg.header.stamp = self.get_clock().now().to_msg()`, and log via `self.get_logger()`. Close the socket in a dedicated `close()` method.

- [ ] **Step 3: Add a ROS2 main lifecycle**

```python
def main(args=None):
    rclpy.init(args=args)
    node = SocketCanBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

- [ ] **Step 4: Verify syntax and remove ROS1 API**

Run:

```bash
python3 -m py_compile script/socketcan_bridge.py
! rg -n 'rospy' script/socketcan_bridge.py
```

Expected: exit code 0.

### Task 5: Migrate the Curses Upper Computer

**Files:**
- Modify: `upper_computer/app.py`
- Modify: `upper_computer/run.sh`

**Interfaces:**
- Consumes: `/vehicle_feedback`, curses keyboard input, ROS2 generated message constants
- Produces: 10 Hz `/vehicle_cmd`, `/fault_reset` pulse, and installed executable `app.py`

- [ ] **Step 1: Record ROS1 usage**

Run:

```bash
rg -n 'rospy|noetic|autoware\.ai' upper_computer
```

Expected: matches exist.

- [ ] **Step 2: Introduce an `UpperComputerNode`**

Use a ROS2 Node to own communication:

```python
class UpperComputerNode(Node):
    def __init__(self):
        super().__init__('upper_computer')
        self.vehicle_cmd_pub = self.create_publisher(VehicleCmd, '/vehicle_cmd', 10)
        self.fault_reset_pub = self.create_publisher(UInt8, '/fault_reset', 10)
        self.feedback_sub = self.create_subscription(
            VehicleFeedback, '/vehicle_feedback', feedback_callback, 10)
        self.publish_timer = self.create_timer(0.1, self.publish_command)

    def publish_command(self):
        msg = VehicleCmd()
        msg.enable = state['enabled']
        msg.gear = state['gear']
        msg.linear_x = state['linear_x']
        msg.linear_y = state['linear_y']
        msg.angular_z = state['angular_z']
        self.vehicle_cmd_pub.publish(msg)
```

Replace the R-key global publisher call with `node.fault_reset_pub.publish(msg)`. Make the curses loop stop when `rclpy.ok()` becomes false. Preserve all key bindings, drawing, speed limits, and exit-state clearing.

- [ ] **Step 3: Use ROS2 initialization and executor cleanup**

The main block must initialize rclpy, instantiate `UpperComputerNode`, spin it in a daemon thread, run curses, publish a final disabled command, then destroy the node and call `rclpy.shutdown()`.

- [ ] **Step 4: Update the launcher environment**

`run.sh` must source Humble and optionally source the nearest workspace install:

```bash
source /opt/ros/humble/setup.bash

WORKSPACE_SETUP="$(cd "$SCRIPT_DIR/../.." && pwd)/install/setup.bash"
if [[ -f "$WORKSPACE_SETUP" ]]; then
  source "$WORKSPACE_SETUP"
fi
```

Invoke the installed node when available via `ros2 run can_client app.py --ros-args`; otherwise provide a clear instruction to build/source the workspace. Do not source Noetic or Autoware.AI.

- [ ] **Step 5: Verify Python/shell migration**

Run:

```bash
python3 -m py_compile upper_computer/app.py
bash -n upper_computer/run.sh
! rg -n 'rospy|noetic|autoware\.ai' upper_computer
```

Expected: exit code 0.

### Task 6: Rewrite README for ROS2 Humble

**Files:**
- Modify: `README.md`

**Interfaces:**
- Consumes: final build, launch, executable, parameter, and topic names
- Produces: complete ROS2 Humble user instructions

- [ ] **Step 1: Record stale ROS1 documentation**

Run:

```bash
rg -n 'ROS1|Noetic|catkin|roslaunch|rosrun|rostopic|rosnode|autoware\.ai' README.md
```

Expected: multiple matches.

- [ ] **Step 2: Replace build and launch instructions**

Document this workflow:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
# place this can_client directory here
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select can_client
source install/setup.bash
ros2 launch can_client can_client.launch.py
```

Show launch overrides as `can_iface:=can0 update_rate:=20.0 max_linear_speed:=1.5 max_angular_speed:=3.0`.

- [ ] **Step 3: Replace every ROS1 CLI example**

Use ROS2 type names and `--rate`/`--once` syntax, for example:

```bash
ros2 topic pub --rate 20 /vehicle_cmd can_client/msg/VehicleCmd \
  "{enable: true, gear: 6, linear_x: 0.5, linear_y: 0.0, angular_z: -0.436}"

ros2 topic pub --once /fault_reset std_msgs/msg/UInt8 "{data: 1}"
ros2 topic echo /vehicle_feedback
ros2 topic hz /vehicle_cmd
ros2 node list
```

Document `ros2 run can_client app.py` and `ros2 run can_client socketcan_bridge.py --ros-args -p can_iface:=can0`.

- [ ] **Step 4: Verify all stale instructions are removed**

Run:

```bash
! rg -n 'ROS1|Noetic|catkin|roslaunch|rosrun|rostopic|rosnode|autoware\.ai' README.md
rg -n 'ROS2 Humble|colcon build|ros2 launch|ros2 topic' README.md
```

Expected: the first command has no matches; the second shows the new Humble instructions.

### Task 7: Full Software Verification

**Files:**
- Verify: all modified files

**Interfaces:**
- Consumes: complete migrated package
- Produces: evidence that the package builds, installs, exposes interfaces, parses launch, and contains no ROS1 API

- [ ] **Step 1: Run static checks**

```bash
python3 -m py_compile launch/can_client.launch.py script/socketcan_bridge.py upper_computer/app.py
bash -n upper_computer/run.sh
! rg -n 'catkin|roscpp|rospy|roslaunch|rosrun|rostopic|rosnode|Noetic|autoware\.ai' --glob '!docs/superpowers/**' .
```

Expected: all commands exit 0.

- [ ] **Step 2: Perform a clean isolated build**

```bash
bash -lc 'source /opt/ros/humble/setup.bash && colcon --log-base /tmp/can_client_final_log build --packages-select can_client --build-base /tmp/can_client_final_build --install-base /tmp/can_client_final_install --event-handlers console_direct+'
```

Expected: `Summary: 1 package finished` and no failed packages.

- [ ] **Step 3: Verify installed interfaces and executables**

```bash
bash -lc 'source /opt/ros/humble/setup.bash && source /tmp/can_client_final_install/setup.bash && ros2 interface show can_client/msg/VehicleCmd && ros2 interface show can_client/msg/VehicleFeedback && ros2 interface show can_client/msg/CANFrame && ros2 pkg executables can_client'
```

Expected: all three message definitions print, and `vehicle_core`, `app.py`, and `socketcan_bridge.py` appear.

- [ ] **Step 4: Validate launch description without CAN hardware**

```bash
bash -lc 'source /opt/ros/humble/setup.bash && source /tmp/can_client_final_install/setup.bash && ros2 launch can_client can_client.launch.py --show-args'
```

Expected: `can_iface`, `update_rate`, `max_linear_speed`, and `max_angular_speed` are listed with the specified defaults.

- [ ] **Step 5: Report the hardware boundary**

Record that build, package discovery, message generation, executable installation, and launch parsing were verified. State explicitly that SocketCAN traffic, VCU feedback, emergency-stop behavior, and physical motion still require a configured 500 kbit/s CAN interface and vehicle hardware.
