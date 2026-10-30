# can_client ROS2 Humble 迁移设计

## 目标

将当前 ROS1 Noetic/catkin 工程完整迁移到 Ubuntu 22.04 上的 ROS2 Humble，使其能够通过 `colcon build` 编译，并通过 ROS2 命令启动车辆驱动、上位机和 SocketCAN 调试工具。

迁移不改变 FW-mini-pro CAN 协议、车辆控制逻辑、自定义消息字段和现有话题名称。

## 范围

本次迁移包括：

- 将构建系统从 catkin 改为 `ament_cmake`。
- 使用 `rosidl_default_generators` 生成三个自定义消息。
- 将车辆 C++ 节点从 `roscpp` 改为 `rclcpp`。
- 将上位机和 SocketCAN 调试工具从 `rospy` 改为 `rclpy`。
- 将 ROS1 XML launch 改为 ROS2 Python launch。
- 将运行脚本和 README 更新为 ROS2 Humble 用法。
- 使用本机 ROS2 Humble 环境执行构建和非硬件级验证。

不包括：

- 修改 CAN 帧 ID、位域、单位、校验或档位定义。
- 拆分新的消息包或驱动包。
- 引入 Lifecycle Node、Composable Node 或新的控制接口。
- 依赖真实车辆完成硬件通信验收。

## 包结构与构建

继续使用单个 `can_client` 包。`package.xml` 升级为 format 3，并声明 `ament_cmake`、`rclcpp`、`rclpy`、`std_msgs`、`rosidl_default_generators` 和 `rosidl_default_runtime` 等依赖。

`CMakeLists.txt` 将：

1. 调用 `rosidl_generate_interfaces` 生成 `VehicleCmd`、`VehicleFeedback` 和 `CANFrame`。
2. 构建 `vehicle_core` C++ 可执行文件并链接本包生成的消息类型支持。
3. 安装头文件、launch 文件及两个 Python 可执行脚本。
4. 通过 `ament_package()` 导出 ROS2 包。

## C++ 节点设计

`VehicleCore` 迁移为 `rclcpp::Node`，负责：

- 声明并读取 `can_iface`、`update_rate`、`max_linear_speed` 和 `max_angular_speed` 参数。
- 订阅 `/vehicle_cmd`。
- 按 `update_rate` 创建 wall timer，周期发送 CAN 心跳并发布 `/vehicle_feedback`。
- 在退出时发送禁用指令。

`FwMiniProVehicle` 保留现有 CAN 打包、解析和安全限制逻辑，通过 ROS2 Node 创建 `/vehicle_feedback` publisher 和 `/fault_reset` subscription。消息头时间戳使用 ROS2 时钟。

接收线程不再 detach。析构时先停止循环、关闭 socket，再 join 线程，避免对象销毁后线程继续访问成员。

## Python 节点设计

`socketcan_bridge.py` 使用 `rclpy.node.Node` 实现 `/can_tx` 订阅与 `/can_rx` 发布，保留 SocketCAN 帧格式和扩展帧处理。日志节流按 ROS2 可用接口实现，退出时关闭 socket 并销毁节点。

`upper_computer/app.py` 使用 `rclpy` publisher、subscription 和后台 spin。curses 操作方式、速度限制、档位切换、使能与故障复位行为保持不变。

## 启动与命令行接口

新增 `launch/can_client.launch.py`，声明以下参数并传递给 `vehicle_core`：

- `can_iface`，默认 `can0`
- `update_rate`，默认 `20.0`
- `max_linear_speed`，默认 `1.5`
- `max_angular_speed`，默认 `3.0`

README 使用 ROS2 Humble 命令：

- `colcon build --packages-select can_client`
- `ros2 launch can_client can_client.launch.py`
- `ros2 run can_client app.py`
- `ros2 topic pub`、`ros2 topic echo` 和 `ros2 topic hz`

`upper_computer/run.sh` 加载 `/opt/ros/humble/setup.bash`，并在存在时加载工作空间的 `install/setup.bash`。

## 兼容性约束

以下接口保持不变：

- 话题：`/vehicle_cmd`、`/vehicle_feedback`、`/fault_reset`、`/can_tx`、`/can_rx`
- 消息包名和类型名：`can_client/msg/VehicleCmd`、`VehicleFeedback`、`CANFrame`
- 所有消息字段、常量、物理单位与正负方向
- launch 参数名称和默认值

ROS2 命令语法、生成头文件名称和 Python 消息导入方式按 Humble 规范变化。

## 错误处理

- CAN 接口打开失败时记录错误，节点继续存活但不发送 CAN 帧，便于无硬件环境检查 ROS 图。
- SocketCAN 读写异常应记录日志，节点关闭时的预期 socket 错误不重复报警。
- `update_rate` 必须为正值；无效值回退到默认的 20 Hz 并记录警告。
- 节点退出时清零速度、切换 disable，并释放 socket 和接收线程。

## 验证

1. 在 `/opt/ros/humble` 环境执行 `colcon build --packages-select can_client`。
2. 加载生成的 `install/setup.bash`，验证包和三个可执行程序可被 ROS2 发现。
3. 在没有 CAN 硬件的情况下启动 launch，确认参数解析和错误日志符合预期。
4. 使用 `ros2 interface show` 检查三个消息定义。
5. 对 Python 文件执行语法编译检查，对 launch 文件执行导入/描述检查。

真实 CAN 收发、VCU 响应和车辆运动属于硬件验收，需要连接配置为 500 kbit/s 的 CAN 接口后执行。
