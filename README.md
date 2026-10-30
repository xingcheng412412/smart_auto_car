# FW-mini-pro 整车控制与自主路径跟踪项目

本项目是面向 FW-mini-pro 四轮独立转向、四轮驱动底盘的 ROS 2 工作区，主要运行环境为 Ubuntu 22.04 / ROS 2 Humble，整车部署入口面向 RK3588。项目集成底盘 CAN 通信、RTK 定位、路径录制与回放跟踪、三维雷达避障和 SSH 终端控制台，同时提供独立的实物三维建图与 Gazebo 仿真功能。

典型使用流程为：配置车辆与 GNSS 原点 → 启动整车节点 → 录制并保存路径 → 选择路径开始跟踪 → 查看定位、任务和车辆反馈。

## 1. 系统组成

```text
RTK 接收机 ──串口──► rtk_gps_driver_cpp ──► /gnss_pose、/fix、/rtk_log
                                              │
                         ┌────────────────────┴──────────────────┐
                         ▼                                       ▼
                lawn_mower_recorder                    lawn_mower_tracker
                         │                                       ▲
                         └──► 路径 CSV ──────────────────────────┤
                                                                │
RSFAIRY 雷达 ──UDP──► rslidar_sdk ──► /rslidar_points ──► 局部避障规划
                                                                │
                                                                ▼
                                                         /vehicle_cmd
                                                                │
                                                                ▼
                                                   can_client / vehicle_core
                                                                │
                                                             SocketCAN
                                                                │
                                                                ▼
                                                           FW-mini-pro

lawn_mower_console：通过服务控制录制与跟踪，显示 RTK、任务和车辆状态
mower_start：统一启动实车后台节点；one_click：管理后台与交互控制台的启停
```

实物三维建图使用另一条独立链路：雷达点云与 IMU → 数据适配 → FAST-LIO2 → 三维位姿、点云地图与 PCD 保存。该入口不向底盘发送控制指令，也未直接接入上述 RTK 路径跟踪链路。

## 2. 源码目录

目录名与 ROS 包名可能不同，`colcon` 和 `ros2` 命令使用 ROS 包名。

| 源码目录 | ROS 包名 / 组件 | 主要职责 |
| --- | --- | --- |
| `fw_mini_pro_ros2_ARM/` | `can_client` | SocketCAN 底盘驱动、自定义控制与反馈消息、手动调试上位机、指令超时保护 |
| `dy_rtk_gps_driver/` | `rtk_gps_driver_cpp` | RTK 串口数据解析、GNSS 位姿与诊断发布、定位原点持久化 |
| `lawn_mower_recorder/` | `lawn_mower_recorder` | 录制 GNSS 路径、分段信息和标记点，输出 CSV |
| `lawn_mower_tracker/` | `lawn_mower_tracker` | 加载 CSV、前进/倒车跟踪、任务状态管理、雷达局部避障 |
| `lawn_mower_console/` | `lawn_mower_console` | 基于 curses 的 SSH 终端控制台，提供录制和跟踪操作 |
| `mower_start/` | `mower_start` | 整车后台统一 launch 与 GNSS 原点配置 |
| `rslidar/` | `rslidar_sdk` | RoboSense 雷达 ROS 驱动，发布点云及 IMU 数据 |
| `rs_driver/` | `rs_driver` 驱动库 | 雷达 UDP 数据接收、协议解码与点云生成 |
| `rslidar_msg/` | `rslidar_msg` | 雷达原始数据包消息接口 |
| `fw_mini_pro_slam3d/` | `fw_mini_pro_slam3d`、`fast_lio` | 实物 FAIRY 三维建图功能组，包含适配、算法与地图保存 |
| `fw_mini_pro_sim/` | `fw_mini_pro_sim` | Gazebo Fortress 整车物理仿真、RTK 模拟、SLAM、融合定位及避障场景 |
| `robinx_log/` | `robinx_log` | RTK 诊断日志 HTTP / WebSocket 查看服务 |

工作区根目录的 `one_click/` 提供整车启停脚本，`docs/` 存放补充文档，`maps/` 可存放地图。默认录制路径目录为工作区根目录下的 `recorded_files/`，运行录制后生成。

## 3. 环境与编译

以下命令均在工作区根目录执行，即 `src/` 的上一级目录。

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y

# 构建整车运行入口及其依赖
colcon build --packages-up-to mower_start --executor sequential
source install/setup.bash
```

真实 RTK 驱动需要 GeographicLib 等依赖；整车脚本还需要 `python3-yaml`、`iproute2`、`util-linux` 和 Bash 5.1 及以上。串口需有访问权限，CAN 配置脚本需要 sudo 权限。Gazebo 和三维建图的附加依赖见对应模块文档。

若此前为仿真仅构建过 RTK 消息接口，部署实车前应显式恢复完整驱动构建：

```bash
colcon build --packages-select rtk_gps_driver_cpp \
  --cmake-args -DRTK_INTERFACES_ONLY=OFF
```

每个新终端都需要重新加载 ROS 和工作区环境。个别子目录文档保留了历史 ROS 1 用法；本工作区以当前 `package.xml`、CMake 配置和 `.launch.py` 文件为准，使用 ROS 2 / colcon 命令。

## 4. 实车启动与操作

### 配置设备与定位原点

- 编辑工作区根目录的 `one_click/config.sh`，核对 CAN、RTK 串口、雷达配置及路径目录。默认 CAN 为 `can0` / `500000 bit/s`，RTK 为 `/dev/ttyS6` / `460800 baud`。
- 编辑 `mower_start/param/gnss_init_pose.yaml` 的实际 `lat/lon/h`；零值仅是模板。
- 配置雷达网络，确认能够接收 UDP 数据；一键脚本不会修改网卡 IP。
- 路径录制与跟踪必须使用相同的坐标原点。已有路径投入使用后，不应随意重设原点。

也可在驱动收到新鲜有效的固定解后，通过服务保存当前位置为原点：

```bash
ros2 service call /initial_gnss_pose std_srvs/srv/SetBool '{data: true}'
```

检查响应中的 `success`。一键入口将原点保存到源码配置路径，后续启动继续使用。

### 一键启动与停止

在本地交互终端或带终端分配的 SSH 会话（`ssh -t 用户@车辆地址`）中执行：

```bash
./one_click/start.sh
```

该入口启动 CAN、RTK、雷达、录制器、启用避障的跟踪器和终端控制台。启动后等待操作，不自动开始行驶。

| 按键 | 操作 |
| --- | --- |
| `R` / `E` | 开始录制 / 结束并保存 |
| `T` | 选择路径并开始跟踪 |
| `P` / `C` / `S` | 暂停 / 恢复 / 停止跟踪 |
| `Q` | 退出一键控制台并关闭本次整车节点 |

可在启动终端按 Ctrl+C，或在另一终端执行 `./one_click/stop.sh`。日志保存在 `log/one_click/`。单独执行 `ros2 run lawn_mower_console lawn_mower_console` 时，`Q` 仅退出控制台，不自动停止后台任务。

只启动后台节点时，先加载环境，再执行：

```bash
ros2 launch mower_start bringup.launch.py \
  gnss_config_file:="$PWD/src/mower_start/param/gnss_init_pose.yaml" \
  path_root:="$PWD/recorded_files"
```

此命令不初始化 CAN，也不启动控制台。完整设备配置与进程管理说明见 [整车一键启停](../one_click/README.md)。

## 5. 主要接口与路径数据

| 类型 | 名称 | 用途 |
| --- | --- | --- |
| 话题 | `/gnss_pose` | `geometry_msgs/msg/PoseStamped`，录制和跟踪使用的定位位姿 |
| 话题 | `/fix` | `sensor_msgs/msg/NavSatFix`，经纬高数据 |
| 话题 | `/rtk_log` | `rtk_gps_driver_cpp/msg/RTKGPSLog`，定位诊断 |
| 话题 | `/rslidar_points` | `sensor_msgs/msg/PointCloud2`，雷达点云 |
| 话题 | `/vehicle_cmd` | `can_client/msg/VehicleCmd`，底盘控制命令 |
| 话题 | `/vehicle_feedback` | `can_client/msg/VehicleFeedback`，底盘反馈 |
| 话题 | `/record_status`、`/path_status` | 录制状态与跟踪任务状态 |
| 服务 | `/process_switch`、`/record_mark_point` | 控制路径录制与标记点记录 |
| 服务 | `/mowerTrackControl` | 跟踪任务的 `start`、`pause`、`resume`、`stop` |
| 服务 | `/initial_gnss_pose` | 将当前位置持久化为 GNSS 原点 |

默认路径保存结构：

```text
recorded_files/
└── map_001/
    ├── lawn_mower_records.csv
    └── mark_points.csv          # 使用标记点功能时生成
```

路径包含 `pose_x/pose_y/pose_z`、姿态、速度、路径分段和经纬度等字段。跟踪器至少需要三个有效路径点；`yaw` 用于终点朝向，`linear_velocity` 和 `path_id` 可用于前进/倒车分段判断。详细格式见 [路径录制](lawn_mower_recorder/README.md) 和 [路径跟踪](lawn_mower_tracker/README.md)。

RTK 局部坐标采用 X 北、Y 西、Z 上约定。实车路径、仿真世界和 SLAM 地图的坐标需明确对齐后使用。

## 6. 仿真与建图入口

以下为独立场景入口，需先按对应模块文档完成构建，不会随整车一键入口自动启动。

| 场景 | 启动命令 |
| --- | --- |
| Gazebo 基础整车仿真 | `ros2 launch fw_mini_pro_sim simulation.launch.py` |
| RTK 模拟与路径跟踪 | `ros2 launch fw_mini_pro_sim simulation.launch.py rtk:=true tracker:=true` |
| 仿真雷达避障 | `ros2 launch fw_mini_pro_sim avoidance.launch.py` |
| 仿真在线 SLAM 与 RTK 融合 | `ros2 launch fw_mini_pro_sim fusion.launch.py` |
| 实物 FAIRY 三维建图 | `ros2 launch fw_mini_pro_slam3d mapping.launch.py` |
| RTK 网页日志服务 | `ros2 launch robinx_log rtkgpslog_web_viewer.launch.py` |

仿真跟踪器启动后仍需调用 `/mowerTrackControl` 开始任务。实物建图停止前需调用 `/slam3d/save_map` 保存 PCD，Ctrl+C 不自动保存。避免在同一 ROS domain 同时运行真实 CAN 驱动与仿真控制接口；雷达驱动也应避免重复启动而争用 UDP 端口。

## 7. 验证与实现边界

核心模块提供单元测试及接口、启动配置测试，构建后可执行：

```bash
colcon test --packages-select can_client lawn_mower_recorder \
  lawn_mower_tracker lawn_mower_console mower_start
colcon test-result --verbose
```

仿真还提供 `fw_mini_pro_sim/scripts/` 下的物理闭环与故障验收脚本，运行方式见模块 README。软件测试不能替代实车通信、制动能力、安装外参与现场路径效果验收。

跟踪器在暂停、停止、定位超时等情况下输出禁用零速；CAN 驱动默认使用 0.25 秒指令超时保护。一键实车入口默认限速 0.15 m/s、角速度上限 0.8 rad/s。雷达避障主要面向近似平地，不包含悬崖检测、动态障碍预测或复杂死路倒车脱困。FAST-LIO2 三维建图不含回环优化与地图重定位；仿真融合定位的验证不代表已完成实车融合部署。

## 8. 模块文档

- [底盘 CAN 驱动](fw_mini_pro_ros2_ARM/README.md)
- [路径录制](lawn_mower_recorder/README.md)
- [路径跟踪与雷达避障](lawn_mower_tracker/README.md)
- [SSH 终端控制台](lawn_mower_console/README.md)
- [Gazebo 仿真、RTK 模拟与融合定位](fw_mini_pro_sim/README.md)
- [实物 FAIRY 三维建图](fw_mini_pro_slam3d/README.md)
- [RoboSense 雷达 ROS 驱动](rslidar/README_CN.md)
- [雷达底层驱动库](rs_driver/README_CN.md)
- [整车一键启停](../one_click/README.md)
