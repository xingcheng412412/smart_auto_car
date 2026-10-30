# FW-mini-pro Gazebo Fortress 仿真

ROS 2 Humble / Ubuntu 22.04。四轮独立转向和驱动，使用 Gazebo 接触物理与关节力控制，定位来自物理世界的测量位姿。

## 安装

在本机终端执行（sudo 密码只在终端输入）：

```bash
sudo apt-get update
sudo apt-get install -y ignition-fortress ros-humble-ros-gz
ign gazebo --versions
```

Gazebo Fortress 是 Humble 官方配套版本：<https://gazebosim.org/docs/latest/ros_installation/>。
复用了厂家 RAR 内的 STL，运行不需要 FreeCAD、STEP 转换器或 libignition-gazebo6-dev。

## 编译和启动

在工作区根目录：

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-up-to fw_mini_pro_sim --cmake-args -DAMENT_CMAKE_SYMLINK_INSTALL=OFF
source install/setup.bash
ros2 launch fw_mini_pro_sim simulation.launch.py
```

`gui:=false` 仅启动物理服务器。默认不启动跟踪器，可以先手动检查运动：

```bash
ros2 topic pub -r 20 /vehicle_cmd can_client/msg/VehicleCmd \
  '{enable: true, gear: 6, linear_x: 0.3, linear_y: 0.0, angular_z: 0.0}'
```

Ctrl+C 停止指令后，适配器在 0.5 秒内下发零轮速（实际停止有物理制动过程）。
倒车设置 `linear_x: -0.3`；转弯设置 `linear_x: 0.3, angular_z: 0.3`；
原地转向设置 `linear_x: 0.0, angular_z: 0.5`；横移设置 `gear: 8, linear_x: 0.0, linear_y: 0.3`。

## 跟踪器闭环

关闭上一套仿真，启动：

```bash
ros2 launch fw_mini_pro_sim simulation.launch.py tracker:=true
```

另一终端 source 同一工作区后启动任务：

```bash
ros2 service call /mowerTrackControl lawn_mower_tracker/srv/MowerTrackControl \
  "{track_state: start, map_name: '', line_id: 1}"
ros2 topic echo /path_status
```

默认轨迹为从原点前进 3 m 的 `paths/straight.csv`。另有 `paths/turn.csv`（左转 90°）和
`paths/reverse.csv`（保持 yaw=0 倒车 3 m）。每次切换示例需重启仿真使车辆回到原点：

```bash
ros2 launch fw_mini_pro_sim simulation.launch.py tracker:=true \
  path_file:="$(ros2 pkg prefix --share fw_mini_pro_sim)/paths/turn.csv"
```

`path_file:=/绝对路径/轨迹.csv` 可使用现有录制路径，但其坐标原点和朝向必须与仿真世界一致；
示例世界车辆在 `(0,0)`、朝向 `+X`。RTK 的大地/投影坐标路径不能不经对齐直接使用。
`tracker_params:=/绝对路径/参数.yaml` 可覆盖仿真跟踪参数。

## 接口和行为

| 接口 | 类型 | 含义 |
|---|---|---|
| `/vehicle_cmd` | `can_client/msg/VehicleCmd` | 现有控制器输入 |
| `/gnss_pose` | `geometry_msgs/msg/PoseStamped` | Gazebo 实测位姿，map 坐标系 |
| `/odom` | `nav_msgs/msg/Odometry` | Gazebo 实测里程计 |
| `/vehicle_feedback` | `can_client/msg/VehicleFeedback` | 实测速度和模拟档位 |
| `/joint_states` | `sensor_msgs/msg/JointState` | 八个关节的实测状态 |
| `/clock` | `rosgraph_msgs/msg/Clock` | Gazebo 时间 |
| `/tf`、`/tf_static` | TF | map→odom（恒等）、odom→base_link |

6 档使用 vx/wz，8 档使用 vx/vy。禁用、其他档位、非有限数值、命令或关节反馈超时均停止驱动。
同时检查墙钟和仿真时钟的指令年龄；仿真时间回退时清除旧指令。转向误差较大时先转轮，再驱动。
各轮速度整体限幅到 1.5 m/s，保持运动曲率；角速度另限 3 rad/s。
仿真入口只启动仿真接口，不启动实车 CAN 驱动；同一 ROS domain 内不要同时启动实车驱动或另一套控制发布器。

## 模型依据和限制

手册：长宽高 565×380×255 mm、轴距 360 mm、轮距 280 mm、轮径 140 mm、整车质量 30 kg。
车体 22 kg，四个转向组件各 0.5 kg，四个轮子各 1.5 kg；惯量采用几何估计。
外观来自用户 RAR 的厂家 STL，拆成九个网格；细节见 `models/fw_mini_pro/meshes/SOURCE.md`。

四轮目标速度满足 `wheel_vx = vx - wz*y`、`wheel_vy = vy + wz*x`。
这是四轮转向刚体运动学，不使用差速插件代替。跟踪器的自行车等效轴距在仿真配置中设为 0.36 m，
并使用较短前视和较低速度；实车 `tracker.yaml` 的原值 1.45 m 未修改。

质量分配、摩擦系数、驱动/转向扭矩和 PID 未经实车辨识。网格按连通组件近似分组，细小零件可能随同组轮体旋转。
无悬挂、轮胎变形、RTK 误差或 CAN 帧仿真；定位为理想测量，不发布 NavSatFix，因此录制器若需要 `/fix` 尚需额外 GNSS 接口。
电池值固定 24 V/100%，不代表电池模型。当前 TF 不含各轮连杆树；Gazebo 内关节显示不受影响。
命令超时保护在适配节点内：其突然崩溃时由 launch 关闭整套仿真；不要单独遗留运行着的 Gazebo 与旧关节命令。

## 验证与重建

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest src/fw_mini_pro_sim/test -q
python3 src/fw_mini_pro_sim/scripts/generate_model.py
```

重建外观需 `python3-numpy`，先将厂家 STL 解压，再运行：

```bash
python3 src/fw_mini_pro_sim/scripts/prepare_vendor_mesh.py /绝对路径/FW-mini-pro.STL
python3 src/fw_mini_pro_sim/scripts/generate_model.py
```

运行验证结果记录于 `VALIDATION.md`；单元测试通过不等于 Gazebo 物理闭环已通过。

自动物理验收（会自行启动独立 ROS domain / Gazebo partition，退出时清理自己的仿真进程）：

```bash
python3 src/fw_mini_pro_sim/scripts/smoke_test.py
python3 src/fw_mini_pro_sim/scripts/smoke_test.py --tracker
```

第一条测量五种运动和超时停车；第二条运行三条示例轨迹，要求跟踪器报告 `6=completed`。
输出包含物理位移、转角、制动后速度，以及临时日志目录。

## 关闭与重新启动

请在启动 `ros2 launch` 的终端按 **Ctrl+C**，等待整套进程退出后再重新启动。仅关闭 Gazebo 图形窗口不会停止后台物理服务器、桥接节点和跟踪器。
同一 `ROS_DOMAIN_ID` / `IGN_PARTITION` 中同时运行两套相同仿真，会产生重复时钟、定位及控制发布者，导致画面和定位跳变。

对 `map_01` 进行完整物理闭环验收（最长等待 600 秒）：

```bash
python3 src/fw_mini_pro_sim/scripts/smoke_test.py --tracker \
  --path-file map_01/simulation_local.csv --timeout 600
```

该脚本自行使用独立通信环境，从原点启动，要求收到完成状态并检查实测终点距离与朝向；
结束后清理自己的仿真进程，不操作手动打开的那套仿真。

## Fairy 雷达与 SLAM

车顶上方 0.3 m 支架、三维点云和二维建图已接入独立 `slam.launch.py`。启动、运动、地图保存、安装坐标与实测限制见 [SLAM.md](SLAM.md)。

## RTK 话题仿真

启用 `rtk:=true`，由独立传感器节点发布 `/gnss_pose`、`/fix` 和真实驱动消息类型的 `/rtk_log`，真值改到 `/sim/ground_truth_pose`。无需 RTK 硬件，不启动串口驱动。

本机缺少 GeographicLib 时可仅构建驱动消息接口：

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select rtk_gps_driver_cpp --cmake-args -DRTK_INTERFACES_ONLY=ON
colcon build --packages-select fw_mini_pro_sim
source install/setup.bash
ros2 launch fw_mini_pro_sim simulation.launch.py rtk:=true tracker:=true
```

跟踪任务仍需按上文调用 `/mowerTrackControl` start。今后构建真实串口驱动时需先安装其依赖，再显式传 `-DRTK_INTERFACES_ONLY=OFF`（CMake 会缓存 ON）。

配置文件 `config/rtk_sim.yaml`：输出上限 10 Hz，位置每轴独立高斯噪声标准差 0.02 m，航向噪声标准差 0.005 rad，随机种子 1。将两项标准差设为 0 可得到理想定位。使用 `rtk_params:=/绝对路径/rtk_sim.yaml` 指定配置。参数在启动时读取。

`latitude/longitude/altitude` 指定世界原点的 WGS84 坐标；默认 (0°, 0°, 0 m) 是合成原点，不代表实际场地。沿用真实驱动的 X北/Y西/Z上约定，yaw=0 朝北。经纬度由带噪位置进行 WGS84 本地切平面反变换得到。位置参考点为 base_link，未加入天线杆臂或实体外观。

手动掉线与恢复：

```bash
ros2 service call /sim/rtk/set_dropout std_srvs/srv/SetBool '{data: true}'
ros2 service call /sim/rtk/set_dropout std_srvs/srv/SetBool '{data: false}'
```

掉线停止发布 `/gnss_pose` 和 `/fix`，`/rtk_log` 继续报告 TIMEOUT；恢复后重新发布。可用 `dropout_start`（相对首个真值的仿真秒，-1 禁用）和 `dropout_duration` 配置一次定时掉线。手动恢复不会覆盖仍在进行的定时掉线。跟踪器应按其定位超时策略停车，恢复定位不等于自动恢复任务。仿真暂停时不重发旧位姿；诊断使用墙钟检测输入超时。噪声只在新测量时采样，不模拟 INS 内部滤波、RTK 浮点解或真实卫星信号。

雷达/SLAM 场景也支持：

```bash
ros2 launch fw_mini_pro_sim slam.launch.py rtk:=true
```

此时定位使用 `rtk_map`，通过恒等 `odom → rtk_map` 关联到仿真世界；SLAM 单独负责 `map → odom`。未实现 RTK/SLAM 融合，不能忽略 frame_id 将 RTK 位置直接用于 SLAM 地图路径。RTK 节点不发布车辆 TF；现有 odom→base_link 仍为理想仿真里程计。

RTK 物理跟踪验收：

```bash
python3 src/fw_mini_pro_sim/scripts/smoke_test.py --tracker --path straight --rtk
```

## 在线 SLAM + RTK 融合定位

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch fw_mini_pro_sim fusion.launch.py
```

使用 `gui:=false rviz:=false` 无界面运行。加 `tracker:=true` 可启动跟踪器；通过原 `/mowerTrackControl` 服务启动任务。跟踪器在此入口订阅 `/localization/pose`，路径必须位于当前 SLAM 的 map 坐标系。初始融合对齐完成前不会输出控制位姿。

| 话题 | 内容 |
|---|---|
| `/gnss_pose` | 原始 RTK 位姿，rtk_map 坐标系 |
| `/slam_toolbox/pose` | 原始 SLAM 位姿与协方差，map 坐标系 |
| `/localization/pose` | 融合位姿，map 坐标系，供跟踪器使用 |
| `/localization/odometry` | 融合二维位姿及协方差，附原始里程计速度 |
| `/localization/status` | FUSED、SLAM_ONLY、RTK_ONLY、ALIGNING、LOCALIZATION_LOST 或 ODOM_TIMEOUT，以及拒绝测量计数 |

算法为二维松耦合 EKF：根据里程计相邻位姿的车体运动增量预测，用 SLAM 的 x/y/yaw 和 RTK 的 x/y/yaw 修正。首次同时收到合格的 SLAM 与 RTK 测量时，通过时间对齐估计 map←rtk_map 的刚体变换，之后固定；它是启动标定，不是持久化地理地图。重启/时间回退重新对齐，已有地图重新定位不属于此入口。

融合模式下仅融合节点发布 `map → odom`；原始 SLAM 关闭此 TF 发布，继续从 odom 获取扫描匹配先验并建图。融合节点还发布标定后的 `map → rtk_map`，取消普通 SLAM+RTK 模式的 `odom → rtk_map` 恒等变换。`odom → base_link` 仍由仿真适配器发布。

`config/fusion.yaml` 配置传感器超时、测量延迟上限和协方差下限；用 `fusion_params:=/绝对路径/fusion.yaml` 覆盖。默认两种绝对定位超过 2 秒无合格更新时停止发布控制位姿，由跟踪器原有 0.5 秒接收超时停车。仍保留 TF 预测以支持建图恢复，因此控制必须使用受保护的定位话题，不能把仍存在的 TF 当成定位有效信号。里程计过期、无效或超过当前 ROS 时间允许的延迟同样不会输出控制位姿。

掉线测试沿用 `/sim/rtk/set_dropout`：RTK 掉线后应转为 SLAM_ONLY；恢复并通过质量、时间和创新门限后回到 FUSED。异常大跳变被拒绝，不自动强行重置到跳变位置。若真正发生地图重定位的大幅跳变，本版需停止任务并重启融合/建图，不支持在线地图坐标重置。

延迟测量用短时里程计历史前推，并增加延迟不确定度；并非完整历史状态重放。SLAM 与预测共享里程计，协方差采用保守下限，但未建模互相关。初始帧对齐为单组时间邻近测量，会带入启动 RTK 噪声；不代表精确外参标定。RTK 噪声增大时应同步调大融合配置的 RTK 标准差。本功能不把 GNSS 因子加入 SLAM 位姿图，因此 RTK 改善车辆定位不等于校正整张地图形变。

当前仍使用理想 Gazebo 里程计；此阶段验收证明故障处理和融合链路，不证明相比单传感器的精度提升。真实 INS 输出之间相关性、天线杆臂、安装角、轮速/IMU 误差及硬件时间同步尚未验证。

自动在线建图与故障验收（独立 ROS 域，自行清理进程）：

```bash
python3 src/fw_mini_pro_sim/scripts/fusion_smoke_test.py
```

SLAM 的位姿输出和关闭 TF 的参数依据 [Humble 上游实现](https://github.com/SteveMacenski/slam_toolbox/blob/humble/src/slam_toolbox_common.cpp)。

实车体停车链路的仿真验收：

```bash
python3 src/fw_mini_pro_sim/scripts/fusion_smoke_test.py --tracker-stop
```

该模式启动跟踪器，确认车辆已运动后同时中断 RTK 和 SLAM，检查禁用命令及 Gazebo 实测速度停止。

## 雷达自动避障演示

`ros2 launch fw_mini_pro_sim avoidance.launch.py` 启动带路径上箱体的独立雷达避障场景，
随后通过 `/mowerTrackControl` 的 `start` 指令开始任务。支持 `gui:=false`。
车体过滤、绕行参数、RViz 话题和完整使用方法见 `lawn_mower_tracker/README.md` 的
“仿真雷达自动避障”章节。默认普通 simulation/fusion 入口不启用此配置。
