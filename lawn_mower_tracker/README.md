# lawn_mower_tracker（ROS2 Humble）

基于 RTK 位姿和 CSV 路径的 FW-mini-pro 底盘跟踪节点。

> 真车避障入口为 `ros2 launch lawn_mower_tracker avoidance.launch.py`。车体尺寸和安装沿用模型，原始雷达坐标单独配置；实车行驶效果尚未现场验证。旧 `tracker.launch.py` 保留原来的不带避障跟踪配置。

## 接口

| 类型 | 名称 | 消息 |
|---|---|---|
| 订阅 | `/gnss_pose` | `geometry_msgs/msg/PoseStamped` |
| 发布 | `/vehicle_cmd` | `can_client/msg/VehicleCmd` |
| 发布 | `/path_status` | `lawn_mower_tracker/msg/PathStatus` |
| 服务 | `/mowerTrackControl` | `lawn_mower_tracker/srv/MowerTrackControl` |

跟踪器输出的线速度 `v` 和转向角 `delta` 按自行车模型转换：

```text
angular_z = v * tan(delta) / wheelbase
```

行驶时使用四转四驱档。暂停、停止、到达终点、RTK 超时、路径错误或节点退出时发布禁用零速命令。

## 路径文件

在 `config/tracker.yaml` 中选择一种方式：

- 设置 `path_file` 为完整 CSV 文件路径；或
- 设置 `path_root`，服务 start 时传入单层 `map_name`，节点读取 `path_root/map_name/path_filename`。

默认 `path_root` 为：

```text
~/fw_mini_pro_control_ros2/recorded_files
```

例如 `map_name=map_001` 时读取
`~/fw_mini_pro_control_ros2/recorded_files/map_001/lawn_mower_records.csv`。
节点会将 `~` 展开为当前用户的 home 目录。

CSV 默认坐标列为 `pose_x`、`pose_y`、`pose_z`，至少需要三个有效点。
所有坐标必须是有限数值。

可选列 `yaw` 记录车体朝向。存在该列时，每一行都必须是有限数值，跟踪器使用
最后一个点的 yaw 作为终点车体朝向；倒车终点也直接使用记录值，不额外加 π。
倒车末段的转向计算会将记录的车体 yaw 转为反向行进方向后参与混合，避免在直线倒车时引入错误转向；终点完成判断仍使用原始车体 yaw。

旧 CSV 没有 `yaw` 时，跟踪器跳过末尾静止重复点，从最后一段有效路径推算车体
朝向；如果整条路径无法提供有效末段方向，则拒绝启动。

可选列 `linear_velocity` 和 `path_id` 用于往返路径的前进/倒车判断：

- `linear_velocity > 0.01 m/s` 表示前进；
- `linear_velocity < -0.01 m/s` 表示倒车；
- 绝对值不超过 `0.01 m/s` 表示静止；
- `path_id` 只标识路径分段，其数值本身不代表行驶方向。

折返点附近的连续静止重复点不会遮蔽换向。跟踪器在合法换向区先输出一个
控制周期的零速度，再切换前进/倒车。合法换向时将路径进度同步提交到新方向段的第一个点，避免倒退离开换向半径后重新采用旧段方向。距离过短的速度符号翻转视为静止抖动，
前视目标在车辆进入换向区前不会跨越方向边界。终点后的重复静止点不影响完成
判定。旧 CSV 可以不包含这两个可选列，此时使用跳过零长度线段的几何换向检测。

进入终点前默认 `3.0 m` 后，跟踪器逐渐提高终点 yaw 在转向控制中的权重，
使车辆在到点前完成朝向收敛。只有终点距离不超过 `0.2 m`，且 yaw 误差不超过
`0.174532925 rad`（10°）才报告完成。若位置已经到达但 yaw 超差，节点立即发布
禁用零速命令并报告停止/错误，不会在到点后前进或倒车微调。对应参数为
`terminal_alignment_distance` 和 `terminal_yaw_tolerance`。

## 编译

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select can_client lawn_mower_tracker
source install/setup.bash
```

## 启动

默认 launch 同时启动底盘驱动和跟踪节点：

```bash
ros2 launch lawn_mower_tracker tracker.launch.py
```

使用自定义参数：

```bash
ros2 launch lawn_mower_tracker tracker.launch.py params_file:=/absolute/path/tracker.yaml
```

## 任务控制

```bash
ros2 service call /mowerTrackControl lawn_mower_tracker/srv/MowerTrackControl \
  "{track_state: start, map_name: map_001, line_id: 1}"

ros2 service call /mowerTrackControl lawn_mower_tracker/srv/MowerTrackControl \
  "{track_state: pause, map_name: '', line_id: 1}"

ros2 service call /mowerTrackControl lawn_mower_tracker/srv/MowerTrackControl \
  "{track_state: resume, map_name: '', line_id: 1}"

ros2 service call /mowerTrackControl lawn_mower_tracker/srv/MowerTrackControl \
  "{track_state: stop, map_name: '', line_id: 1}"
```

状态值：`0=idle`、`3=tracking`、`4=paused`、`5=stopped_or_error`、`6=completed`。

## 测试

```bash
colcon test --packages-select can_client lawn_mower_tracker
colcon test-result --verbose
```

上车前应架空车轮或隔离驱动动力，逐项验证 RTK 断流、pause、stop、终点和进程退出均能使 `/vehicle_cmd` 变为禁用零速命令。

## 真车雷达自动避障部署

真车与模型的车体、轴距和安装位置相同，直接沿用其几何。真车原始点云
`rslidar` 与仿真 `fairy_optical` 的坐标原点不同，使用对应配置即可，无需转换点云。

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select can_client lawn_mower_tracker --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
ros2 launch lawn_mower_tracker avoidance.launch.py \
  can_iface:=can0 path_file:=/absolute/path/to/lawn_mower_records.csv
```

机器上还需要已构建的 `rslidar_sdk`、`rslidar_msg` 和运行中的 RTK 定位节点。
入口读取现有 `/gnss_pose`，也可指定 `odom_topic:=/your/localization/pose`；
路径 CSV 必须与该位姿使用相同的平面坐标原点。不会启动 Gazebo 或使用仿真时钟。
启动节点时不自动开始任务；路径和雷达有效后，再通过现有 `start` 服务开始。

```bash
ros2 service call /mowerTrackControl lawn_mower_tracker/srv/MowerTrackControl \
  "{track_state: start, map_name: '', line_id: 1}"
```

如果真实 CAN 或雷达节点已运行，可设置 `with_can:=false` / `with_lidar:=false`，
防止重复启动。自定义参数使用 `params_file:=/absolute/tracker.yaml`，
雷达配置使用 `driver_config:=/absolute/driver.yaml`。

| 项目 | 真车初始配置 |
|---|---|
| 参数文件 | `config/tracker_real_avoidance.yaml` |
| 点云输入 / frame | `/rslidar_points` / `rslidar` |
| 点云原点相对 base_link 高度 | `lidar_origin_height: 0.483976069` m |
| 轴距 | 0.36 m |
| 限速 / 安全余量 | 0.15 m/s / 0.20 m |
| 雷达断流 / CAN 指令断流 | 0.30 s / 0.25 s |
| 制动减速度假设 | `avoidance_braking_deceleration: 0.30` m/s² |

`lidar_origin_height` 是点云坐标原点的安装高度。真车驱动解码本身已经处理内部
镜心偏移；不要再加仿真专用的 0.035 m。旧 `lidar_optical_height` 参数仍兼容，
新参数存在时优先使用。随包的 `fairy_avoidance_driver.yaml` 使用主机时间戳，
避免未同步雷达时钟导致点云被判过期。若使用其他驱动，需保证 frame、时间和点坐标含义一致。

CAN 驱动在收到新命令后的 0.25 s 内重发有效命令；超过期限持续发送禁用零速。
这个超时用单调时钟计算，不受 ROS 时间回拨影响。无效速度字段也会立即禁用。
因此跟踪节点退出、卡住或不再发布时，不会无限重发最后一条运动指令。
退出底盘节点时也实际下发禁用帧。

RViz 可将 Fixed Frame 设为 `rslidar`，查看 `/avoidance/filtered_points` 和
`/avoidance/local_path`，不需要为了避障额外发布 TF。

软件验证覆盖原始 `rslidar` 输入、仿真原点与真车原点的过滤一致性、断流停车和
CAN 看门狗。尚未实际启动本机 CAN 或雷达，未做真车道路验收；初次部署应先隔离
驱动动力，核对车体过滤与零速保护，再以此低速参数测试绕障。制动减速度需由实车
测得；若达不到配置值，应降低该参数或速度、扩大安全余量。固定高度过滤适用于
近似平地，坡面地面分割、悬崖和动态障碍预测仍不在当前实现范围。

## 仿真雷达自动避障

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select lawn_mower_tracker fw_mini_pro_sim --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
ros2 launch fw_mini_pro_sim avoidance.launch.py
# 无界面运行：追加 gui:=false
```

在另一个终端启动任务（先 source 同一工作区）：

```bash
ros2 service call /mowerTrackControl lawn_mower_tracker/srv/MowerTrackControl \
  "{track_state: start, map_name: '', line_id: 1}"
```

默认场景在原路线 x=1.8 m 处放置箱体，目标为 x=4.2 m。车辆自动搜索左右通道，
沿自行车运动学允许的转向路线绕行并回归原路线；不主动切换前进/倒车方向。
`path_file:=/absolute/path.csv` 可更换任务，`world:=/absolute/world.sdf` 可更换场景。
此入口只启动仿真适配器，不加载 CAN 硬件驱动。

点云直接使用 `/rslidar_points` 的 `fairy_optical` 坐标，x/y 与车体水平轴重合，
光心高于 base_link 0.518976069 m。无需 TF 查询或逐点坐标转换。
车体过滤盒包括车身、轮胎、雷达与支架；高度过滤按 base_link 高度解释，
默认保留 -0.04 至 0.55 m 的障碍点，地面在约 -0.07 m。
仅将少量路线目标和车辆运动表达在最近一帧雷达坐标中。

参数位于 `fw_mini_pro_sim/config/avoidance_sim.yaml`。默认限速 0.15 m/s、
雷达超时 0.5 s、安全余量 0.12 m、局部搜索半径 5 m、前方目标距离 3.6 m。
整车外接圆约 0.355 m，碰撞检查在其外增加安全余量与轨迹采样补偿；
狭窄通道可能保守停车。速度与超时不匹配安全余量时启动会报错，
制动余量按仿真假设 0.5 m/s² 计算。每帧雷达触发一次路线搜索，控制周期内继续检查短程运动。

RViz 设置 Fixed Frame 为 `map`，添加：

- PointCloud2 `/avoidance/filtered_points`：过滤后的障碍点。
- Path `/avoidance/local_path`：当前选择的绕行路线，停车时清空。

无安全路线、雷达缺失/过期、frame 不符、空点云或字段损坏时发布禁用零速。
任务保持跟踪状态，恢复有效观测且路线可行后自动重试；pause/stop 仍优先。
点云支持小端 FLOAT32 的 x/y/z，含行填充的组织点云也可读取。

该实现针对平地仿真中当前雷达可见的障碍；不处理悬崖、遮挡区域的未知障碍、
动态障碍预测或需要自行倒车脱困的复杂死路。高度带以外的障碍不会参与规划。

自动实跑验收（会自行启动并关闭隔离的无界面仿真）：

```bash
python3 src/fw_mini_pro_sim/scripts/avoidance_smoke_test.py
```
