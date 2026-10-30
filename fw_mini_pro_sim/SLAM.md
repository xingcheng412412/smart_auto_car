# Fairy 雷达与二维 SLAM

先启动仿真，再驱动车辆；本入口不会自动启动 map_01 跟踪器。

## 启动

在第一个终端：

```bash
cd /home/sun/all_can_client/fw_mini_pro_control_ros2
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=88 ROS_LOCALHOST_ONLY=1 IGN_PARTITION=fw_fairy
ros2 launch fw_mini_pro_sim slam.launch.py
```

打开 Gazebo 和 RViz。`gui:=false rviz:=false` 可关闭两个窗口，但雷达仍需要 GPU/软件渲染支持。
不要与其他仿真使用同一个通信域和 Gazebo 分区。关闭整套程序请在 launch 终端 Ctrl+C；只关窗口不会关闭后台。

第二个终端运行同样的 cd、source 和 export 命令，再执行：

```bash
python3 src/fw_mini_pro_sim/scripts/slam_smoke_test.py --output /tmp/fairy_slam_run
```

这会让车辆绕一个约 1.17 m 半径的小圈，并自动停止，检查点云、地图更新、实际平移距离和返回误差。只在本文件的独立建图场景使用此测试路线。

也可手动低速行驶：

```bash
ros2 topic pub -r 20 /vehicle_cmd can_client/msg/VehicleCmd \
  '{enable: true, gear: 6, linear_x: 0.2, angular_z: 0.0}'
```

Ctrl+C 停止发送后，0.5 秒指令超时会下发零轮速；有物理制动过程。转弯可设置 angular_z: 0.2。此入口无自动避障，不要持续驶向墙体。

## 保存地图

使用相同 ROS 环境及通信域：

```bash
mkdir -p maps
ros2 run nav2_map_server map_saver_cli -f "$PWD/maps/fairy_map" \
  --ros-args -p save_map_timeout:=10.0
```

生成 PGM 地图和 YAML 元数据。已验证的示例在 `validation/fairy/`。这是二维占据地图，不是三维点云地图。

## 安装与数据

- 原车身 STL 顶面相对 base_link 高 0.183976069 m；支架高 0.3 m；雷达底座 (0,0,0.483976069) m。
- 雷达近似圆柱直径 0.075 m、高 0.07 m，底座为 rslidar 坐标原点。
- 光学原点 fairy_optical 暂估在底座上方 0.035 m；不是实测标定。点云 frame_id 为 fairy_optical，TF 转到 rslidar 时会正确包含该偏移。
- 规则 GPU 射线：96×1440，水平 360°、垂直 ±16°，量程 0.1–150 m，目标 10 Hz。并非 Fairy UDP、发射时序、强度或多回波仿真。
- `/rslidar_points` 为 PointCloud2；取光学原点上下 0.06 m 切片转换成 `/scan`，建图使用距离上限 20 m。
- `/map` 为 slam_toolbox 输出。TF 链为 map→odom→base_link→rslidar→fairy_optical。
- SLAM 模式关闭旧的固定 map→odom，由 SLAM 发布；Gazebo 理想里程计提供 odom→base_link。真值另发布到 `/sim/ground_truth_pose`，坐标系为 odom。此入口不把真值作为 SLAM 定位发布给跟踪器。
- 未启用实车 rslidar_sdk 或 IMU 融合。后续接实车仍需补齐支持 Fairy 的驱动和时间同步。

## 验收结果（2026-09-12）

最终动态验收通过：累计平移 7.420 m，最大离起点 2.353 m，返回距离误差 0.075 m，运动期间 30 个不同地图版本。地图 237×237 栅格，分辨率 0.05 m，包含约 1016 个占据栅格与 50621 个自由栅格。此返回误差使用 Gazebo 里程计计算，不是 SLAM 定位精度指标，也未单独证明触发了算法回环约束。

完整 96×1440 点云在本机接收频率：无 RViz 首轮约 5.56 Hz，软件渲染 RViz 开启时约 4.49 Hz；最终实测仿真/实际时间比约 0.77。配置的 10 Hz 并未实际达到。若需更快速度，可降低生成脚本中的水平 samples 后重新生成、构建；需要记录为降采样模型。

21 项 Python 回归通过，模型和世界 SDF 验证通过；原跟踪核心另行回归。RViz 的点云和地图已通过用户提供截图核对。首次地图出现时本机记录一次 indexed_8bit_image 着色器告警，但地图实际可见；上游也记录过相同首次显示告警：https://github.com/ros2/rviz/issues/463 。

## 修改与构建

修改 `scripts/generate_fairy_model.py` 后执行：

```bash
python3 src/fw_mini_pro_sim/scripts/generate_fairy_model.py
colcon build --packages-select fw_mini_pro_sim
source install/setup.bash
```

基础车模型更新后也需要重新生成带雷达版本。普通 `simulation.launch.py` 保持原有无雷达场景。
