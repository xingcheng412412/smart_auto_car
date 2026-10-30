# 验证记录（2026-09-08）

环境：Ubuntu 22.04，ROS 2 Humble，Gazebo Fortress 6.18.0，ros_gz 0.244.25。

## 安装和构建

- Gazebo/ROS 桥接包已安装，`ign gazebo --versions` 输出 `6.18.0`。
- 软件源旧索引引起的依赖 404 已通过 apt-get update 解决。
- `colcon build --packages-up-to fw_mini_pro_sim --cmake-args -DAMENT_CMAKE_SYMLINK_INSTALL=OFF`：3 个包通过。
- 使用普通安装与工作区原有构建方式一致；不切换为 symlink 安装，以免旧生成消息目录冲突。
- `ign sdf -k models/fw_mini_pro/model.sdf`：Valid。
- 设置 `SDF_PATH` 指向本包 models 后验证世界 SDF：Valid。
- `ros2 launch fw_mini_pro_sim simulation.launch.py --show-args`：通过。

## 自动回归

`colcon test --packages-select lawn_mower_tracker fw_mini_pro_sim` 全部通过。
`colcon test-result` 汇总跟踪器 72 条测试记录、仿真包 18 条，0 错误、0 失败、0 跳过。
其中跟踪核心 GTest 为 38 项，包括两项新加的倒车终点回归。

## Gazebo 物理实测

`python3 scripts/smoke_test.py` 通过。位移由 Gazebo 实测里程计得到，非指令积分。

| 指令（持续 4 秒） | 实测结果 |
|---|---|
| 前进 0.3 m/s | 前进 1.128 m |
| 倒车 -0.3 m/s | 倒退 1.126 m |
| 前进 0.25 m/s，转弯 0.3 rad/s | 转角 1.022 rad，前向位移 0.782 m |
| 左横移 0.25 m/s | 左移 0.814 m，前向漂移 -0.015 m |
| 原地旋转 0.4 rad/s | 转角 0.711 rad，平移约 0.00023 m |

每段停止发送指令 1.5 秒后，实测线速度和角速度均为零。
上述结果包含转轮等待、加减速、接触摩擦等初始仿真参数的影响，不表示真实电机响应已辨识。
完整数值：`validation/motion_results.json`。

## 现有跟踪器闭环

`python3 scripts/smoke_test.py --tracker` 通过；三条路径均报告 `6=completed`。

| 轨迹 | 终点距离误差 | 终点朝向误差 |
|---|---:|---:|
| 直线 3 m | 0.1643 m | 0.000003 rad |
| 倒车 3 m | 0.1673 m | 0.000022 rad |
| 左转 90° | 0.1708 m | 0.0664 rad（3.81°） |

均在当前 0.2 m / 10° 完成阈值内。直线任务同时测试了 pause 停车和 resume 恢复。
实测轨迹：`validation/tracking.png`；完整结果和轨迹点保存在同目录 JSON 文件。

## 仿真发现并修复的跟踪器错误

原算法把倒车路径方向与终点车体 yaw 直接插值。在直退路径的 x=-2.5 m、yaw=0 处，
即便目标为 (-3,0,yaw=0)，仍产生 0.926233 rad 转向，导致真实 Gazebo 倒车终点验收失败。

经用户批准：在转向计算中先确定行驶方向，倒车时将终点车体 yaw 转为 yaw+π 的行进方向，
再与前视方向混合。终点完成判定仍使用原始车体 yaw。两项回归测试修复前均失败、修复后通过，
并通过独立代码审查和 Gazebo 复测。实车配置中的原始轴距及其他参数未修改。

## map_01 完整路径补充验收

2026-09-08 修复换向时新行驶方向与旧路径进度不一致导致的中途错误停车。
新增三项回归后，跟踪核心 41 项通过；colcon 汇总跟踪器 75 条、仿真包 18 条测试记录，均无失败、错误或跳过。
完整 `map_01/simulation_local.csv`（986 点，122.761 m）Gazebo 闭环运行通过，报告状态 6，终点距离误差 0.115992 m、朝向误差 0.076765 rad。
实测结果和轨迹保存在工作区 `map_01/validation/`，复验命令见 `map_01/SIMULATION.md`。

## 图形显示

已实际启动 Gazebo GUI，确认厂家底盘网格、车轮、地面和实体树正常显示。
截图：`validation/gazebo.png`。测试窗口和所有验收进程已关闭。

本验收针对平地、理想位姿和估计动力学参数，不替代实车上的电机/轮胎/RTK/CAN 验证。

## Fairy 雷达与 SLAM（2026-09-12）

独立建图入口、带雷达模型、障碍物世界已完成。21 项 Python 回归和 41 项跟踪核心测试通过。动态路线 7.420 m、返回误差 0.075 m；地图保存成功。具体限制、频率与复验方式见 [SLAM.md](SLAM.md)，地图及指标见 `validation/fairy/`。

## RTK simulation — 2026-09-19

- Built `rtk_gps_driver_cpp` with `RTK_INTERFACES_ONLY=ON`, and `fw_mini_pro_sim` successfully. Full serial-driver build unavailable here because GeographicLib is not installed.
- Python regression: 31 passed, including WGS84 NWU conversion, deterministic noise, sampling, scheduled dropout, time reset, message outputs and invalid attitude/source-timeout handling.
- Isolated live ROS transport probe passed truth→RTK pose, SetBool dropout and recovery.
- Gazebo `smoke_test.py --tracker --path straight --rtk` passed with default noise. Tracker status 6; ground-truth goal distance 0.15680 m, yaw error 0.01205 rad; pause/resume passed. Log directory `/tmp/fw_sim_acceptance_qkpitguk`.
- Both launch argument descriptions load. SLAM+RTK physical run and dropout-triggered physical tracker stop have not been dynamically validated in this change. RTK is a simplified output-level model, not a hardware accuracy claim.

## Online SLAM + RTK fusion — 2026-09-19

- Python regression: 39 passed; includes SE(2) math, covariance correction, yaw wrap, outlier rejection, delayed measurement transport, initial frame alignment, RTK outage/recovery, dual-source loss, clock reset and jointly stale odometry/measurements.
- Built fw_mini_pro_sim and loaded fusion launch parameters successfully.
- `fusion_smoke_test.py`: PASS online mapping, FUSED→SLAM_ONLY→FUSED→LOCALIZATION_LOST, 849 fused poses, 22 map messages, max adjacent position step 0.02835 m; control pose publication stops on dual-source loss. Observed map→odom and verified SLAM transform_publish_period=0. Results `/tmp/fusion_acceptance_v6mrx9cf/results.json`.
- `fusion_smoke_test.py --tracker-stop`: PASS vehicle moving under fused-pose tracker before fault, both absolute sources disabled, resulting command enable=false and measured Gazebo speed=0.0 m/s; 309 poses, 10 map messages, maximum adjacent position step 0.02388 m. Results `/tmp/fusion_acceptance_k6q_w_g5/results.json`.
- Initial test harness attempt failed because this installed Humble rclpy uses single-argument subscription callbacks; corrected harness and reran. No localization outcome claimed from the failed run.
- No long-duration drift, deliberate SLAM loop-closure jump, noisy odometry, map deformation correction or hardware accuracy validation. Synthetic ideal odometry remains in use. TF continues for mapping during loss; control validity is governed by gated pose output and status.


## 2026-10-26 雷达自动避障

入口：`ros2 launch fw_mini_pro_sim avoidance.launch.py`。
复现：`python3 src/fw_mini_pro_sim/scripts/avoidance_smoke_test.py`，脚本隔离 ROS domain
与 Gazebo partition，自动启动任务、检查测量轨迹和完成状态，最后清理进程。

箱体中心为 (1.8, 0)，尺寸 0.5 × 0.6 × 0.7 m；参考路线终点为 (4.2, 0)。
实跑成功绕障并完成任务，最大横向偏移 0.873 m，车体中心到箱体边界最小距离
0.516 m。最终位置误差 0.134 m，yaw 误差 4.62°。
记录见 `validation/avoidance_acceptance.json`；该距离为车体中心到箱体边界，
不是车身表面间隙。外接圆半径约 0.355 m。

回归：tracker 的 colcon 报告 92 项、sim 的 pytest 39 项，均为零错误、零失败。
测试涵盖点云过滤、左右绕行、封堵、倒车、闭环回归与终点、雷达缺失/超时、
错误 frame、负时间戳停车及恢复。A* 有单次 30 ms 搜索预算及多目标 60 ms 检查点；
它们不包含全部建桶与回调开销，不是严格实时上限。发布驱动前再次检查 RTK 和点云时效。
