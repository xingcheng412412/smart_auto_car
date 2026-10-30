# lawn_mower_console

通过 SSH 终端运行的 ROS 2 curses 上位机，用于控制路径录制和路径规控，
持续显示 `/rtk_log` 状态与最新 `/gnss_pose` 数据。

本程序不提供手动驾驶，不发布 `/vehicle_cmd`，也不能替代物理急停。

## 依赖与编译

需要已安装并 source 提供
`rtk_gps_driver_cpp/msg/RTKGPSLog` 的 RTK 驱动工作区。

```bash
cd ~/fw_mini_pro_control_ros2
source /opt/ros/humble/setup.bash
colcon build --packages-select \
  can_client lawn_mower_recorder lawn_mower_tracker lawn_mower_console
source install/setup.bash
```

## SSH 运行

```bash
ssh <user>@<vehicle-host>
cd ~/fw_mini_pro_control_ros2
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run lawn_mower_console lawn_mower_console
```

终端建议至少为 `80x24`。SSH 断开或按 `Q` 只退出上位机，不会自动停止
正在运行的 recorder 或 tracker。

## 界面

左侧持续滚动显示 `/rtk_log`：

- INFO 为绿色。
- WARN 为黄色。
- ERROR 为红色。
- `X` 清空本地日志窗口，不影响 RTK 驱动。

右侧持续显示 `/gnss_pose`：

- X、Y、Z。
- 四元数换算的 yaw。
- 消息年龄及在线/超时状态。

下方显示 recorder、tracker、车辆安全状态及最近服务结果。

## 按键

| 按键 | 操作 |
|---|---|
| `R` | 弹出地图名输入框并开始录制 |
| `E` | 停止录制并保存 |
| `T` | 校验目标 CSV 后开始规控 |
| `P` | 暂停规控 |
| `C` | 恢复规控 |
| `S` | 停止规控 |
| `X` | 清空 RTK 日志窗口 |
| `Q` | 退出上位机，不停止任务 |

地图名只能是一个目录名称，不能包含 `/`、`.`、`..` 或绝对路径。

规控默认检查：

```text
~/fw_mini_pro_control_ros2/recorded_files/<map_name>/lawn_mower_records.csv
```

CSV 必须包含有限数值的 `pose_x`、`pose_y`，且至少有三个点。

## 参数

```bash
ros2 run lawn_mower_console lawn_mower_console --ros-args \
  -p path_root:=~/fw_mini_pro_control_ros2/recorded_files \
  -p rtk_log_topic:=/rtk_log \
  -p max_rtk_log_lines:=500 \
  -p gnss_timeout_sec:=1.0
```

主要参数：

- `path_root`
- `path_filename`
- `pose_topic`
- `rtk_log_topic`
- `record_status_topic`
- `path_status_topic`
- `vehicle_feedback_topic`
- `record_service`
- `track_service`
- `gnss_timeout_sec`
- `service_timeout_sec`
- `max_rtk_log_lines`

## 运行约束

- 录制中不能开始规控。
- 规控或暂停状态下不能开始录制。
- 启动规控前必须收到新鲜的 `/gnss_pose`。
- 服务离线、请求超时或 CSV 错误会显示在界面中，不会退出程序。
- 急停、碰撞条和故障只做红色提示，安全动作仍由底盘及 tracker 负责。
