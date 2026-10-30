# lawn_mower_recorder（ROS 2 Humble）

`lawn_mower_recorder` 记录割草机的 GNSS 位姿与经纬度数据，输出可供
`lawn_mower_tracker` 加载的 CSV 路径文件。

## 接口

| 类型 | 名称 | 消息类型 |
|---|---|---|
| 订阅 | `/gnss_pose` | `geometry_msgs/msg/PoseStamped` |
| 订阅 | `/fix` | `sensor_msgs/msg/NavSatFix` |
| 发布 | `/record_status` | `lawn_mower_recorder/msg/RecordStatus` |
| 服务 | `/process_switch` | `lawn_mower_recorder/srv/ProcessSwitch` |
| 服务 | `/record_mark_point` | `lawn_mower_recorder/srv/RecordMarkPoint` |

当前 CAN 反馈消息不提供刀片状态，recorder 不订阅 CAN 反馈，CSV 也不包含
刀片状态列。

## 参数

| 参数 | 默认值 | 说明 |
|---|---|---|
| `base_dir` | `~/fw_mini_pro_control_ros2/recorded_files` | 录制文件根目录 |
| `pose_topic` | `/gnss_pose` | GNSS 位姿话题 |
| `fix_topic` | `/fix` | GNSS 经纬度话题 |
| `record_status_topic` | `/record_status` | 录制状态话题 |
| `record_period` | `0.1` | 录制定时器周期，单位秒 |
| `status_period` | `1.0` | 状态发布周期，单位秒 |

## 编译

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select can_client lawn_mower_recorder
source install/setup.bash
```

## 启动

```bash
ros2 launch lawn_mower_recorder data_record.launch.py
```

指定输出目录：

```bash
ros2 launch lawn_mower_recorder data_record.launch.py \
  base_dir:=/absolute/path/to/recorded_files
```

## 开始和停止录制

开始录制会创建或覆盖根目录中的 `lawn_mower_records.csv`：

```bash
ros2 service call /process_switch lawn_mower_recorder/srv/ProcessSwitch \
  "{data: true, map_name: '', record_status: ''}"
```

停止时可以提供地图名，文件会移动到 `base_dir/<map_name>/`：

```bash
ros2 service call /process_switch lawn_mower_recorder/srv/ProcessSwitch \
  "{data: false, map_name: 'map_001', record_status: ''}"
```

`map_name` 只能是一个目录名称，不能包含 `/`、绝对路径、`.` 或 `..`。
传入空地图名时仅停止并关闭文件，不移动文件。

## 记录标记点

录制至少一个点后，可以把最新记录追加到 `mark_points.csv`：

```bash
ros2 service call /record_mark_point lawn_mower_recorder/srv/RecordMarkPoint \
  "{mark_id: 1}"
```

未录制或尚无有效记录时，服务返回 `success: false`。

## CSV 格式

普通记录表头：

```text
id,path_id,timestamp,pose_x,pose_y,pose_z,yaw,pitch,roll,linear_velocity,linear_velocity_direction,angular_velocity,segment_type,distance_from_prev,cumulative_distance,latitude,longitude
```

标记点文件使用相同字段，并在最后增加 `point_mark`。

首次有效采样只初始化运动状态。节点按 0.1 秒周期记录后续有效输入，
累计行驶距离每超过 10 米递增一次 `path_id`。

## 测试

```bash
source /opt/ros/humble/setup.bash
colcon test --packages-select lawn_mower_recorder
colcon test-result --verbose
```
