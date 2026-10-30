# FAIRY 实物 FAST-LIO2 三维建图

本目录是独立的三维建图功能组，包含官方 FAST-LIO2 ROS 2 算法、FAIRY 点云/IMU 适配器、体素地图累积和 PCD 保存。面向 ROS 2 Humble 与真实 RSFAIRY；不依赖 Gazebo，不向底盘发送控制指令。

## 编译

在工作区根目录执行：

```bash
source /opt/ros/humble/setup.bash
CMAKE_BUILD_PARALLEL_LEVEL=2 MAKEFLAGS=-j2 colcon build --packages-up-to fw_mini_pro_slam3d --executor sequential
source install/setup.bash
```

`fast_lio/` 已包含固定版本的官方 ROS2 源码和 ikd-Tree。设备驱动依赖位于本工作区的 `src/rslidar`、`src/rs_driver` 和 `src/rslidar_msg`。驱动必须以 `POINT_TYPE=XYZIRT`、`ENABLE_IMU_DATA_PARSE=ON` 编译（当前已配置）。首次算法编译需要 PCL、Eigen、ROS 2 的 pcl_ros/pcl_conversions/tf2_ros 等开发包；当前机器已完成编译。

## 真实设备建图

雷达启动后先保持静止几秒，使 IMU 完成初始化，再缓慢移动。电脑有线 IP 当前为 `192.168.86.15`，设备数据来源为 `192.168.86.200`；本程序不修改网卡或雷达设置。

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch fw_mini_pro_slam3d mapping.launch.py
```

默认启动驱动、适配器、FAST-LIO、地图累积节点和 RViz。无图形界面运行：

```bash
ros2 launch fw_mini_pro_slam3d mapping.launch.py rviz:=false
```

如果驱动已经单独启动，使用 `with_driver:=false`，避免争用 UDP 端口。这个选项仅适用于已输出 XYZIRT 点云和 IMU 的驱动。

保存位置可以在启动时指定（相对路径会转换成当前目录下的绝对路径）：

```bash
ros2 launch fw_mini_pro_slam3d mapping.launch.py map_file:=src/fw_mini_pro_slam3d/maps/site01.pcd
```

在另一个已 source 的终端中，**停止建图前**保存：

```bash
ros2 service call /slam3d/save_map std_srvs/srv/Trigger '{}'
```

默认保存位置为启动目录的 `maps/fairy_日期_时间.pcd`。服务响应包含实际路径及体素数量。文件先写临时文件再替换目标文件；多次保存会更新同一个文件。空地图或写盘失败会返回 `success: false`。Ctrl-C 不自动保存，必须先调用服务。

## 数据与坐标系

```text
/rslidar_points ── 点云时间适配 ── /fairy/points ┐
/rslidar_imu_data ── IMU 校验 ── /fairy/imu      ├─ FAST-LIO2 ── 位姿 / 注册点云
                                              ┘                   └─ 体素地图 / PCD
```

| 输出 | 内容 |
| --- | --- |
| `/slam3d/odometry` | 三维位姿，`camera_init` → `body`（IMU 坐标系） |
| `/slam3d/registered` | 配准到 `camera_init` 的当前点云 |
| `/slam3d/registered_body` | IMU 坐标系中的去畸变点云 |
| `/slam3d/path` | 轨迹 |
| `/slam3d/map` | 全会话体素地图，1 Hz，late-join 可获取最近一帧 |
| `/slam3d/save_map` | PCD 保存服务 |

静态 TF：`body` → `rslidar` 使用配置中的雷达到 IMU 外参，`body` → `fairy_imu` 为单位变换。没有猜测车体安装位置，也没有发布 `base_link` 的变换。后续融合车体里程计时，需要实测安装外参并明确 TF 发布责任。

点云适配将 FLOAT64 绝对 `timestamp` 转换为以消息头时间戳为基准的 FLOAT32 `time`（秒），保留 `ring`，删除无效点并按逐点时间排序。FAST-LIO 使用 `lidar_type: 2`、`timestamp_unit: 0`，这里只是选择兼容的数据格式。内置 IMU 物理量不缩放、不旋转。

保留雷达的时间基准，点云与 IMU 共同使用设备时间。本机实测设备时间不是电脑系统时间；不对每帧分别加到达时间偏移。接入其他传感器、导航或系统时间 TF 前，需要配置硬件同步或经过验证的统一时间转换。设备重启或录包回放跳时后，请重启整套建图程序。

## 配置和目录

- `fast_lio/`：官方 FAST-LIO2 ROS 2，最小接入修改见 `docs/upstream.md`。
- `bringup/`：ROS 2 包 `fw_mini_pro_slam3d`，适配器、地图保存节点和单元测试。
- `config/fairy_fastlio.yaml`：滤波、匹配、IMU 噪声与外参。
- `config/fairy_driver.yaml`：真实 FAIRY UDP 驱动参数，端口 6699/7788/6688。
- `launch/mapping.launch.py`：一键启动，可用 `--show-args` 查看参数。
- `rviz/fairy.rviz`：点云地图和轨迹显示。
- `scripts/check_mapping.py`：运行中的数据链路检查。
- `maps/`：可选地图输出目录。
- `docs/calibration.md`：本台设备实测 DIFOP 标定及单位假设。

自定义算法配置通过 `params_file:=/绝对路径/fairy_fastlio.yaml`，自定义驱动配置通过 `driver_config:=/绝对路径/fairy_driver.yaml`。修改源码里的配置后应重新编译安装，或者传入源码配置的绝对路径。

## 验证

```bash
colcon test --packages-select fw_mini_pro_slam3d --ctest-args --output-on-failure
colcon test-result --test-result-base build/fw_mini_pro_slam3d --verbose
# 在建图运行期间，从工作区根目录执行：
python3 src/fw_mini_pro_slam3d/scripts/check_mapping.py --seconds 20 --save
```

检查工具验证点云、IMU、里程计、注册点云和地图是否发布，位姿是否有限、时间是否递增，以及可选的保存服务响应。它不能代替运动建图精度验收。

## 当前边界

- FAST-LIO2 提供激光惯性里程计和增量建图，不包含回环优化、地图重定位或导航规划。
- 配置使用本台设备 DIFOP 标定的逆变换；标定平移按米解释，原厂说明未明确列出该字段单位，需确认后再做精度验收。更换设备后应重新读取标定。
- 地图分辨率为 0.15 m，最多 100 万体素。达到上限时明确报警并停止新增区域；保存服务返回不完整地图提示。可在 launch 中调整 `voxel_size` / `max_voxels`，增加容量会增加内存和发布成本。
- 地图积累沿用 FAST-LIO 的估计轨迹，长距离会有漂移；动态物体不会被专门剔除。
- 运行时必须使用同一组数据及外参；当前验证不证明移动建图精度。

本机实物验证结果见 [docs/validation.md](docs/validation.md)。雷达驱动仍存在停止时抛出 `std::system_error` 的问题，新增建图节点正常退出。
