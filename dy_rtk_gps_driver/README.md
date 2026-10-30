# rtk_gps_driver

ROS Catkin 功能包 —— RTK GPS 驱动节点（C++ 版本），支持单串口双向通信，集成 NTRIP 客户端与 NMEA 解析器。

## 功能概述

`rtk_gps_driver` 为 RTK GPS 接收机（CHCNAV 组合 GNSS+IMU 设备）提供 ROS 驱动，通过单串口实现双向通信：

- **串口上行**：接收 GPS 接收机输出的 GGA 与 GPCHCX 语句，解析后发布为 ROS 标准消息。
- **串口下行**：将从 NTRIP 服务端获取的 RTCM 差分改正数据转发至 GPS 接收机，实现 RTK 固定解定位。
- **NTRIP 闭环**：定期向 NTRIP 服务端回报 GGA 位置信息，形成完整差分闭环。
- **位姿输出**：内置经纬高到局部坐标转换、状态门槛、卡尔曼滤波与跳点抑制，以 50Hz 发布位姿和 TF。

## 目录结构

```
rtk_gps_driver/
├── CMakeLists.txt                        # Catkin 构建配置
├── package.xml                           # ROS 包描述与依赖声明
├── msg/
│   └── RTKGPSLog.msg                     # 自定义诊断日志消息
├── launch/
│   └── rtk_gps_cpp.launch                # 节点启动文件
├── include/rtk_gps_driver_cpp/
│   ├── rtk_gps_driver_node.h             # 主节点类头文件
│   ├── nmea_parser.h                     # NMEA 语句解析器头文件
│   ├── pose_localizer.h                   # 位姿转换、状态门槛与滤波
│   ├── ntrip_client.h                    # NTRIP 客户端头文件
│   └── serial_port.h                     # 串口 I/O 头文件
└── src/
    ├── rtk_gps_driver_node.cpp           # 主节点实现 + 入口 main()
    ├── nmea_parser.cpp                   # NMEA 解析器实现
    ├── pose_localizer.cpp                 # 局部坐标位姿实现
    ├── ntrip_client.cpp                  # NTRIP 客户端实现
    └── serial_port.cpp                   # 串口通信实现
```

## 依赖

### ROS 依赖

| 依赖包                                   | 说明                              |
| ---------------------------------------- | --------------------------------- |
| `roscpp`                                 | ROS C++ 标准库                    |
| `std_msgs` / `std_srvs`                  | 标准 ROS 消息与服务               |
| `sensor_msgs`                            | 传感器标准消息（NavSatFix / Imu） |
| `nmea_msgs`                              | NMEA 语句消息                     |
| `geometry_msgs`                          | 几何消息                          |
| `tf`                                     | ROS 坐标变换                      |
| `message_generation` / `message_runtime` | 自定义消息生成与运行时            |

### 第三方库

| 库            | 说明                       |
| ------------- | -------------------------- |
| Boost.Asio    | 串口异步 I/O               |
| Boost.System  | Boost 系统错误处理         |
| GeographicLib | 经纬高到局部 ENU 坐标转换  |
| `termios`     | 串口配置（波特率、8N1 等） |

## 构建与运行

### 编译

在工作空间根目录下执行：

```bash
catkin_make
# 或
catkin build
```

### 硬件联调

先将模板复制到 `mower_start` 并填写 NTRIP 凭据，再以实际串口启动：

```bash
roslaunch rtk_gps_driver_cpp rtk_gps_cpp.launch serial_port:=/dev/ttyUSB0
```

另开终端检查话题和热复位服务：

```bash
rostopic hz /fix
rostopic hz /imu_raw
rostopic hz /velocity_enu
rostopic hz /gnss_pose
rosrun tf tf_echo map gps
rostopic echo -n 1 /rtk_log
```

确认 `rtk_log` 每 0.5 秒输出，`ntrip_failure_category`、失败次数和下次重连时间在断网或错误凭据时可见。

### 启动节点

```bash
roslaunch rtk_gps_driver_cpp rtk_gps_cpp.launch
```

第三方 CORS 参数从 `mower_start/param/rtk_gps.yaml` 加载。可从本包模板复制：

```bash
cp $(rospack find rtk_gps_driver_cpp)/param/rtk_gps.yaml.template \
  $(rospack find mower_start)/param/rtk_gps.yaml
```

模板 [rtk_gps.yaml.template](param/rtk_gps.yaml.template) 只包含 `ntrip_ip`、`ntrip_port`、`ntrip_account`、`ntrip_password` 与 `mountPoint`。填写服务器和账号信息后即可使用；勿将真实凭据提交到仓库。

启动参数（可在 launch 文件中修改）：

| 参数                       | 默认值                                  | 说明                                     |
| -------------------------- | --------------------------------------- | ---------------------------------------- |
| `serial_port`              | `/dev/ttyS0`                            | 串口设备路径                             |
| `baudrate`                 | `460800`                                | 串口波特率                               |
| `frame_id`                 | `gps`                                   | TF 坐标帧 ID                             |
| `log_interval`             | `0.5`                                   | 状态日志发布间隔（秒）                   |
| `network_check_interval`   | `0.5`                                   | 网络健康检查间隔（秒）                   |
| `rtcm_timeout_threshold`   | `3.0`                                   | RTCM 超时阈值（秒）                      |
| `gnss_config_file`         | `mower_start/param/gnss_init_pose.yaml` | 位姿原点参数文件，根节点包含 `lat/lon/h` |
| `pose_publish_rate`        | `50.0`                                  | 位姿与 TF 发布频率（Hz）                 |
| `pose_timeout`             | `0.5`                                   | 无有效 GPCHCX 后停止位姿的时间（秒）     |
| `map_frame` / `pose_frame` | `map` / `gps`                           | 位姿父坐标系与 TF 子坐标系               |

## ROS 话题与服务

### 发布话题

| 话题            | 消息类型                                   | 说明                                                             |
| --------------- | ------------------------------------------ | ---------------------------------------------------------------- |
| `fix`           | `sensor_msgs/NavSatFix`                    | GPCHCX 定位数据（经纬度、高度与位置协方差）                      |
| `imu_raw`       | `sensor_msgs/Imu`                          | GPCHCX IMU 数据（姿态、角速度、线加速度与姿态协方差）            |
| `velocity_enu`  | `geometry_msgs/TwistWithCovarianceStamped` | GPCHCX 速度；`linear.x/y/z` 分别为东/北/天向速度（m/s）          |
| `nmea_sentence` | `nmea_msgs/Sentence`                       | 透传校验和正确的原始 `$GPGGA` 与 `$GPCHCX`；不发布其他 NMEA 语句 |
| `rtk_log`       | `RTKGPSLog`                                | 驱动诊断日志（连接状态、定位质量、RTCM 统计等）                  |
| `gnss_pose`     | `geometry_msgs/PoseStamped`                | `x=北、y=-东、z=天` 的滤波位姿；满足状态门槛时以 50Hz 发布       |

### 服务

| 服务                | 类型               | 说明                                                                                   |
| ------------------- | ------------------ | -------------------------------------------------------------------------------------- |
| `initial_gnss_pose` | `std_srvs/SetBool` | 请求为 `true` 时，将最新有效 GPCHCX 经纬高持久化为局部坐标原点，并重新执行五秒发布门槛 |

初始化当前位置为新的位姿原点：

```bash
rosservice call /initial_gnss_pose "data: true"
```

服务只接受最近 0.5 秒内、`chc_status == 42`、数值有效
的最新一条 GPCHCX。成功时会原子更新
`gnss_config_file` 根节点的 `lat/lon/h`，同步更新节点参数并清空旧滤波状态；
`/gnss_pose` 和 TF 在新状态连续保持五秒后恢复。无数据、数据过期、状态错误、
或文件写入失败时返回明确的失败类别，不修改当前原点或运行状态。
失败类别包括 `REQUEST_DISABLED`、`NO_GNSS_DATA`、`STALE_GNSS_DATA`、
`STATUS_NOT_FIXED`、`INVALID_GNSS_DATA`、
`LOCALIZER_INIT_FAILED` 和 `CONFIG_WRITE_FAILED`。

## 自定义消息

### RTKGPSLog.msg

综合诊断日志消息，包含以下字段组：

**连接状态：**

| 字段                              | 说明                                                                                                             |
| --------------------------------- | ---------------------------------------------------------------------------------------------------------------- |
| `log_level`                       | 日志级别：INFO / WARN / ERROR                                                                                    |
| `connection_phase`                | 连接阶段：RUNNING / DISCONNECTED / CONNECTING / AUTH_FAILED                                                      |
| `sock_connected`                  | Socket 连接状态                                                                                                  |
| `network_status`                  | 网络状态：CONNECTING / WAITING_GGA / NORMAL / DISCONNECTED / TIMEOUT / AUTH_FAILED / SERIAL_ERROR / CONFIG_ERROR |
| `ntrip_consecutive_failure_count` | 连续连接失败次数；连接成功后清零                                                                                 |
| `ntrip_next_reconnect_delay`      | 下次重连等待时间（秒）                                                                                           |
| `ntrip_failure_category`          | `NETWORK`、`SERIAL`、`AUTHORIZATION` 或 `CONFIGURATION`                                                          |
| `ntrip_sdk_status_code`           | 为兼容旧消息保留；直连 NTRIP 模式恒为 `0`                                                                        |

**GPS 定位质量：**

| 字段                                  | 说明                                                      |
| ------------------------------------- | --------------------------------------------------------- |
| `fix_quality`                         | 定位类型：0=无定位, 1=GPS, 2=DGPS, 4=RTK 固定, 5=RTK 浮点 |
| `num_satellites`                      | 卫星数                                                    |
| `hdop`                                | HDOP 值                                                   |
| `latitude` / `longitude` / `altitude` | 经纬高（度 / 米）                                         |

**GPCHCX 组合导航状态：**

| 字段               | 说明                                             |
| ------------------ | ------------------------------------------------ |
| `chc_status`       | 字段 21 原始状态：十位为卫星状态、个位为系统状态 |
| `differential_age` | 字段 22 差分延时（秒）                           |
| `warning_flags`    | 字段 23 告警位掩码；例如 `0102` 解析为 `0x0102`  |

**RTCM 数据统计：**

| 字段                   | 说明                       |
| ---------------------- | -------------------------- |
| `rtcm_receive_freq`    | RTCM 接收频率（包/秒）     |
| `time_since_last_rtcm` | 距上次 RTCM 接收时间（秒） |
| `network_latency`      | 网络延迟（ms）             |
| `connection_duration`  | 连接持续时间（秒）         |

**RTK 固定解统计：**

| 字段                     | 说明                                            |
| ------------------------ | ----------------------------------------------- |
| `non_rtk_fixed_count`    | 非 RTK 固定解次数累计                           |
| `non_rtk_fixed_duration` | 当前非 RTK 固定解持续时间（秒），RTK 固定时为 0 |

**GGA 上报诊断：**

| 字段               | 说明                    |
| ------------------ | ----------------------- |
| `gga_send_time_ms` | 最近 GGA 发送耗时（ms） |

**错误信息：**

| 字段                | 说明                                                     |
| ------------------- | -------------------------------------------------------- |
| `error_type`        | 错误类型：TIMEOUT / SOCKET / AUTH_ERROR / PARSE_ERROR 等 |
| `error_description` | 错误描述                                                 |

## 核心类说明

### RTKGPSDriverNode

主节点类，负责整体协调。以 100Hz 主循环读取串口数据、解析 NMEA 语句、定期发布状态日志。

- `run()` — 主循环（读取串口 → 分类 NMEA → 解析 → 发布）
- `publish_status_log()` — 按 `log_interval` 周期发布 RTKGPSLog 诊断消息
- `initial_gnss_pose_callback()` — 校验最新 GPCHCX、持久化原点并重置位姿门槛

### NmeaParser

NMEA 语句解析器，基于回调模式输出解析结果：

- `parse()` — 按语句类型分发至对应解析函数
- `parse_gga()` — 校验并解析 GPGGA，保留原始语句作为直连 NTRIP 的位置上报输入
- `parse_gpchcx()` — 解析 GPCHCX 组合导航语句，输出定位、IMU、ENU 速度与协方差
- `nmea_to_decimal()` — NMEA 经纬度格式 → 十进制度转换

### NtripClient

基于 POSIX TCP Socket 的直连 NTRIP 客户端：

- 后台单线程负责 DNS、TCP 建连、HTTP Basic 认证、RTCM 接收和重连，不阻塞 ROS 主循环，也避免接收线程与重连线程竞争 Socket。
- 接受 `ICY 200` 与 HTTP `200` 响应；响应头后同一数据包内的 RTCM 字节不会丢失。
- RTCM 保持字节不变地交给串口可靠写入；串口拒绝写入时标记为 `SERIAL` 并触发重连。
- 仅校验和正确的原始 `$GPGGA` 用于位置上报，不重建或篡改质量、卫星数、时间和 HDOP。最新一条会在断线期间缓存，重连成功后立即上报。
- GPGGA 上报频率由接收机输出频率决定；如需每秒一次，请将接收机配置为 1 Hz。
- RTCM 超时看门狗仅在当前连接首次成功上传 GPGGA 后启用；HTTP `401`、`403` 或挂载点拒绝以 60 秒冷却重试，NTRIP 网络断线、服务端关闭或 RTCM 超时在失败结束后固定等待 500ms 重连。单次 TCP 建连和 HTTP 握手仍各有 5 秒超时。
- `rtk_log` 发布 `CONNECTING`、`RUNNING`、`DISCONNECTED`、`AUTH_FAILED` 与 `RETRYING` 等连接阶段。

### PoseLocalizer

- 使用 `mower_start/param/gnss_init_pose.yaml` 的 `lat/lon/h` 建立 GeographicLib 局部坐标原点。
- 初始上电或配置原点后，系统要求 `fix_quality==4`（INS完全对齐）且 `gps_diff_status` 为48/49/50（GPS固定解），连续保持五秒后才开始发布 `/gnss_pose` 与 `map -> gps` TF。
- 一旦通过状态门槛，后续数据不再检查定位质量，直接进入卡尔曼滤波并发布位姿。
- 超过 0.5 秒没有被接受的新 GPCHCX 数据时停止输出，恢复后重新等待五秒。
- 所有测量数据均进入位置卡尔曼滤波，无速度限制检查。
- `/initial_gnss_pose` 使用最新有效固定解更新 `gnss_init_pose.yaml`，成功后清空滤波并重新等待五秒状态门槛。
- 姿态保持旧本地化器约定：`roll=字段5`、`pitch=-字段4`、`yaw=π-字段3`。

### SerialPort

基于 Boost.Asio 的串口通信类，带后台读取线程：

- `open()` — 配置串口（8N1，禁用 OPOST 防止二进制 RTCM 数据的 `\n` → `\r\n` 破坏）
- `drain_lines()` — 非阻塞获取最新的一条 GGA 和一条 GPCHCX
- `write_with_drain()` — 使用 POSIX `poll()` + `::write()` 写入，并以 `TIOCOUTQ` 在 1 秒内确认输出队列排空；禁用 `IXON/IXOFF`，避免接收数据暂停热复位或 RTCM 下行。

## 系统架构

```
GPS 接收机 (串口) ←→ SerialPort ←→ RTKGPSDriverNode (100Hz 主循环)
       ↑                                      ↓
  RTCM 改正数据                      NMEA 语句解析
       ↑                                      ↓
  NtripClient ──→ NTRIP 播发服务器     NavSatFix / Imu / NMEA
       ↑                                      ↓
  GGA 位置回报                ROS 标准话题 / 50Hz 位姿与 TF
```

## GPCHCX 字段与协方差映射

仅 `$GPCHCX` 用作组合导航解析；旧 `$GPCHC` 不再解析。GGA 保留为 NTRIP 上报数据源，不再发布重复的 `fix`。

| GPCHCX 字段                             | 发布位置                         | 映射                                                                                                    |
| --------------------------------------- | -------------------------------- | ------------------------------------------------------------------------------------------------------- |
| 12–14：Lat / Lon / Alt                  | `fix`                            | 位置与 `NavSatFix` 协方差                                                                               |
| 15–17：Ve / Vn / Vu                     | `velocity_enu`                   | `linear.x/y/z` = 东/北/天                                                                               |
| 24–26：Lat/Lon/Alt 标准差（m）          | `fix.position_covariance`        | 对角线 `[Lon_std², Lat_std², Alt_std²]`，ENU 顺序                                                       |
| 27–29：Ve/Vn/Vu 标准差（m/s）           | `velocity_enu.twist.covariance`  | 对角线 `[Ve_std², Vn_std², Vu_std²]`                                                                    |
| 30–32：Roll/Pitch/Heading 标准差（deg） | `imu_raw.orientation_covariance` | 先转 rad 再平方；表格中 Pitch 为 X 轴、Roll 为 Y 轴，故对角线为 `[Pitch_std², Roll_std², Heading_std²]` |
| 21：Status                              | `fix.status.status` / 位姿门槛   | 十位映射为 GGA `fix_quality` 后减 1；精确值 80 映射 `fix_quality=6`；位姿仅接受精确值 42                |
| 43：NSU1                                | `fix.status.service`             | 主天线实际使用卫星数；不使用字段19/20可见卫星数                                                         |

字段 33–42 的速度航向和安装外参以及字段 44–45 的副天线卫星数、SN 会被忽略，不影响标准话题与位姿输出。

## 设计要点

- **单串口双向通信**：GPS 接收机与 NTRIP 改正数据共用同一串口，上行接收 NMEA，下行发送 RTCM。
- **NMEA 去重**：同一周期内若收到多条相同类型的 NMEA 语句，仅解析最新一条，避免处理过期数据。
- **优雅降级**：未配置 NTRIP 时节点以纯 GPS 模式运行；认证、网络或 RTCM 超时失败时继续运行并在后台重连。
- **串口可靠性**：禁用内核 OPOST 处理防止二进制 RTCM 数据损坏；关键写入绕过 Boost.Asio 使用 POSIX 直写保证可靠送达。
- **长期运行保护**：单条串口输入最多 1024 字节；GGA 与 GPCHCX 各仅保留一条最新帧，新帧直接覆盖旧帧，避免串口积压造成定位滞后。超长异常帧会丢弃并每 5 秒节流告警一次。
- **节点自动恢复**：launch 文件中设置 `respawn="true"`；串口读取端检测到 HUP、I/O 错误或无效描述符时，节点会记录 `SERIAL_READER_STOPPED` 后退出，由 launch 自动重启。

## 注意事项

- NTRIP 配置参数从外部 YAML 文件（`mower_start/param/rtk_gps.yaml`）加载，需确保该文件存在且配置正确；请勿将含真实账号或密码的 YAML 提交到版本库。
- 串口设备路径需根据实际硬件连接调整（默认 `/dev/ttyS0`）。
- CHC 语句为 CHCNAV 接收机专有格式，包含组合 GNSS+IMU 数据（姿态角、角速度、线加速度）。
