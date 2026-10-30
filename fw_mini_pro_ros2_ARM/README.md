# can_client 使用说明（ROS2 Humble）

> FW-mini-pro 全向型智能线控底盘 CAN 通讯协议 V1.0.0（深圳煜禾森科技）| ROS2 Humble（Ubuntu 22.04）| CAN2.0B 扩展帧，波特率 500 kbit/s

本包使用自定义消息，`vehicle_core` 通过 SocketCAN 直接与底盘通信，不依赖 Autoware 消息。

## 1. 系统架构

```text
上层应用 / 上位机（upper_computer）
      │
      │ /vehicle_cmd（can_client/msg/VehicleCmd）
      ▼
  vehicle_core（can_client）
      │  内置 SocketCAN 直连
      │
      ├── 发送 ──► can0 ──► VCU（ctrl_cmd / io_cmd）
      └── 接收 ◄── can0 ◄── VCU（ctrl_fb / io_fb / bms_fb / error_fb）
                    │
                    ▼
          /vehicle_feedback（can_client/msg/VehicleFeedback）
```

`script/socketcan_bridge.py` 是可选的原始 CAN 调试工具，使用 `/can_tx` 和 `/can_rx`。`vehicle_core` 正常运行时不需要启动它。

## 2. 自定义消息

### can_client/msg/VehicleCmd

| 字段 | 类型 | 说明 |
|---|---|---|
| `enable` | bool | false 时强制切换到 disable 档 |
| `gear` | uint8 | `GEAR_DISABLE=0`、`GEAR_PARK=1`、`GEAR_NEUTRAL=2`、`GEAR_4WD4WS=6`、`GEAR_CRAB=8` |
| `linear_x` | float32 | m/s，前正后负 |
| `linear_y` | float32 | m/s，仅横移档有效，左正右负 |
| `angular_z` | float32 | rad/s，仅四转四驱档有效，逆时针为正 |

### can_client/msg/VehicleFeedback

汇总 `ctrl_fb`、`io_fb`、`bms_fb`、`bms_flag_fb` 和 `error_fb` 反馈帧。字段定义见 `msg/VehicleFeedback.msg`。

### can_client/msg/CANFrame

原始 CAN 帧，仅用于 `socketcan_bridge.py` 调试。

## 3. CAN 通信协议

详细位域定义见 `include/fw_mini_pro.hpp` 顶部注释。

### 发送帧（ADAS → VCU）

| 报文 | ID | 周期 | 说明 |
|---|---|---|---|
| ctrl_cmd | `0x18C4D1D0` | 10～50 ms | 档位、X/Y 线速度、Z 角速度、心跳、BCC |
| io_cmd | `0x18C4D7D0` | 10 ms | 灯控制、安全停车解锁、低功耗、心跳、BCC |

### 接收帧（VCU → ADAS）

| 报文 | ID | 周期 | 说明 |
|---|---|---|---|
| ctrl_fb | `0x18C4D1EF` | 10 ms | 当前档位和 X/Z/Y 反馈 |
| io_fb | `0x18C4DAEF` | 50 ms | 急停、遥控、充电、防撞条状态 |
| bms_fb | `0x18C4E1EF` | 100 ms | 电池电压、电流、剩余容量 |
| bms_flag_fb | `0x18C4E2EF` | 100 ms | 电量、保护标志、最高/最低温度 |
| error_fb | `0x18C4EAEF` | 100 ms | 故障等级、部件和故障编码 |

信号使用 Intel 小端位域。BCC 为 Byte0 到 Byte6 的异或值，存放在 Byte7。

## 4. 环境与编译

目标环境：Ubuntu 22.04、ROS2 Humble。

将本包放入 ROS2 工作空间后编译：

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
# 将本目录复制或链接为 can_client

cd ~/ros2_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select can_client
source install/setup.bash
```

每次打开新终端都需要加载环境：

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
```

## 5. 配置 SocketCAN

在源码包目录运行脚本，按提示输入 sudo 密码，即可将 `can0` 配置为 500 kbit/s 并启动：

```bash
./script/start_can.sh
```

也可以指定接口名和波特率：

```bash
./script/start_can.sh can1 500000
```

第一个参数是 CAN 接口名，第二个参数是十进制波特率。脚本会检查接口是否存在，并在完成后验证接口为 UP、CAN 控制器未停止且波特率正确。

如需手动配置，下面以 `can0` 为例：

```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 500000
sudo ip link set can0 up
ip -details link show can0
```

## 6. 运行

启动车辆驱动：

```bash
ros2 launch can_client can_client.launch.py
```

覆盖参数：

```bash
ros2 launch can_client can_client.launch.py \
  can_iface:=can0 \
  update_rate:=20.0 \
  max_linear_speed:=1.5 \
  max_angular_speed:=3.0
```

### 上位机 TUI

```bash
ros2 run can_client app.py --enable
```

也可以从源码包目录运行辅助脚本；默认启动时使能，传入 `--no-enable` 可关闭自动使能：

```bash
./upper_computer/run.sh
./upper_computer/run.sh --no-enable
```

方向键调节速度或转向，空格切换使能，G 切换四转四驱/横移档，R 请求安全停车解锁，S 停车，Q 退出。

### 原始 CAN 调试桥

```bash
ros2 run can_client socketcan_bridge.py --ros-args -p can_iface:=can0
```

## 7. 控制车辆

### 7.1 四转四驱模式

```bash
ros2 topic pub --rate 20 /vehicle_cmd can_client/msg/VehicleCmd \
  "{enable: true, gear: 6, linear_x: 0.5, linear_y: 0.0, angular_z: -0.436}"
```

`angular_z` 单位为 rad/s，`-0.436 rad/s` 约等于 `-25°/s`。

### 7.2 横移模式

```bash
ros2 topic pub --rate 20 /vehicle_cmd can_client/msg/VehicleCmd \
  "{enable: true, gear: 8, linear_x: 0.3, linear_y: -0.3, angular_z: 0.0}"
```

### 7.3 停车 / 禁用

```bash
ros2 topic pub --rate 20 /vehicle_cmd can_client/msg/VehicleCmd \
  "{enable: false, gear: 1, linear_x: 0.0, linear_y: 0.0, angular_z: 0.0}"
```

### 7.4 安全停车解锁 / 故障复位

防撞条触发安全停车后，发送一次解锁脉冲：

```bash
ros2 topic pub --once /fault_reset std_msgs/msg/UInt8 "{data: 1}"
```

### 7.5 查看反馈

```bash
ros2 topic echo /vehicle_feedback
```

## 8. 参数

| 参数 | 默认值 | 说明 |
|---|---:|---|
| `can_iface` | `can0` | SocketCAN 接口名 |
| `update_rate` | `20.0` | 主循环及 CAN 心跳频率，Hz |
| `max_linear_speed` | `1.5` | X/Y 线速度上限，m/s |
| `max_angular_speed` | `3.0` | Z 角速度上限，rad/s |

查看运行时参数：

```bash
ros2 param list /vehicle_core
ros2 param get /vehicle_core can_iface
```

## 9. 故障排查

### 看不到控制帧

```bash
ros2 node list
ros2 topic hz /vehicle_cmd
candump can0 | grep -i "18C4D1D0\|18C4D7D0"
```

### VCU 没有反馈

```bash
ip -details link show can0
candump can0
```

检查 CAN 接口是否为 `UP`、波特率是否为 500 kbit/s，以及终端电阻和线序是否正确。

### 查看故障和急停

```bash
ros2 topic echo /vehicle_feedback
```

`fault_level`：`0x00` 表示无故障，`0xE1`、`0xE2`、`0xE3` 分别表示一般、严重和致命故障。具体故障编码请参考底盘手册或联系售后。


## 指令超时保护

`command_timeout_sec` 默认 0.25 秒。`vehicle_core` 使用单调时钟检查最近一次有效
`/vehicle_cmd` 的接收时间；启动后尚无命令或超时后，持续发送禁用零速心跳。
新有效命令到达后恢复控制。NaN/Inf 速度字段立即禁用；正常退出会实际下发禁用帧。
上层应以高于超时倒数的频率持续发布，建议至少 10 Hz。

```bash
ros2 launch can_client can_client.launch.py command_timeout_sec:=0.25
```
