# 车辆状态反馈
# 汇总 FW-mini-pro CAN 协议的 ctrl_fb(0x18C4D1EF)、io_fb(0x18C4DAEF)、
# bms_fb(0x18C4E1EF)、bms_flag_fb(0x18C4E2EF)、error_fb(0x18C4EAEF) 帧内容

std_msgs/Header header

# ---- ctrl_fb：当前运动状态 ----
uint8   gear           # 见 VehicleCmd.GEAR_*
float32 linear_x        # m/s
float32 linear_y        # m/s
float32 angular_z       # rad/s

# ---- io_fb：开关与安全状态 ----
bool estop                  # 急停开关是否触发
bool remote_control         # true=遥控器控制中，false=指令(CAN)控制中
bool charging               # 是否接入充电桩或手动充电器
bool bumper_front_left
bool bumper_front_center
bool bumper_front_right
bool bumper_rear_left
bool bumper_rear_center
bool bumper_rear_right

# ---- bms_fb / bms_flag_fb：电池状态 ----
float32 battery_voltage     # V
float32 battery_current     # A，正值代表充电
float32 battery_percent     # 0~100
float32 battery_temp_max    # 摄氏度
float32 battery_temp_min    # 摄氏度
bool    battery_fault       # 任意电池保护位被触发

# ---- error_fb：故障信息（原始值透传，具体编码见售后资料）----
uint8  fault_level          # 0x00=无故障 0xE1/0xE2/0xE3=一般/严重/致命故障
uint8  fault_component      # 0x00=无 0xB0=电池 0xC0=VCU 0xD1=驱动 0xD2=转向 0xD3=制动 0xF0=传感器 0x80=其他
uint8  fault_id              # 0x00=默认 0x06=左前 0x07=左后 0x08=右前 0x09=右后
uint32 fault_code
