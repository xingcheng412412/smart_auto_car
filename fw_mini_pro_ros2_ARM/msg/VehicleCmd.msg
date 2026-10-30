# 车辆运动控制指令
# 对应 FW-mini-pro CAN 协议 ctrl_cmd 帧 (0x18C4D1D0)，详见用户手册 §4.4

uint8 GEAR_DISABLE = 0
uint8 GEAR_PARK    = 1
uint8 GEAR_NEUTRAL = 2
uint8 GEAR_4WD4WS  = 6   # 四转四驱档：linear_x + angular_z 有效
uint8 GEAR_CRAB    = 8   # 横移档：linear_x + linear_y 有效

bool  enable        # false 时强制切换为 disable 档，车辆不接受运动指令
uint8 gear          # 见上方 GEAR_* 常量，默认应使用 GEAR_4WD4WS

float32 linear_x     # m/s，车体 X 轴线速度，前正后负
float32 linear_y     # m/s，车体 Y 轴线速度（仅 GEAR_CRAB 有效），左正右负
float32 angular_z    # rad/s，车体 Z 轴角速度（仅 GEAR_4WD4WS 有效），逆时针为正
