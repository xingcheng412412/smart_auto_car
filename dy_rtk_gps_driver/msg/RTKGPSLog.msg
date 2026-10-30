# RTKGPSLog.msg - INS5720DAA BDDB0B驱动日志消息

std_msgs/Header header

# 日志级别: INFO, WARN, ERROR
string log_level

# ========== INS状态 ==========
# INS初始对准状态: 0=无定位, 1=位置对准, 4=完全对准(INS)
int32 initial_state
# 纬度(度)
float64 latitude
# 经度(度)
float64 longitude
# 高度(m)
float64 altitude

# INS状态: bit0=位置, bit1=速度, bit2=姿态, bit3=航向角; 为1表示完成初对准
uint8 ins_status

# ========== 轮询数据(根据poll_type解析) ==========
# --- Type=0: 定位信息精度(m) ---
float64 lat_std
float64 lon_std
float64 h_std

# --- Type=1: 定速信息精度(m/s) ---
float64 vn_std
float64 ve_std
float64 vd_std

# --- Type=2: 姿态信息精度(deg), 计算: data*200/32768 ---
float64 roll_std
float64 pitch_std
float64 yaw_std

# --- Type=22: 设备内部温度(℃) ---
float64 temperature

# --- Type=32: GPS状态 ---
# GPS定位状态
int32 gps_position_status
# 收星数
int32 gps_num_satellites
# GPS定向状态(差分标志位):
# 0=NONE无解, 1=FIXEDPOS, 2=FIXEDHEIGHT, 8=DOPPLER_VELOCITY
# 16=SINGLE单点定位, 17=PSRDIFF伪距差分, 18=SBAS
# 32=L1_FLOAT, 33=IONOFREE_FLOAT, 34=NARROW_FLOAT
# 48=L1_INT固定解, 49=WIDE_INT宽巷固定解, 50=NARROW_INT窄巷固定解
int32 gps_diff_status

# --- Type=33: 轮速状态 ---
# 轮速有无标志(不为0表示有轮速, 连续为0表示无轮速)
int32 wheel_speed_status

# ========== INS对准统计 ==========
# 非完全对准次数(initial_state!=4的帧累计)
int32 non_rtk_fixed_count
# 当前非完全对准持续时间(秒), 完全对准时为0
float64 non_rtk_fixed_duration

# ========== GNSS Pose状态 ==========
# gnss_pose发布状态:
# INACTIVE=未启用(缺少原点参数)
# WAITING=等待INS对准和RTK固定解
# STATUS_HOLD=INS已对准且RTK固定, 等待5s稳定门限
# PUBLISHING=正在发布gnss_pose
# STATUS_RESET=状态不满足, 已重置
# INVALID_SAMPLE=无效数据
# REJECTED_OUTLIER=速度超限, 拒绝离群点
# TIMEOUT=数据超时, 停止发布
string gnss_pose_state

# ========== 错误信息 ==========
string error_type
string error_description
