# 原始 CAN 帧，仅用于监控/调试（如 script/socketcan_bridge.py）

std_msgs/Header header
uint32   id            # CAN ID（29位扩展帧时不含 EFF/RTR/ERR 标志位）
bool     is_extended    # true=29位扩展帧，false=11位标准帧
uint8    dlc
uint8[8] data
