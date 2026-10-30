#!/usr/bin/env python3
"""
车辆上位机 - 终端 TUI（curses，无需安装任何依赖）
运行: ./run.sh
操作: 方向键调节，空格使能，q退出
"""
import curses
import math
import threading
import sys
import time

import rclpy
from rclpy.node import Node
from can_client.msg import VehicleCmd, VehicleFeedback
from std_msgs.msg import UInt8

# ── 共享状态 ──────────────────────────────────
state = {
    'enabled':   False,
    'gear':      VehicleCmd.GEAR_4WD4WS,   # 4T4D 或 横移(GEAR_CRAB)
    'linear_x':  0.0,    # m/s
    'linear_y':  0.0,    # m/s，仅横移档有效
    'angular_z': 0.0,    # rad/s，仅4T4D档有效
}
feedback = {
    'gear': 0, 'linear_x': 0.0, 'linear_y': 0.0, 'angular_z': 0.0,
    'estop': False, 'remote_control': False, 'charging': False,
    'bumper': False,
    'battery_percent': 0.0, 'battery_voltage': 0.0, 'battery_current': 0.0,
    'fault_level': 0,
}

GEAR_NAMES = {
    VehicleCmd.GEAR_DISABLE: 'disable',
    VehicleCmd.GEAR_PARK:    '驻车',
    VehicleCmd.GEAR_NEUTRAL: '空档',
    VehicleCmd.GEAR_4WD4WS:  '4T4D',
    VehicleCmd.GEAR_CRAB:    '横移',
}

ros_node = None


def feedback_callback(msg):
    feedback['gear']            = int(msg.gear)
    feedback['linear_x']        = float(msg.linear_x)
    feedback['linear_y']        = float(msg.linear_y)
    feedback['angular_z']       = float(msg.angular_z)
    feedback['estop']           = bool(msg.estop)
    feedback['remote_control']  = bool(msg.remote_control)
    feedback['charging']        = bool(msg.charging)
    feedback['bumper']          = any([
        msg.bumper_front_left, msg.bumper_front_center, msg.bumper_front_right,
        msg.bumper_rear_left, msg.bumper_rear_center, msg.bumper_rear_right,
    ])
    feedback['battery_percent'] = float(msg.battery_percent)
    feedback['battery_voltage'] = float(msg.battery_voltage)
    feedback['battery_current'] = float(msg.battery_current)
    feedback['fault_level']     = int(msg.fault_level)


class UpperComputerNode(Node):
    def __init__(self):
        super().__init__('upper_computer')
        self.vehicle_cmd_pub = self.create_publisher(
            VehicleCmd, '/vehicle_cmd', 10)
        self.fault_reset_pub = self.create_publisher(
            UInt8, '/fault_reset', 10)
        self.feedback_sub = self.create_subscription(
            VehicleFeedback, '/vehicle_feedback', feedback_callback, 10)
        self.publish_timer = self.create_timer(0.1, self.publish_command)

    def publish_command(self):
        msg = VehicleCmd()
        msg.enable    = state['enabled']
        msg.gear      = state['gear']
        msg.linear_x  = state['linear_x']
        msg.linear_y  = state['linear_y']
        msg.angular_z = state['angular_z']
        self.vehicle_cmd_pub.publish(msg)


# ── TUI ───────────────────────────────────────
def draw(stdscr):
    curses.curs_set(0)
    stdscr.nodelay(True)
    curses.start_color()
    curses.init_pair(1, curses.COLOR_GREEN,  curses.COLOR_BLACK)  # 正常/使能
    curses.init_pair(2, curses.COLOR_RED,    curses.COLOR_BLACK)  # 错误/禁用
    curses.init_pair(3, curses.COLOR_YELLOW, curses.COLOR_BLACK)  # 数值
    curses.init_pair(4, curses.COLOR_CYAN,   curses.COLOR_BLACK)  # 标题
    curses.init_pair(5, curses.COLOR_WHITE,  curses.COLOR_BLACK)  # 普通

    GRN = curses.color_pair(1) | curses.A_BOLD
    RED = curses.color_pair(2) | curses.A_BOLD
    YEL = curses.color_pair(3) | curses.A_BOLD
    CYN = curses.color_pair(4) | curses.A_BOLD
    NRM = curses.color_pair(5)

    while rclpy.ok():
        key = stdscr.getch()

        # ── 按键处理 ──────────────────────────
        if key == ord('q') or key == ord('Q'):
            break
        elif key == ord(' '):                          # 空格：切换使能
            state['enabled'] = not state['enabled']
        elif key == ord('s') or key == ord('S'):       # S：紧急停车
            state['enabled']   = False
            state['linear_x']  = 0.0
            state['linear_y']  = 0.0
            state['angular_z'] = 0.0
        elif key == curses.KEY_UP:                     # ↑：X 轴速度 +0.1m/s
            state['linear_x'] = min(1.5, state['linear_x'] + 0.1)
        elif key == curses.KEY_DOWN:                   # ↓：X 轴速度 -0.1m/s
            state['linear_x'] = max(-1.5, state['linear_x'] - 0.1)
        elif key == curses.KEY_RIGHT:                  # →：右转/右移
            if state['gear'] == VehicleCmd.GEAR_CRAB:
                state['linear_y'] = max(-1.5, state['linear_y'] - 0.1)
            else:
                state['angular_z'] = max(-1.5, state['angular_z'] - 0.1)
        elif key == curses.KEY_LEFT:                   # ←：左转/左移
            if state['gear'] == VehicleCmd.GEAR_CRAB:
                state['linear_y'] = min(1.5, state['linear_y'] + 0.1)
            else:
                state['angular_z'] = min(1.5, state['angular_z'] + 0.1)
        elif key == ord('0'):                          # 0：速度归零
            state['linear_x'] = 0.0
            state['linear_y'] = 0.0
        elif key == ord('c') or key == ord('C'):       # C：角速度归零
            state['angular_z'] = 0.0
        elif key == ord('g') or key == ord('G'):       # G：切换 4T4D / 横移档
            if state['gear'] == VehicleCmd.GEAR_4WD4WS:
                state['gear'] = VehicleCmd.GEAR_CRAB
            else:
                state['gear'] = VehicleCmd.GEAR_4WD4WS
            state['linear_y']  = 0.0
            state['angular_z'] = 0.0
        elif key == ord('r') or key == ord('R'):       # R：安全停车解锁 / 故障复位
            msg = UInt8(); msg.data = 1
            ros_node.fault_reset_pub.publish(msg)

        # ── 绘制界面 ──────────────────────────
        stdscr.erase()
        h, w = stdscr.getmaxyx()
        row = 0

        def put(r, c, text, attr=NRM):
            try:
                stdscr.addstr(r, c, text, attr)
            except curses.error:
                pass

        title = '  FW-mini-pro 车辆上位机  '
        put(row, 0, '=' * w, CYN)
        row += 1
        put(row, max(0, (w - len(title)) // 2), title, CYN)
        row += 1
        put(row, 0, '=' * w, CYN)
        row += 2

        # VCU / 遥控 / 急停状态
        vcu_ok = feedback['gear'] != VehicleCmd.GEAR_DISABLE
        put(row, 2, '● VCU 已连接' if vcu_ok else '● VCU 未连接', GRN if vcu_ok else RED)
        put(row, 20, '● 遥控器控制中' if feedback['remote_control'] else '● 指令控制中',
            YEL if feedback['remote_control'] else NRM)
        row += 1
        put(row, 2, '● 急停触发' if feedback['estop'] else '● 急停释放',
            RED if feedback['estop'] else GRN)
        put(row, 20, '● 防撞条触发' if feedback['bumper'] else '● 防撞条正常',
            RED if feedback['bumper'] else GRN)
        row += 1
        put(row, 2, '● 使能: ON ' if state['enabled'] else '● 使能: OFF',
            GRN if state['enabled'] else RED)
        put(row, 20, f'● 档位: {GEAR_NAMES.get(state["gear"], "?")}', CYN)
        row += 2

        # 指令 / 反馈
        put(row, 0, '─' * w, NRM); row += 1
        put(row, 2, '指令', CYN)
        put(row, 30, '反馈', CYN); row += 1

        put(row, 2,  f'X轴速度: {state["linear_x"]:+6.2f} m/s', YEL)
        put(row, 30, f'反馈: {feedback["linear_x"]:+6.2f} m/s', NRM)
        row += 1

        if state['gear'] == VehicleCmd.GEAR_CRAB:
            put(row, 2,  f'Y轴速度: {state["linear_y"]:+6.2f} m/s', YEL)
            put(row, 30, f'反馈: {feedback["linear_y"]:+6.2f} m/s', NRM)
        else:
            put(row, 2,  f'Z轴角速度: {math.degrees(state["angular_z"]):+6.1f} °/s', YEL)
            put(row, 30, f'反馈: {math.degrees(feedback["angular_z"]):+6.1f} °/s', NRM)
        row += 2

        # 电池
        put(row, 0, '─' * w, NRM); row += 1
        put(row, 2, f'电池: {feedback["battery_percent"]:5.1f} %', NRM)
        put(row, 20, f'{feedback["battery_voltage"]:5.2f} V', NRM)
        put(row, 32, f'{feedback["battery_current"]:+6.2f} A', NRM)
        row += 2

        # 故障
        put(row, 0, '─' * w, NRM); row += 1
        if feedback['fault_level']:
            put(row, 2, f'⚠  故障等级: 0x{feedback["fault_level"]:02X}  按 R 尝试复位/解锁', RED)
        else:
            put(row, 2, '✓  无故障', GRN)
        row += 2

        # 速度条
        put(row, 0, '─' * w, NRM); row += 1
        bar_w = max(10, w - 24)
        ratio = (state['linear_x'] + 1.5) / 3.0
        fill  = int(ratio * bar_w)
        bar   = '█' * fill + '░' * (bar_w - fill)
        put(row, 2, f'X速度 [{bar}] {state["linear_x"]:+5.2f}m/s', YEL)
        row += 2

        # 操作说明
        put(row, 0, '─' * w, NRM); row += 1
        put(row, 2, '↑↓ X轴速度±0.1  ←→ 转向/横移±0.1  空格 使能  G 切换档位', NRM)
        row += 1
        put(row, 2, '0 速度归零  C 角速度归零  R 安全停车解锁/故障复位  S 紧急停车  Q 退出', NRM)

        stdscr.refresh()
        curses.napms(100)   # 100ms 刷新

    # 退出时停车
    state['enabled']   = False
    state['linear_x']  = 0.0
    state['linear_y']  = 0.0
    state['angular_z'] = 0.0


# ── 主程序 ────────────────────────────────────
if __name__ == '__main__':
    enable_on_start = '--enable' in sys.argv
    ros_args = [arg for arg in sys.argv if arg != '--enable']
    if enable_on_start:
        state['enabled'] = True

    rclpy.init(args=ros_args)
    ros_node = UpperComputerNode()
    spin_thread = threading.Thread(
        target=rclpy.spin, args=(ros_node,), daemon=True)
    spin_thread.start()

    try:
        curses.wrapper(draw)
    finally:
        ros_node.publish_command()
        time.sleep(0.1)
        if rclpy.ok():
            rclpy.shutdown()
        spin_thread.join(timeout=1.0)
        ros_node.destroy_node()
