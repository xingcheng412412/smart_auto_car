import curses
from datetime import datetime
import math


KEYS = ("R", "E", "T", "P", "C", "S", "X", "Q")

RECORD_NAMES = {
    0: "空闲",
    1: "录制中",
    2: "已停止",
    3: "暂停",
}
TRACK_NAMES = {
    0: "空闲",
    3: "跟踪中",
    4: "已暂停",
    5: "停止/错误",
    6: "已完成",
}


def _safe_add(window, row, column, text, attr=0, max_width=None):
    height, width = window.getmaxyx()
    if row < 0 or column < 0 or row >= height or column >= width:
        return
    available = width - column - 1
    if max_width is not None:
        available = min(available, max_width)
    if available <= 0:
        return
    try:
        window.addstr(row, column, str(text)[:available], attr)
    except curses.error:
        pass


def prompt_map_name(window, title, initial=""):
    height, width = window.getmaxyx()
    dialog_width = min(60, max(30, width - 4))
    dialog_height = 7
    if height < dialog_height + 2 or width < 32:
        return None
    dialog = curses.newwin(
        dialog_height,
        dialog_width,
        (height - dialog_height) // 2,
        (width - dialog_width) // 2,
    )
    dialog.keypad(True)
    dialog.timeout(-1)
    value = str(initial)
    while True:
        dialog.erase()
        dialog.box()
        _safe_add(dialog, 1, 2, title, curses.A_BOLD)
        _safe_add(dialog, 3, 2, value, max_width=dialog_width - 4)
        _safe_add(dialog, 5, 2, "Enter 确认  Esc 取消", curses.A_DIM)
        dialog.refresh()
        key = dialog.getch()
        if key in (10, 13, curses.KEY_ENTER):
            return value.strip() or None
        if key == 27:
            return None
        if key in (curses.KEY_BACKSPACE, 127, 8):
            value = value[:-1]
        elif 0 <= key <= 0x10FFFF:
            character = chr(key)
            if character.isprintable() and len(value) < dialog_width - 5:
                value += character


def _handle_key(key, window, node, state):
    if not 0 <= key < 256:
        return True
    command = chr(key).upper()
    if command == "Q":
        return False
    if command == "R":
        map_name = prompt_map_name(window, "输入录制地图名")
        if map_name:
            node.start_recording(map_name)
    elif command == "E":
        map_name = node.current_recording_map
        if not map_name:
            map_name = prompt_map_name(window, "输入保存地图名")
        if map_name:
            node.stop_recording(map_name)
    elif command == "T":
        map_name = prompt_map_name(window, "输入规控地图名")
        if map_name:
            node.start_tracking(map_name)
    elif command == "P":
        node.pause_tracking()
    elif command == "C":
        node.resume_tracking()
    elif command == "S":
        node.stop_tracking()
    elif command == "X":
        state.clear_rtk_logs()
    return True


def _log_time(received_ns):
    try:
        return datetime.fromtimestamp(
            received_ns / 1_000_000_000
        ).strftime("%H:%M:%S")
    except (OSError, OverflowError, ValueError):
        return "--:--:--"


def _log_attr(level, colors):
    if level == "ERROR":
        return colors["error"]
    if level == "WARN":
        return colors["warning"]
    return colors["normal"]


def _operation(snapshot):
    if snapshot.pending_operation:
        return f"处理中: {snapshot.pending_operation}", "warning"
    if snapshot.last_operation_message:
        kind = "error" if snapshot.last_operation_success is False else "normal"
        return snapshot.last_operation_message, kind
    return "等待操作", "normal"


def _draw_compact(window, snapshot, colors):
    height, width = window.getmaxyx()
    _safe_add(window, 0, 0, "=" * width, colors["title"])
    _safe_add(
        window,
        1,
        1,
        "终端至少需要 80x24，请扩大 SSH 终端窗口",
        colors["warning"],
    )
    gnss = "在线" if snapshot.gnss_fresh else "超时"
    _safe_add(window, 3, 1, f"GNSS:{gnss}")
    _safe_add(
        window,
        4,
        1,
        f"Recorder:{RECORD_NAMES.get(snapshot.record_state, '?')} "
        f"Tracker:{TRACK_NAMES.get(snapshot.tracker_state, '?')}",
    )
    important = [
        line for line in snapshot.rtk_logs if line.level in ("ERROR", "WARN")
    ][-max(1, height - 8):]
    for index, line in enumerate(important, start=6):
        text = f"{_log_time(line.received_ns)} {line.level} {line.text}"
        _safe_add(window, index, 1, text, _log_attr(line.level, colors))


def _draw_full(window, snapshot, colors):
    height, width = window.getmaxyx()
    divider = max(48, int(width * 0.66))
    _safe_add(window, 0, 0, "=" * width, colors["title"])
    _safe_add(window, 1, 2, "RTK 状态日志", colors["title"])
    _safe_add(window, 1, divider + 2, "GNSS Pose", colors["title"])
    _safe_add(window, 2, 0, "-" * width, colors["dim"])

    log_bottom = height - 7
    visible_count = max(1, log_bottom - 3)
    for row, line in enumerate(snapshot.rtk_logs[-visible_count:], start=3):
        text = f"{_log_time(line.received_ns)} {line.level:<5} {line.text}"
        _safe_add(
            window,
            row,
            1,
            text,
            _log_attr(line.level, colors),
            divider - 2,
        )

    for row in range(1, log_bottom):
        _safe_add(window, row, divider, "|", colors["dim"])

    gnss_attr = colors["normal"] if snapshot.gnss_fresh else colors["error"]
    _safe_add(
        window,
        3,
        divider + 2,
        f"状态: {'在线' if snapshot.gnss_fresh else '超时'}",
        gnss_attr,
    )
    if snapshot.pose_x is None:
        pose_lines = ("x: --", "y: --", "z: --", "yaw: --", "更新: --")
    else:
        yaw_degrees = math.degrees(snapshot.pose_yaw)
        age = (
            f"{snapshot.gnss_age_sec:.2f}s前"
            if snapshot.gnss_age_sec is not None
            else "--"
        )
        pose_lines = (
            f"x: {snapshot.pose_x:.3f} m",
            f"y: {snapshot.pose_y:.3f} m",
            f"z: {snapshot.pose_z:.3f} m",
            f"yaw: {yaw_degrees:.1f} deg",
            f"更新: {age}",
        )
    for index, text in enumerate(pose_lines, start=5):
        _safe_add(window, index, divider + 2, text)

    vehicle_error = (
        snapshot.vehicle_estop
        or snapshot.vehicle_bumper
        or snapshot.fault_level != 0
    )
    vehicle_attr = colors["error"] if vehicle_error else colors["normal"]
    _safe_add(
        window,
        12,
        divider + 2,
        f"底盘: {'在线' if snapshot.vehicle_online else '等待'}",
    )
    _safe_add(
        window,
        13,
        divider + 2,
        f"急停:{'触发' if snapshot.vehicle_estop else '正常'} "
        f"碰撞:{'触发' if snapshot.vehicle_bumper else '正常'}",
        vehicle_attr,
    )
    _safe_add(
        window,
        14,
        divider + 2,
        f"电池:{snapshot.battery_percent:.1f}% "
        f"故障:0x{snapshot.fault_level:02X}",
        vehicle_attr,
    )

    _safe_add(window, height - 7, 0, "-" * width, colors["dim"])
    _safe_add(
        window,
        height - 6,
        1,
        f"Recorder:{RECORD_NAMES.get(snapshot.record_state, '?')} "
        f"点数:{snapshot.record_count} "
        f"距离:{snapshot.record_distance:.1f}m "
        f"地图:{snapshot.record_map or '-'}",
    )
    _safe_add(
        window,
        height - 5,
        1,
        f"Tracker:{TRACK_NAMES.get(snapshot.tracker_state, '?')} "
        f"line_id:{snapshot.line_id}",
    )
    operation_text, operation_kind = _operation(snapshot)
    _safe_add(
        window,
        height - 4,
        1,
        f"最近操作: {operation_text}",
        colors[operation_kind],
    )
    _safe_add(window, height - 3, 0, "-" * width, colors["dim"])
    _safe_add(
        window,
        height - 2,
        1,
        "R录制 E保存 T规控 P暂停 C恢复 S停止 X清日志 Q退出",
        colors["title"],
    )


def run_tui(stdscr, node, state):
    curses.curs_set(0)
    stdscr.keypad(True)
    stdscr.timeout(100)
    curses.start_color()
    curses.use_default_colors()
    curses.init_pair(1, curses.COLOR_CYAN, -1)
    curses.init_pair(2, curses.COLOR_GREEN, -1)
    curses.init_pair(3, curses.COLOR_YELLOW, -1)
    curses.init_pair(4, curses.COLOR_RED, -1)
    colors = {
        "title": curses.color_pair(1) | curses.A_BOLD,
        "normal": curses.color_pair(2),
        "warning": curses.color_pair(3) | curses.A_BOLD,
        "error": curses.color_pair(4) | curses.A_BOLD,
        "dim": curses.A_DIM,
    }
    running = True
    while running:
        key = stdscr.getch()
        if key != -1:
            running = _handle_key(key, stdscr, node, state)
        snapshot = state.snapshot(node.get_clock().now().nanoseconds)
        height, width = stdscr.getmaxyx()
        stdscr.erase()
        if height < 24 or width < 80:
            _draw_compact(stdscr, snapshot, colors)
        else:
            _draw_full(stdscr, snapshot, colors)
        stdscr.refresh()
