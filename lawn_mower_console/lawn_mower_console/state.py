from collections import deque
from dataclasses import dataclass
import threading

from lawn_mower_console.rtk_formatter import RtkLogLine


@dataclass(frozen=True)
class Snapshot:
    pose_x: float | None
    pose_y: float | None
    pose_z: float | None
    pose_yaw: float | None
    gnss_received_ns: int
    gnss_fresh: bool
    gnss_age_sec: float | None
    rtk_logs: tuple
    record_state: int
    record_count: int
    record_distance: float
    record_map: str
    record_received_ns: int
    tracker_state: int
    line_id: int
    tracker_received_ns: int
    vehicle_online: bool
    vehicle_estop: bool
    vehicle_bumper: bool
    battery_percent: float
    fault_level: int
    vehicle_received_ns: int
    pending_operation: str
    last_operation_success: bool | None
    last_operation_message: str


class ConsoleState:
    def __init__(
        self,
        max_rtk_log_lines=500,
        gnss_timeout_sec=1.0,
    ):
        if max_rtk_log_lines < 1:
            raise ValueError("max_rtk_log_lines must be positive")
        if gnss_timeout_sec <= 0.0:
            raise ValueError("gnss_timeout_sec must be positive")

        self._lock = threading.Lock()
        self._rtk_logs = deque(maxlen=max_rtk_log_lines)
        self._gnss_timeout_ns = int(gnss_timeout_sec * 1_000_000_000)
        self._pose_x = None
        self._pose_y = None
        self._pose_z = None
        self._pose_yaw = None
        self._gnss_received_ns = 0
        self._record_state = 0
        self._record_count = 0
        self._record_distance = 0.0
        self._record_map = ""
        self._record_received_ns = 0
        self._tracker_state = 0
        self._line_id = 1
        self._tracker_received_ns = 0
        self._vehicle_online = False
        self._vehicle_estop = False
        self._vehicle_bumper = False
        self._battery_percent = 0.0
        self._fault_level = 0
        self._vehicle_received_ns = 0
        self._pending_operation = ""
        self._last_operation_success = None
        self._last_operation_message = ""

    def update_pose(self, x, y, z, yaw, received_ns):
        with self._lock:
            self._pose_x = float(x)
            self._pose_y = float(y)
            self._pose_z = float(z)
            self._pose_yaw = float(yaw)
            self._gnss_received_ns = int(received_ns)

    def append_rtk_log(self, line):
        if not isinstance(line, RtkLogLine):
            raise TypeError("line must be an RtkLogLine")
        with self._lock:
            self._rtk_logs.append(line)

    def clear_rtk_logs(self):
        with self._lock:
            self._rtk_logs.clear()

    def set_record_status(
        self, state, count, distance, map_name, received_ns
    ):
        with self._lock:
            self._record_state = int(state)
            self._record_count = int(count)
            self._record_distance = float(distance)
            self._record_map = str(map_name)
            self._record_received_ns = int(received_ns)

    def set_tracker_status(self, state, line_id, received_ns):
        with self._lock:
            self._tracker_state = int(state)
            self._line_id = int(line_id)
            self._tracker_received_ns = int(received_ns)

    def set_vehicle_status(
        self,
        online,
        estop,
        bumper,
        battery,
        fault,
        received_ns,
    ):
        with self._lock:
            self._vehicle_online = bool(online)
            self._vehicle_estop = bool(estop)
            self._vehicle_bumper = bool(bumper)
            self._battery_percent = float(battery)
            self._fault_level = int(fault)
            self._vehicle_received_ns = int(received_ns)

    def begin_operation(self, name):
        with self._lock:
            if self._pending_operation:
                return False
            self._pending_operation = str(name)
            self._last_operation_success = None
            self._last_operation_message = ""
            return True

    def finish_operation(self, name, success, message):
        with self._lock:
            if self._pending_operation != name:
                return
            self._pending_operation = ""
            self._last_operation_success = bool(success)
            self._last_operation_message = str(message)

    def can_start_recording(self):
        with self._lock:
            if self._tracker_state in (3, 4):
                return False, "tracking is active"
            return True, ""

    def can_start_tracking(self):
        with self._lock:
            if self._record_state == 1:
                return False, "recording is active"
            return True, ""

    def snapshot(self, now_ns):
        with self._lock:
            gnss_age = int(now_ns) - self._gnss_received_ns
            gnss_fresh = (
                self._gnss_received_ns > 0
                and 0 <= gnss_age <= self._gnss_timeout_ns
            )
            return Snapshot(
                pose_x=self._pose_x,
                pose_y=self._pose_y,
                pose_z=self._pose_z,
                pose_yaw=self._pose_yaw,
                gnss_received_ns=self._gnss_received_ns,
                gnss_fresh=gnss_fresh,
                gnss_age_sec=(
                    gnss_age / 1_000_000_000
                    if self._gnss_received_ns > 0 and gnss_age >= 0
                    else None
                ),
                rtk_logs=tuple(self._rtk_logs),
                record_state=self._record_state,
                record_count=self._record_count,
                record_distance=self._record_distance,
                record_map=self._record_map,
                record_received_ns=self._record_received_ns,
                tracker_state=self._tracker_state,
                line_id=self._line_id,
                tracker_received_ns=self._tracker_received_ns,
                vehicle_online=self._vehicle_online,
                vehicle_estop=self._vehicle_estop,
                vehicle_bumper=self._vehicle_bumper,
                battery_percent=self._battery_percent,
                fault_level=self._fault_level,
                vehicle_received_ns=self._vehicle_received_ns,
                pending_operation=self._pending_operation,
                last_operation_success=self._last_operation_success,
                last_operation_message=self._last_operation_message,
            )
