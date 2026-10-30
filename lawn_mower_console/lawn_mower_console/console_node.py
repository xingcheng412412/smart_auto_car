import math
from pathlib import Path
import threading
import time

from can_client.msg import VehicleFeedback
from geometry_msgs.msg import PoseStamped
from lawn_mower_recorder.msg import RecordStatus
from lawn_mower_recorder.srv import ProcessSwitch
from lawn_mower_tracker.msg import PathStatus
from lawn_mower_tracker.srv import MowerTrackControl
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rtk_gps_driver_cpp.msg import RTKGPSLog

from lawn_mower_console.path_loader import (
    load_target_path,
    validate_map_name,
)
from lawn_mower_console.rtk_formatter import format_rtk_log
from lawn_mower_console.state import ConsoleState


class ConsoleNode(Node):
    def __init__(self):
        super().__init__("lawn_mower_console")
        self.declare_parameter(
            "path_root", "~/fw_mini_pro_control_ros2/recorded_files"
        )
        self.declare_parameter(
            "path_filename", "lawn_mower_records.csv"
        )
        self.declare_parameter("pose_topic", "/gnss_pose")
        self.declare_parameter("rtk_log_topic", "/rtk_log")
        self.declare_parameter("record_status_topic", "/record_status")
        self.declare_parameter("path_status_topic", "/path_status")
        self.declare_parameter(
            "vehicle_feedback_topic", "/vehicle_feedback"
        )
        self.declare_parameter("record_service", "/process_switch")
        self.declare_parameter("track_service", "/mowerTrackControl")
        self.declare_parameter("gnss_timeout_sec", 1.0)
        self.declare_parameter("service_timeout_sec", 3.0)
        self.declare_parameter("max_rtk_log_lines", 500)

        self.path_root = Path(
            self.get_parameter("path_root").value
        ).expanduser()
        self.path_filename = str(
            self.get_parameter("path_filename").value
        )
        self.service_timeout_sec = float(
            self.get_parameter("service_timeout_sec").value
        )
        if self.service_timeout_sec <= 0.0:
            raise ValueError("service_timeout_sec must be positive")

        self.state = ConsoleState(
            max_rtk_log_lines=int(
                self.get_parameter("max_rtk_log_lines").value
            ),
            gnss_timeout_sec=float(
                self.get_parameter("gnss_timeout_sec").value
            ),
        )
        self.current_recording_map = ""
        self._pending = {}
        self._pending_lock = threading.Lock()

        self.create_subscription(
            PoseStamped,
            str(self.get_parameter("pose_topic").value),
            self._pose_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            RTKGPSLog,
            str(self.get_parameter("rtk_log_topic").value),
            self._rtk_log_callback,
            50,
        )
        self.create_subscription(
            RecordStatus,
            str(self.get_parameter("record_status_topic").value),
            self._record_status_callback,
            10,
        )
        self.create_subscription(
            PathStatus,
            str(self.get_parameter("path_status_topic").value),
            self._path_status_callback,
            10,
        )
        self.create_subscription(
            VehicleFeedback,
            str(self.get_parameter("vehicle_feedback_topic").value),
            self._vehicle_feedback_callback,
            10,
        )
        self.record_client = self.create_client(
            ProcessSwitch,
            str(self.get_parameter("record_service").value),
        )
        self.track_client = self.create_client(
            MowerTrackControl,
            str(self.get_parameter("track_service").value),
        )
        self.create_timer(0.1, self._check_timeouts)

    def _now_ns(self):
        return self.get_clock().now().nanoseconds

    def _pose_callback(self, message):
        q = message.pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        position = message.pose.position
        self.state.update_pose(
            position.x, position.y, position.z, yaw, self._now_ns()
        )

    def _rtk_log_callback(self, message):
        received_ns = self._now_ns()
        try:
            line = format_rtk_log(message, received_ns)
        except Exception as error:
            self.get_logger().error(f"Failed to format RTK log: {error}")
            return
        self.state.append_rtk_log(line)

    def _record_status_callback(self, message):
        self.state.set_record_status(
            message.record_state,
            message.record_count,
            message.cumulative_distance,
            message.map_name,
            self._now_ns(),
        )

    def _path_status_callback(self, message):
        self.state.set_tracker_status(
            message.tracker_state, message.line_id, self._now_ns()
        )

    def _vehicle_feedback_callback(self, message):
        bumper = any(
            (
                message.bumper_front_left,
                message.bumper_front_center,
                message.bumper_front_right,
                message.bumper_rear_left,
                message.bumper_rear_center,
                message.bumper_rear_right,
            )
        )
        self.state.set_vehicle_status(
            True,
            message.estop,
            bumper,
            message.battery_percent,
            message.fault_level,
            self._now_ns(),
        )

    def _reject(self, operation, message):
        if self.state.begin_operation(operation):
            self.state.finish_operation(operation, False, message)
        return False

    def _send(self, operation, client, request, success_callback=None):
        if not self.state.begin_operation(operation):
            return False
        if not client.service_is_ready():
            self.state.finish_operation(
                operation, False, "service is not available"
            )
            return False

        future = client.call_async(request)
        deadline = time.monotonic() + self.service_timeout_sec
        with self._pending_lock:
            self._pending[operation] = (future, deadline)

        def done_callback(completed):
            with self._pending_lock:
                entry = self._pending.pop(operation, None)
            if entry is None:
                return
            try:
                response = completed.result()
            except Exception as error:
                self.state.finish_operation(
                    operation, False, f"service call failed: {error}"
                )
                return
            success = bool(response.success)
            message = str(response.message)
            if success and success_callback is not None:
                success_callback()
            self.state.finish_operation(
                operation, success, message or operation
            )

        future.add_done_callback(done_callback)
        return True

    def _check_timeouts(self):
        expired = []
        now = time.monotonic()
        with self._pending_lock:
            for operation, (_, deadline) in self._pending.items():
                if now >= deadline:
                    expired.append(operation)
            for operation in expired:
                self._pending.pop(operation, None)
        for operation in expired:
            self.state.finish_operation(
                operation, False, "service request timed out"
            )

    def start_recording(self, map_name):
        try:
            safe_name = validate_map_name(map_name)
        except ValueError as error:
            return self._reject("start recording", str(error))
        allowed, reason = self.state.can_start_recording()
        if not allowed:
            return self._reject("start recording", reason)

        request = ProcessSwitch.Request()
        request.data = True
        request.map_name = ""
        request.record_status = ""
        return self._send(
            "start recording",
            self.record_client,
            request,
            lambda: setattr(self, "current_recording_map", safe_name),
        )

    def stop_recording(self, map_name):
        try:
            safe_name = validate_map_name(map_name)
        except ValueError as error:
            return self._reject("stop recording", str(error))
        request = ProcessSwitch.Request()
        request.data = False
        request.map_name = safe_name
        request.record_status = ""
        return self._send(
            "stop recording",
            self.record_client,
            request,
            lambda: setattr(self, "current_recording_map", ""),
        )

    def start_tracking(self, map_name):
        operation = "start tracking"
        try:
            safe_name = validate_map_name(map_name)
            load_target_path(
                self.path_root, safe_name, self.path_filename
            )
        except (OSError, ValueError) as error:
            return self._reject(operation, str(error))
        allowed, reason = self.state.can_start_tracking()
        if not allowed:
            return self._reject(operation, reason)
        if not self.state.snapshot(self._now_ns()).gnss_fresh:
            return self._reject(operation, "GNSS data is stale")

        request = MowerTrackControl.Request()
        request.track_state = "start"
        request.map_name = safe_name
        request.line_id = 1
        return self._send(
            operation, self.track_client, request
        )

    def _track_command(self, command):
        operation = f"{command} tracking"
        snapshot = self.state.snapshot(self._now_ns())
        request = MowerTrackControl.Request()
        request.track_state = command
        request.map_name = ""
        request.line_id = max(1, snapshot.line_id)
        return self._send(operation, self.track_client, request)

    def pause_tracking(self):
        return self._track_command("pause")

    def resume_tracking(self):
        return self._track_command("resume")

    def stop_tracking(self):
        return self._track_command("stop")
