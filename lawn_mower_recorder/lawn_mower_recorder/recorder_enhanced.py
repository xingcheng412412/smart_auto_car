#!/usr/bin/env python3
import math
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix

from lawn_mower_recorder.msg import RecordStatus
from lawn_mower_recorder.recorder_core import (
    RecordingSession,
    quaternion_to_euler,
)
from lawn_mower_recorder.srv import ProcessSwitch, RecordMarkPoint


class LawnMowerRecorder(Node):
    def __init__(self):
        super().__init__("lawn_mower_recorder_node")

        self.declare_parameter(
            "base_dir", "~/fw_mini_pro_control_ros2/recorded_files"
        )
        self.declare_parameter("pose_topic", "/gnss_pose")
        self.declare_parameter("fix_topic", "/fix")
        self.declare_parameter("record_status_topic", "/record_status")
        self.declare_parameter("record_period", 0.1)
        self.declare_parameter("status_period", 1.0)

        base_dir = Path(
            self.get_parameter("base_dir").get_parameter_value().string_value
        ).expanduser()
        pose_topic = self.get_parameter("pose_topic").value
        fix_topic = self.get_parameter("fix_topic").value
        status_topic = self.get_parameter("record_status_topic").value
        record_period = float(self.get_parameter("record_period").value)
        status_period = float(self.get_parameter("status_period").value)
        if record_period <= 0.0 or status_period <= 0.0:
            raise ValueError("timer periods must be positive")

        self.session = RecordingSession(base_dir)
        self.pose = None
        self.gnss_fix = None
        self.current_map_name = None
        self._last_wait_log_ns = 0

        self.linear_threshold = 3.0
        self.angular_threshold = 0.05
        self.min_straight_interval = 5.0
        self._reset_recording_state()

        self.create_subscription(
            PoseStamped,
            pose_topic,
            self.pose_cb,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            NavSatFix,
            fix_topic,
            self.gnss_fix_cb,
            qos_profile_sensor_data,
        )
        self.create_service(
            ProcessSwitch,
            "process_switch",
            self.handle_process_switch_request,
        )
        self.create_service(
            RecordMarkPoint,
            "record_mark_point",
            self.handle_record_mark_point_request,
        )
        self.record_status_pub = self.create_publisher(
            RecordStatus, status_topic, 10
        )
        self.create_timer(record_period, self.record_timer_cb)
        self.create_timer(status_period, self.publish_record_status)
        self.get_logger().info(
            "LawnMowerRecorder initialized; waiting for process_switch"
        )

    def _reset_recording_state(self):
        self.record_count = 0
        self.current_path_id = 1
        self.travelling_distance_of_this_path = 0.0
        self.cumulative_distance = 0.0
        self.is_first_start = True
        self.last_record_time_ms = None
        self.last_record_position = None
        self.last_record_yaw = None
        self.last_record_velocity = 0.0
        self.last_velocity_direction = None
        self.last_pose = None
        self.last_yaw = None

    def pose_cb(self, message):
        self.pose = message

    def gnss_fix_cb(self, message):
        self.gnss_fix = message

    @staticmethod
    def calculate_distance(first, second):
        if first is None or second is None:
            return float("inf")
        return math.hypot(first.x - second.x, first.y - second.y)

    @staticmethod
    def get_euler(orientation):
        return quaternion_to_euler(
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w,
        )

    def should_record(self, now_ms, position, yaw, linear_velocity):
        if (
            self.last_record_position is None
            or self.last_record_yaw is None
            or self.last_record_time_ms is None
        ):
            return True, False

        time_interval = (now_ms - self.last_record_time_ms) * 0.001
        distance = self.calculate_distance(position, self.last_record_position)
        yaw_difference = abs(yaw - self.last_record_yaw)
        if yaw_difference > math.pi:
            yaw_difference = 2.0 * math.pi - yaw_difference
        is_curve = yaw_difference > self.angular_threshold

        if distance / max(time_interval, 0.001) < 0.05:
            return False, is_curve
        if is_curve:
            return True, True
        should_record = (
            distance > self.linear_threshold
            or time_interval > self.min_straight_interval
            or abs(linear_velocity - self.last_record_velocity) > 0.1
        )
        return should_record, False

    def _warn_waiting_for_inputs(self, now_ns):
        if now_ns - self._last_wait_log_ns >= 5_000_000_000:
            self.get_logger().warning(
                "Waiting for gnss_pose and fix messages before recording"
            )
            self._last_wait_log_ns = now_ns

    def record_timer_cb(self):
        if not self.session.active:
            return

        now_ns = self.get_clock().now().nanoseconds
        if self.pose is None or self.gnss_fix is None:
            self._warn_waiting_for_inputs(now_ns)
            return

        now_ms = now_ns // 1_000_000
        position = self.pose.pose.position
        orientation = self.pose.pose.orientation
        roll, pitch, yaw = self.get_euler(orientation)

        velocity = 0.0
        angular_velocity = 0.0
        if self.last_pose is not None and self.last_record_time_ms is not None:
            distance = self.calculate_distance(self.last_pose, position)
            self.travelling_distance_of_this_path += distance
            elapsed = (now_ms - self.last_record_time_ms) * 0.001
            velocity = distance / elapsed if elapsed > 0.0 else 0.0
            delta_x = position.x - self.last_pose.x
            delta_y = position.y - self.last_pose.y
            displacement = math.hypot(delta_x, delta_y)
            if displacement > 0.0:
                direction_cosine = (
                    delta_x * math.cos(yaw) + delta_y * math.sin(yaw)
                ) / displacement
                velocity *= 1.0 if direction_cosine >= 0.0 else -1.0

        if self.last_velocity_direction is None and abs(velocity) < 0.02:
            self.last_pose = position
            self.last_record_time_ms = now_ms
            return

        if self.last_yaw is not None and self.last_record_time_ms is not None:
            yaw_difference = yaw - self.last_yaw
            if yaw_difference > math.pi:
                yaw_difference -= 2.0 * math.pi
            elif yaw_difference < -math.pi:
                yaw_difference += 2.0 * math.pi
            elapsed = (now_ms - self.last_record_time_ms) * 0.001
            angular_velocity = (
                yaw_difference / elapsed if elapsed > 0.0 else 0.0
            )

        _, is_curve = self.should_record(now_ms, position, yaw, velocity)

        velocity_direction = self.last_velocity_direction
        if abs(velocity) > 0.12:
            velocity_direction = yaw if velocity >= 0.0 else yaw + math.pi
            velocity_direction %= 2.0 * math.pi

        if self.travelling_distance_of_this_path > 10.0:
            self.current_path_id += 1
            self.travelling_distance_of_this_path = 0.0

        if self.is_first_start:
            self.last_record_time_ms = now_ms
            self.last_velocity_direction = None
            self.last_record_position = position
            self.last_record_yaw = yaw
            self.last_record_velocity = velocity
            self.last_pose = position
            self.last_yaw = yaw
            self.cumulative_distance = 0.0
            self.is_first_start = False
            return

        distance_from_previous = self.calculate_distance(
            position, self.last_record_position
        )
        self.cumulative_distance += distance_from_previous
        self.record_count += 1
        row = [
            self.record_count,
            self.current_path_id,
            now_ms,
            position.x,
            position.y,
            position.z,
            yaw,
            pitch,
            roll,
            velocity,
            velocity_direction,
            angular_velocity,
            1 if is_curve else 0,
            distance_from_previous,
            self.cumulative_distance,
            self.gnss_fix.latitude,
            self.gnss_fix.longitude,
        ]
        try:
            self.session.append(row)
        except (OSError, RuntimeError, ValueError) as error:
            self.get_logger().error(f"Failed to write record: {error}")
            self.session.cleanup()
            return

        self.last_velocity_direction = velocity_direction
        self.last_record_time_ms = now_ms
        self.last_record_position = position
        self.last_record_yaw = yaw
        self.last_record_velocity = velocity
        self.last_pose = position
        self.last_yaw = yaw

    def handle_record_mark_point_request(self, request, response):
        try:
            self.session.mark(request.mark_id)
        except (OSError, RuntimeError, ValueError) as error:
            response.success = False
            response.message = str(error)
            self.get_logger().error(f"Failed to record mark point: {error}")
        else:
            response.success = True
            response.message = f"Mark point {request.mark_id} recorded"
        return response

    def handle_process_switch_request(self, request, response):
        try:
            if request.data:
                record_path = self.session.start()
                self._reset_recording_state()
                self.current_map_name = None
                response.message = f"Recording to {record_path}"
            else:
                destination = self.session.stop(request.map_name)
                self.current_map_name = request.map_name or None
                response.message = (
                    f"Recording saved to {destination}"
                    if destination is not None
                    else "Recording stopped"
                )
        except (OSError, RuntimeError, ValueError) as error:
            response.success = False
            response.message = str(error)
            self.get_logger().error(f"Failed to change recording state: {error}")
        else:
            response.success = True
        return response

    def publish_record_status(self):
        message = RecordStatus()
        message.record_state = (
            1 if self.session.active else (2 if self.current_map_name else 0)
        )
        message.record_count = self.record_count
        message.path_id = self.current_path_id
        message.cumulative_distance = self.cumulative_distance
        message.map_name = self.current_map_name or ""
        self.record_status_pub.publish(message)

    def cleanup(self):
        self.session.cleanup()


def main(args=None):
    rclpy.init(args=args)
    node = LawnMowerRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cleanup()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
