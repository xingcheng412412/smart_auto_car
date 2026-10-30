#!/usr/bin/env python3
"""
SocketCAN 总线监控/调试桥接工具（vehicle_core 本身直连 SocketCAN）。

订阅: /can_tx  (can_client/msg/CANFrame) -> 发送到 CAN 接口
发布: /can_rx  (can_client/msg/CANFrame) <- 从 CAN 接口读取
"""

import socket
import struct
import threading
import time

import rclpy
from rclpy.node import Node

from can_client.msg import CANFrame


CAN_RAW = 1
PF_CAN = 29

# struct can_frame: uint32 id, uint8 dlc, 3 bytes padding, uint8[8] data
CAN_FRAME_FMT = "=IB3x8s"
CAN_FRAME_SIZE = struct.calcsize(CAN_FRAME_FMT)

CAN_EFF_FLAG = 0x80000000
CAN_RTR_FLAG = 0x40000000
CAN_ERR_FLAG = 0x20000000


class SocketCanBridge(Node):
    def __init__(self):
        super().__init__('socketcan_bridge')

        self.declare_parameter('can_iface', 'can0')
        self.iface = self.get_parameter('can_iface').value

        self.sock = socket.socket(PF_CAN, socket.SOCK_RAW, CAN_RAW)
        self.sock.settimeout(0.5)
        self.sock.bind((self.iface,))
        self.get_logger().info(f'[socketcan_bridge] 绑定到 {self.iface}')

        self.pub = self.create_publisher(CANFrame, '/can_rx', 100)
        self.sub = self.create_subscription(
            CANFrame, '/can_tx', self.tx_callback, 100)

        self.running = True
        self.last_warning = {}
        self.rx_thread = threading.Thread(target=self.rx_loop, daemon=True)
        self.rx_thread.start()

    def warn_throttled(self, key, message, period=5.0):
        now = time.monotonic()
        if now - self.last_warning.get(key, 0.0) >= period:
            self.last_warning[key] = now
            self.get_logger().warning(message)

    def tx_callback(self, msg):
        """ROS /can_tx -> CAN 帧发送。"""
        can_id = msg.id
        if msg.is_extended:
            can_id |= CAN_EFF_FLAG
        dlc = min(int(msg.dlc), 8)
        data = bytes(list(msg.data)[:dlc]).ljust(8, b'\x00')
        frame = struct.pack(CAN_FRAME_FMT, can_id, dlc, data)
        try:
            self.sock.send(frame)
        except OSError as exc:
            if self.running:
                self.warn_throttled('tx', f'[socketcan_bridge] 发送失败: {exc}')

    def rx_loop(self):
        """CAN 帧接收 -> ROS /can_rx。"""
        while self.running and rclpy.ok():
            try:
                raw = self.sock.recv(CAN_FRAME_SIZE)
                if len(raw) < CAN_FRAME_SIZE:
                    continue
                can_id_raw, dlc, data = struct.unpack(CAN_FRAME_FMT, raw)

                msg = CANFrame()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.is_extended = bool(can_id_raw & CAN_EFF_FLAG)
                msg.id = can_id_raw & ~(
                    CAN_EFF_FLAG | CAN_RTR_FLAG | CAN_ERR_FLAG)
                msg.dlc = dlc
                msg.data = [data[i] if i < dlc else 0 for i in range(8)]
                self.pub.publish(msg)
            except socket.timeout:
                continue
            except OSError as exc:
                if self.running and rclpy.ok():
                    self.warn_throttled(
                        'rx', f'[socketcan_bridge] 接收异常: {exc}')

    def close(self):
        self.running = False
        self.sock.close()
        if self.rx_thread.is_alive():
            self.rx_thread.join(timeout=1.0)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = SocketCanBridge()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
