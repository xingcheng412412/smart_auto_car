import curses
import threading

import rclpy

from lawn_mower_console.console_node import ConsoleNode
from lawn_mower_console.tui import run_tui


def main(args=None):
    rclpy.init(args=args)
    node = ConsoleNode()
    spin_thread = threading.Thread(
        target=rclpy.spin, args=(node,), daemon=True
    )
    spin_thread.start()
    try:
        curses.wrapper(run_tui, node, node.state)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        spin_thread.join(timeout=1.0)


if __name__ == "__main__":
    main()
