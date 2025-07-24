#!/usr/bin/env python3
# not sure if this is important but here if needed later. Dont use now.
import os
import signal
import subprocess
import threading
import time
from datetime import datetime

import rclpy
from rclpy.node import Node


class RosbagRecorder(Node):
    def __init__(self):
        super().__init__('rosbag_recorder')

        # Create filename with timestamp
        now = datetime.now()
        filename = now.strftime("%Y-%m-%d_%H-%M-%S")
        self.output_dir = f"/home/robosub/rosbags/{filename}" #change later

        # Create directory if not exists
        os.makedirs("/home/robosub/rosbags", exist_ok=True)

        # Topics to record
        self.topics = [
            "/auv/devices/compass",
            "/auv/devices/imu",
            "/auv/devices/baro",
            "/auv/devices/thrusters",
            "/auv/devices/setDepth",
            "/auv/status/arm",
            "/auv/status/mode"
        ]

        self.get_logger().info(f"📦 Starting rosbag2 recording to: {self.output_dir}")
        self.get_logger().info(f"🛰️ Topics: {', '.join(self.topics)}")

        # Start rosbag record as subprocess
        self.rosbag_proc = subprocess.Popen([
            "ros2", "bag", "record", "-o", self.output_dir, *self.topics
        ])

        # Register shutdown hook
        signal.signal(signal.SIGINT, self.shutdown_handler)
        signal.signal(signal.SIGTERM, self.shutdown_handler)

    def shutdown_handler(self, signum, frame):
        self.get_logger().info("🛑 Shutting down rosbag recording...")
        try:
            self.rosbag_proc.send_signal(signal.SIGINT)
            self.rosbag_proc.wait(timeout=5)
            self.get_logger().info(f"✅ Bag saved to {self.output_dir}")
        except Exception as e:
            self.get_logger().error(f"⚠️ Error during shutdown: {str(e)}")
        finally:
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = RosbagRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
