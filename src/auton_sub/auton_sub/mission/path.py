import json
import time

import rclpy
from rclpy.node import Node

from auton_sub.sensors.cv_handler import CVHandler
from auton_sub.motion.robot_control import RobotControl
from auton_sub.utils import arm, disarm


class PathMission(Node):
    """
    ROS 2 class to handle the Follow-the-Path mission without deviceHelper.
    """
    def __init__(self):
        super().__init__('path_mission')

        self.cv_files = ["path_cv"]
        self.data = {}
        self.next_data = {}
        self.received = False
        self.path_compass_heading = None

        self.robot_control = RobotControl()

        # Direct configuration instead of deviceHelper
        self.config = {
            "camera_topic": "/camera/front/image_raw",
            # "cv_dummy": ["/path/to/test_video.mp4"]  # Uncomment if using a dummy video
        }

        self.cv_handler = CVHandler(**self.config)

        for file in self.cv_files:
            self.cv_handler.start_cv(
                mission_name=file,
                module_path=f"auv.cv.{file}",
                camera_topic=self.config["camera_topic"],
                callback=self.callback
            )

        self.get_logger().info("[MISSION] PathMission initialized")

    def callback(self, msg):
        try:
            file_name = msg._connection_header["topic"].split("/")[-1]
        except AttributeError:
            file_name = "path_cv"

        data = json.loads(msg.data)
        self.next_data[file_name] = data
        self.received = True

    def run(self):
        self.get_logger().info("[MISSION] Running PathMission")

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)

            if not self.received:
                continue

            for key in self.next_data:
                if key in self.data:
                    self.data[key].update(self.next_data[key])
                else:
                    self.data[key] = self.next_data[key]
            self.received = False
            self.next_data = {}

            if "path_cv" not in self.data:
                continue

            try:
                if self.data["path_cv"].get("end", False):
                    self.robot_control.movement()
                    break

                yaw = self.data["path_cv"].get("yaw", 0.0)
                forward = self.data["path_cv"].get("forward", 0.0)
                lateral = self.data["path_cv"].get("lateral", 0.0)

                self.get_logger().info(f"[MOTION] Yaw: {yaw}, Fwd: {forward}, Lat: {lateral}")
                self.robot_control.movement(yaw=yaw, forward=forward, lateral=lateral)

            except Exception as e:
                self.get_logger().error(f"[ERROR] {e}")
                self.robot_control.movement()
                break

        self.path_compass_heading = self.robot_control.get_heading()
        self.get_logger().info(f"[DEBUG] Final heading: {self.path_compass_heading}")

        self.robot_control.forward_dvl(distance=2)
        self.get_logger().info("[MISSION] PathMission complete")

    def cleanup(self):
        for file in self.cv_files:
            self.cv_handler.stop_cv(file)

        self.robot_control.movement()
        self.get_logger().info("[MISSION] PathMission cleanup complete")


def main(args=None):
    rclpy.init(args=args)

    mission = PathMission()

    arm.arm()

    mission.run()
    mission.cleanup()

    disarm.disarm()

    mission.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
