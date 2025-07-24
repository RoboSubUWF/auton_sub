import json
import time
import rclpy
from rclpy.node import Node

from auton_sub.sensors.cv_handler import CVHandler
from auton_sub.motion.robot_control import RobotControl
from auton_sub.utils import arm, disarm


class OctagonApproachMission(Node):
    def __init__(self, target=None, **config):
        super().__init__('octagon_approach_mission')
        self.config = config
        self.target = target
        self.robot_control = RobotControl()
        self.cv_handler = CVHandler(**self.config)

        self.cv_files = ["octagon_approach_cv"]
        self.data = {}
        self.next_data = {}
        self.received = False

        for file in self.cv_files:
            self.cv_handler.start_cv(
                mission_name=file,
                module_path=f"auv.cv.{file}",
                camera_topic="/camera/front/image_raw",  # Start with front camera
                callback=self.callback
            )

        self.robot_control.set_depth(0.38)  # Initial dive
        self.get_logger().info("[MISSION] Octagon Approach initialized.")

    def callback(self, msg):
        """
        Handles CV data callback from camera vision system.
        """
        file_name = msg._connection_header["topic"].split("/")[-1]
        data = json.loads(msg.data)
        self.next_data[file_name] = data
        self.received = True

    def merge_data(self):
        """
        Merges the latest CV handler data.
        """
        for key in self.next_data:
            if key in self.data:
                self.data[key].update(self.next_data[key])
            else:
                self.data[key] = self.next_data[key]
        self.received = False
        self.next_data = {}

    def run(self):
        self.get_logger().info("[MISSION] Running Octagon Approach...")
        centering_complete = False
        surface_complete = False

        while rclpy.ok():
            rclpy.spin_once(self)

            if not self.received:
                continue

            self.merge_data()
            cv_data = self.data["octagon_approach_cv"]

            lateral = cv_data.get("lateral", 0.0)
            forward = cv_data.get("forward", 0.0)
            yaw = cv_data.get("yaw", 0.0)
            vertical = cv_data.get("vertical", 0.0)
            centered = cv_data.get("centered", False)
            end = cv_data.get("end", False)

            if end:
                self.get_logger().info("[MISSION] CV finished early.")
                break

            if not centering_complete:
                self.robot_control.movement(forward=forward, lateral=lateral, yaw=yaw, vertical=vertical)
                if centered:
                    self.get_logger().info("[MISSION] Centered above table. Switching to vertical ascent.")
                    centering_complete = True
                    self.robot_control.movement(0, 0, 0, 0)
                    time.sleep(1)

            elif not surface_complete:
                self.robot_control.ascend(feet=5)  # You can replace with set_depth if needed
                surface_complete = True
                self.get_logger().info("[MISSION] Surfaced above platform. Beginning picture rotation.")

            else:
                self.robot_control.movement(yaw=0.5)  # Begin slow spin to search for pictures
                # Could be extended with more vision logic here
                time.sleep(5)
                break

        self.robot_control.movement(0, 0, 0, 0)
        self.get_logger().info("[MISSION] Octagon Approach complete.")

    def cleanup(self):
        for file in self.cv_files:
            self.cv_handler.stop_cv(file)
        self.robot_control.movement(0, 0, 0, 0)
        self.get_logger().info("[MISSION] Cleanup complete.")


def main(args=None):
    rclpy.init(args=args)
    node = OctagonApproachMission()

    arm.arm()
    node.run()
    node.cleanup()
    disarm.disarm()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
