import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from auton_sub.motion import robot_control  # For controlling motors


class StraightLeftMission(Node):
    def __init__(self):
        super().__init__('straight_left_mission')

        self.robot_control = robot_control.RobotControl()
        self.get_logger().info("[INFO] Straight Left Mission Node Initialized")

        # Parameters (can be dynamically configured later if needed)
        self.forward_duration = 15.0  # seconds
        self.pause_time = 1.0         # seconds to pause between steps
        self.turn_duration = 3.0      # seconds to rotate 90 degrees left
        self.turn_speed = 0.5         # yaw command value for turning

    def run(self):
        self.get_logger().info("[INFO] Starting Straight Left Mission")

        try:
            # Step 1: Go forward 30 ft (~15 seconds)
            self.robot_control.movement(forward=0.5)
            self.get_logger().info("[MOTION] Moving forward 30 ft")
            self.sleep(self.forward_duration)

            self.robot_control.movement(forward=0.0)
            self.sleep(self.pause_time)

            # Step 2: Turn left 90 degrees
            self.robot_control.movement(yaw=self.turn_speed)
            self.get_logger().info("[MOTION] Turning left")
            self.sleep(self.turn_duration)

            self.robot_control.movement(yaw=0.0)
            self.sleep(self.pause_time)

            # Step 3: Go forward 1 ft (~0.5 second)
            self.robot_control.movement(forward=0.5)
            self.get_logger().info("[MOTION] Moving forward 1 ft")
            self.sleep(0.5)

            self.robot_control.movement(forward=0.0)
            self.sleep(self.pause_time)

            # Step 4: Turn left 90 degrees again
            self.robot_control.movement(yaw=self.turn_speed)
            self.get_logger().info("[MOTION] Turning left again")
            self.sleep(self.turn_duration)

            self.robot_control.movement(yaw=0.0)
            self.sleep(self.pause_time)

            # Step 5: Go forward 30 ft (~15 seconds)
            self.robot_control.movement(forward=0.5)
            self.get_logger().info("[MOTION] Moving forward final 30 ft")
            self.sleep(self.forward_duration)

            self.robot_control.movement(forward=0.0)
            self.get_logger().info("[INFO] Mission Complete. Submarine stopped.")

        except KeyboardInterrupt:
            self.get_logger().info("[INFO] Mission interrupted")
        finally:
            self.robot_control.movement(forward=0.0, yaw=0.0)
            self.get_logger().info("[INFO] Robot stopped. Exiting mission")

    def sleep(self, seconds):
        end_time = self.get_clock().now().seconds_nanoseconds()[0] + seconds
        while rclpy.ok() and self.get_clock().now().seconds_nanoseconds()[0] < end_time:
            rclpy.spin_once(self, timeout_sec=0.1)


def main(args=None):
    rclpy.init(args=args)
    node = StraightLeftMission()
    node.run()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
