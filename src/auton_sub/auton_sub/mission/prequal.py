import time
import rclpy
from rclpy.node import Node
from auton_sub.utils import arm, disarm
from auton_sub.utils.guided import set_guided_mode


from auton_sub.motion.robot_control import RobotControl  # Updated import to match your structure


class StraightLeftMission(Node):
    def __init__(self):
        super().__init__('straight_left_mission')

        self.robot_control = RobotControl()
        self.get_logger().info("[INFO] Straight Left Mission Node Initialized")

        # Parameters (can be dynamically configured later if needed)
        self.forward_duration = 15.0  # seconds
        self.pause_time = 1.0         # seconds to pause between steps
        self.turn_duration = 3.0      # seconds to rotate 90 degrees left
        self.forward_speed = 0.5      # forward movement speed
        self.turn_speed = 0.5         # yaw command value for turning

    def run(self):
        self.get_logger().info("[INFO] Starting Straight Left Mission")
        arm.ArmerNode()
        set_guided_mode()
        try:
            # Set mode to direct control
            self.robot_control.mode = "direct"
            
            # Step 1: Go forward 30 ft (~15 seconds)
            with self.robot_control.lock:
                self.robot_control.direct_input = [0.0, self.forward_speed, 0.0, 0.0, 0.0, 0.0]
            self.get_logger().info("[MOTION] Moving forward 30 ft")
            self.sleep(self.forward_duration)

            # Stop movement
            with self.robot_control.lock:
                self.robot_control.direct_input = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            self.sleep(self.pause_time)

            # Step 2: Turn left 90 degrees
            with self.robot_control.lock:
                self.robot_control.direct_input = [0.0, 0.0, self.turn_speed, 0.0, 0.0, 0.0]
            self.get_logger().info("[MOTION] Turning left")
            self.sleep(self.turn_duration)

            # Stop turning
            with self.robot_control.lock:
                self.robot_control.direct_input = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            self.sleep(self.pause_time)

            # Step 3: Go forward 1 ft (~0.5 second)
            with self.robot_control.lock:
                self.robot_control.direct_input = [0.0, self.forward_speed, 0.0, 0.0, 0.0, 0.0]
            self.get_logger().info("[MOTION] Moving forward 1 ft")
            self.sleep(0.5)

            # Stop movement
            with self.robot_control.lock:
                self.robot_control.direct_input = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            self.sleep(self.pause_time)

            # Step 4: Turn left 90 degrees again
            with self.robot_control.lock:
                self.robot_control.direct_input = [0.0, 0.0, self.turn_speed, 0.0, 0.0, 0.0]
            self.get_logger().info("[MOTION] Turning left again")
            self.sleep(self.turn_duration)

            # Stop turning
            with self.robot_control.lock:
                self.robot_control.direct_input = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            self.sleep(self.pause_time)

            # Step 5: Go forward 30 ft (~15 seconds)
            with self.robot_control.lock:
                self.robot_control.direct_input = [0.0, self.forward_speed, 0.0, 0.0, 0.0, 0.0]
            self.get_logger().info("[MOTION] Moving forward final 30 ft")
            self.sleep(self.forward_duration)

            # Final stop
            with self.robot_control.lock:
                self.robot_control.direct_input = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            self.get_logger().info("[INFO] Mission Complete. Submarine stopped.")

        except KeyboardInterrupt:
            self.get_logger().info("[INFO] Mission interrupted")
        finally:
            with self.robot_control.lock:
                self.robot_control.direct_input = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            self.get_logger().info("[INFO] Robot stopped. Exiting mission")

    def sleep(self, seconds):
        """Sleep while allowing ROS to continue spinning"""
        end_time = time.time() + seconds
        while rclpy.ok() and time.time() < end_time:
            rclpy.spin_once(self, timeout_sec=0.1)

    def cleanup(self):
        """Clean up the mission by stopping movement"""
        with self.robot_control.lock:
            self.robot_control.direct_input = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.robot_control.stop()
        self.get_logger().info("[INFO] Straight Left mission cleanup complete")


def main(args=None):
    rclpy.init(args=args)
    node = StraightLeftMission()
    
    try:
        node.run()
        disarm.DisarmerNode()
    finally:
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()