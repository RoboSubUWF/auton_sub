import rclpy
from rclpy.node import Node
from pymavlink import mavutil  # MAVLink to communicate with Pixhawk
import time

class GuidedNavigation(Node):
    def __init__(self):
        super().__init__('navigation')

        # ✅ Connect to Pixhawk via MAVLink
        self.master = mavutil.mavlink_connection('/dev/ttyTHS1', baud=57600)

        # ✅ Wait for heartbeat to confirm connection
        self.get_logger().info("🔄 Waiting for heartbeat from Pixhawk...")
        self.master.wait_heartbeat()
        self.get_logger().info("✅ Heartbeat received! Connected to Pixhawk.")

        # ✅ Arm the submarine
        self.arm_ardusub()

        # ✅ Set GUIDED mode
        self.set_guided_mode()

        # ✅ Set initial waypoint
        self.target_position = [2.0, -1.0, -3.0]  # Example X, Y, Z target

        # ✅ Start navigation loop (every 2 seconds)
        self.timer = self.create_timer(2.0, self.navigate_to_target)

        self.get_logger().info("✅ GUIDED navigation started!")

    def arm_ardusub(self):
        """Arms the submarine to allow movement."""
        self.get_logger().info("🛠️ Arming ArduSub...")
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1, 0, 0, 0, 0, 0, 0  # 1 = Arm, 0 = Disarm
        )
        self.get_logger().info("✅ ArduSub armed!")

    def set_guided_mode(self):
        """Sets the Pixhawk to GUIDED mode for waypoint navigation."""
        self.get_logger().info("🛠️ Setting GUIDED mode...")
        self.master.mav.set_mode_send(
            self.master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            4  # 4 = GUIDED mode in ArduSub
        )
        self.get_logger().info("✅ GUIDED mode activated!")

    def navigate_to_target(self):
        """Commands the Pixhawk to move to the target waypoint."""
        x, y, z = self.target_position
        self.get_logger().info(f"📍 Navigating to target: {self.target_position}")

        self.master.mav.set_position_target_local_ned_send(
            0,  # time_boot_ms (not used)
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            0b110111111000,  # Ignore velocity and acceleration, use position
            x, y, z,  # Target position in meters (relative to home)
            25, 25, 25,  # No velocity control
            0, 0, 0,  # No acceleration control
            0, 4  # No yaw control
        )

        self.get_logger().info("🚀 Moving to waypoint...")

def main(args=None):
    rclpy.init(args=args)
    node = GuidedNavigation()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
