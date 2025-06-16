import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from pymavlink import mavutil

class ThrusterNode(Node):
    def __init__(self):
        super().__init__('thruster_node')

        # ✅ Connect to Pixhawk via MAVLink
        self.master = mavutil.mavlink_connection('/dev/ttyTHS1', baud=57600)

        # ✅ Wait for a heartbeat to confirm connection
        self.get_logger().info("🔄 Waiting for heartbeat from Pixhawk...")
        self.master.wait_heartbeat()
        self.get_logger().info("✅ Heartbeat received! Connected to Pixhawk.")

        # ✅ Arm the vehicle
        self.arm_ardusub()

        # ✅ Set to MANUAL mode
        self.set_manual_mode()

        # ✅ Subscribe to thruster command topic
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/thruster_cmds',
            self.send_thruster_command,
            10)

        self.get_logger().info("✅ Thruster node ready! Listening for thruster commands...")

    def arm_ardusub(self):
        """Arms the Pixhawk for thruster operation."""
        self.get_logger().info("🛠️ Arming ArduSub...")
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1, 0, 0, 0, 0, 0, 0  # 1 = Arm, 0 = Disarm
        )
        self.get_logger().info("✅ ArduSub armed!")

    def set_manual_mode(self):
        """Sets Pixhawk to MANUAL mode (required for thruster control)."""
        self.get_logger().info("🛠️ Setting MANUAL mode...")
        self.master.mav.set_mode_send(
            self.master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            19  # 19 = MANUAL mode in ArduSub
        )
        self.get_logger().info("✅ MANUAL mode activated!")

    def send_thruster_command(self, msg):
        """Receives thruster commands from ROS 2 and sends them to Pixhawk via MAVLink."""
        thruster_values = msg.data  # Expecting 6 values for 6 thrusters

        if len(thruster_values) != 6:
            self.get_logger().error("❌ Invalid thruster command. Expected 6 values, got: " + str(len(thruster_values)))
            return

        self.get_logger().info(f"🚀 Sending Thruster Command: {thruster_values}")

        # ✅ Convert thrust commands into MAVLink `manual_control_send` message
        self.master.mav.manual_control_send(
            target=1,
            x=int(thruster_values[0] * 1000),  # Forward/Backward (X-axis)
            y=int(thruster_values[1] * 1000),  # Left/Right (Y-axis)
            z=int(thruster_values[2] * 1000),  # Up/Down (Z-axis)
            r=int(thruster_values[3] * 1000),  # Yaw rotation
            buttons=0  # No button press
        )

def main(args=None):
    rclpy.init(args=args)
    node = ThrusterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
