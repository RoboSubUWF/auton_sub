import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from pymavlink import mavutil

class ThrusterControl(Node):
    def __init__(self):
        super().__init__('thruster_control')

        # MAVLink Connection (Ensure Pixhawk is connected via serial)
        self.master = mavutil.mavlink_connection('/dev/ttyTHS1', baud=57600)

        # Subscribe to thruster command topic
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/thruster_cmds',
            self.send_thruster_command,
            10)
        
        self.get_logger().info("✅ Thruster control node started!")

    def send_thruster_command(self, msg):
        """Sends received thrust values to Pixhawk using MAVLink."""
        thruster_values = msg.data  # Expecting 6 values for 6 thrusters

        if len(thruster_values) != 6:
            self.get_logger().error("❌ Invalid thruster command. Expected 6 values, got: " + str(len(thruster_values)))
            return
        
        self.get_logger().info(f"🚀 Sending Thruster Command: {thruster_values}")

        # Convert thrust commands into MAVLink `manual_control_send` message
        self.master.mav.manual_control_send(
            self.master.target_system,
            int(thruster_values[0] * 1000),  # Forward/Backward (X-axis)
            int(thruster_values[1] * 1000),  # Left/Right (Y-axis)
            int(thruster_values[2] * 1000),  # Up/Down (Z-axis)
            int(thruster_values[3] * 1000),  # Yaw rotation
            0  # No button press
        )

def main(args=None):
    rclpy.init(args=args)
    node = ThrusterControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
