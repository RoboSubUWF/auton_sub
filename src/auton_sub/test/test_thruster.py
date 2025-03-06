import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import time

class TestThruster(Node):
    def __init__(self):
        super().__init__('test_thruster')

        # Publisher: Sends thruster commands to `/thruster_cmds`
        self.publisher_ = self.create_publisher(Float32MultiArray, '/thruster_cmds', 10)

        self.get_logger().info("✅ Thruster test node started!")
        self.run_test()

    def send_thruster_command(self, thruster_values):
        """Publishes a test thruster command."""
        msg = Float32MultiArray()
        msg.data = thruster_values
        self.publisher_.publish(msg)
        self.get_logger().info(f"🚀 Sent Thruster Command: {thruster_values}")

    def run_test(self):
        """Runs a sequence of thruster tests."""
        test_commands = [
            ([0.5, 0.0, 0.0, 0.0, 0.0, 0.0], "➡️ Moving Forward"),
            ([-0.5, 0.0, 0.0, 0.0, 0.0, 0.0], "⬅️ Moving Backward"),
            ([0.0, 0.5, 0.0, 0.0, 0.0, 0.0], "⬆️ Moving Left"),
            ([0.0, -0.5, 0.0, 0.0, 0.0, 0.0], "⬇️ Moving Right"),
            ([0.0, 0.0, 0.5, 0.0, 0.0, 0.0], "🔼 Moving Up"),
            ([0.0, 0.0, -0.5, 0.0, 0.0, 0.0], "🔽 Moving Down"),
            ([0.0, 0.0, 0.0, 0.5, 0.0, 0.0], "↩️ Turning Left"),
            ([0.0, 0.0, 0.0, -0.5, 0.0, 0.0], "↪️ Turning Right"),
            ([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], "🛑 Stopping Thrusters")
        ]

        for command, description in test_commands:
            self.get_logger().info(description)
            self.send_thruster_command(command)
            time.sleep(3)  # Wait for 3 seconds before next command

        self.get_logger().info("✅ Thruster test complete!")

def main(args=None):
    rclpy.init(args=args)
    node = TestThruster()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
