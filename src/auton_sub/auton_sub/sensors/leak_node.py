import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import Jetson.GPIO as GPIO  # Use Jetson's GPIO library

# Define the GPIO pin connected to the leak sensor
LEAK_SENSOR_PIN = 12  # actual pin 32

class LeakNode(Node):
    def __init__(self):
        super().__init__('leak_node')

        # Set up ROS 2 publisher
        self.publisher_ = self.create_publisher(Bool, '/leak_detected', 10)

        # Initialize GPIO for Jetson
        GPIO.setmode(GPIO.BCM)  # Use Broadcom pin numbering
        GPIO.setup(LEAK_SENSOR_PIN, GPIO.IN)

        # Create a timer to check for leaks every second
        self.timer = self.create_timer(1.0, self.check_leak)

        self.get_logger().info("✅ Leak detection node started on Jetson Orin Nano!")

    def check_leak(self):
        """Reads the leak sensor and publishes the status."""
        leak_status = GPIO.input(LEAK_SENSOR_PIN)  # Read GPIO pin state
        msg = Bool()
        msg.data = bool(leak_status)
        self.publisher_.publish(msg)

        if leak_status:
            self.get_logger().warn("🚨 LEAK DETECTED! Take immediate action!")
        else:
            self.get_logger().info("✅ No leaks detected.")

    def cleanup(self):
        """Ensure GPIO is cleaned up on shutdown."""
        GPIO.cleanup()
        self.get_logger().info("🔄 GPIO cleaned up.")

def main(args=None):
    rclpy.init(args=args)
    node = LeakNode()

    try:
        rclpy.spin(node)  # Run the node
    except KeyboardInterrupt:
        node.get_logger().info("🛑 Leak detection node shutting down...")
    finally:
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
