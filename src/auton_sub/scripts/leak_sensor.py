#is not currently integrated, but works great and ready to be integrated at a later time. 
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import Jetson.GPIO as GPIO  # Use Jetson's GPIO library
import time

# Define the GPIO pin connected to the leak sensor
LEAK_SENSOR_PIN = 12  #Actual pin 32
# GPIO setup



print("Starting leak sensor monitoring. Press Ctrl+C to exit.")

class LeakSensor(Node):
    def __init__(self):
        super().__init__('leak_sensor')

        # Set up ROS 2 publisher
        self.publisher_ = self.create_publisher(Bool, '/leak_detected', 10)

        # Initialize GPIO for Jetson
        GPIO.setmode(GPIO.BCM)  # Use Broadcom pin numbering
        GPIO.setup(LEAK_SENSOR_PIN, GPIO.IN)

        # Create a timer to check for leaks every second
        self.timer = self.create_timer(1.0, self.check_leak)

        self.get_logger().info("✅ Leak sensor node started on Jetson Orin Nano!")

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
    node = LeakSensor()

    try:
        rclpy.spin(node)  # Run the node
    except KeyboardInterrupt:
        node.get_logger().info("🛑 Leak sensor node shutting down...")
    finally:
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
