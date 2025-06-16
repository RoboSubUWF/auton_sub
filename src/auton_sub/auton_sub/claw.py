import rclpy
from rclpy.node import Node
from pymavlink import mavutil
import time

class ClawController(Node):
    def __init__(self):
        super().__init__('claw_controller')
        self.get_logger().info("Connecting to Pixhawk for claw control...")
        self.master = mavutil.mavlink_connection('/dev/ttyTHS1', baud=115200)
        self.master.wait_heartbeat()
        self.get_logger().info("Connected to Pixhawk.")
        self.operate_claw()

    def operate_claw(self):
        while rclpy.ok():
            self.get_logger().info("Opening claw...")
            self.set_motor_speed(500)  # Adjust PWM value for "open" position
            time.sleep(2)  # Adjust time as necessary for full movement

            self.get_logger().info("Closing claw...")
            self.set_motor_speed(-500)  # Adjust PWM value for "closed" position
            time.sleep(2)  # Adjust time as necessary for full movement

    def set_motor_speed(self, pwm_value):
        """Send PWM command to Pixhawk to control the claw motor."""
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
            0,
            9,        # Servo number (adjust based on Pixhawk configuration)
            pwm_value, # PWM value
            0, 0, 0, 0, 0
        )

def main(args=None):
    rclpy.init(args=args)
    node = ClawController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
