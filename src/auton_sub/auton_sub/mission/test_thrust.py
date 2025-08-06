import time
import rclpy
from rclpy.node import Node
from mavros_msgs.msg import OverrideRCIn

from auton_sub.utils import arm, disarm
from auton_sub.utils.guided import set_guided_mode

# DON'T import RobotControl - we want direct thruster control
# from auton_sub.motion.robot_control import RobotControl


class DescentTestMission(Node):
    def __init__(self):
        super().__init__('descent_test_mission')

        # DON'T create RobotControl instance - it interferes with our direct control
        # self.robot_control = RobotControl()
        
        self.get_logger().info("[INFO] Descent Test Mission Node Initialized")

        # Test parameters
        self.descent_duration = 5.0  # seconds to descend
        self.throttle_channel = 3  # Channel 3 in ArduSub docs = array index 2 (0-indexed)
        
        # PWM parameters (from robot_control.py)
        self.pwm_neutral = 1500
        self.pwm_max = 1900  # Max upward throttle
        self.pwm_min = 1100  # Max downward throttle
        
        # Descent PWM (adjust this value to control descent speed)
        # 1400 = gentle descent, 1300 = moderate descent, 1200 = faster descent
        self.descent_pwm = 1100  # Moderate descent speed

        # Create direct PWM publisher for precise control
        self.throttle_pub = self.create_publisher(OverrideRCIn, '/mavros/rc/override', 10)

    def send_neutral_pwm(self):
        """Send neutral PWM to all channels"""
        pwm = OverrideRCIn()
        pwm.channels = [self.pwm_neutral] * 18
        self.throttle_pub.publish(pwm)
        self.get_logger().info("[PWM] All channels set to neutral")

    def descend(self, duration):
        """Descend for specified duration"""
        self.get_logger().info(f"[DESCENT] Starting descent at PWM {self.descent_pwm} for {duration} seconds")
        
        start_time = time.time()
        
        while (time.time() - start_time) < duration:
            # Create PWM message with all channels neutral except throttle
            pwm = OverrideRCIn()
            pwm.channels = [self.pwm_neutral] * 18
            
            # Set throttle channel to descent PWM
            pwm.channels[self.throttle_channel] = self.descent_pwm
            
            # Publish the command
            self.throttle_pub.publish(pwm)
            
            elapsed = time.time() - start_time
            remaining = duration - elapsed
            
            # Log progress every second
            if int(elapsed) != int(elapsed - 0.1):  # Log once per second
                self.get_logger().info(f"[DESCENT] Descending... {remaining:.1f}s remaining")
            
            self.sleep(0.1)  # 10Hz update rate
        
        # Stop descent
        self.send_neutral_pwm()
        self.get_logger().info(f"[DESCENT] ✅ Descent complete, returning to neutral")

    def run(self):
        self.get_logger().info("[INFO] Starting Descent Test Mission")
        
        try:
            # Step 1: Arm the vehicle using imported utility
            self.get_logger().info("[INFO] Arming vehicle...")
            arm_node = arm.ArmerNode()
            # Give some time for arming to complete
            self.sleep(2.0)
            
            # Step 2: Set GUIDED mode using imported utility
            self.get_logger().info("[INFO] Setting GUIDED mode...")
            if not set_guided_mode():
                self.get_logger().error("[ERROR] Failed to set GUIDED mode")
                return False
            
            # Step 3: Wait for systems to be ready
            self.get_logger().info("[INFO] Waiting for systems to be ready...")
            self.sleep(2.0)
            
            # Step 4: Ensure all channels start at neutral
            self.send_neutral_pwm()
            self.sleep(1.0)
            
            # Step 5: Perform descent test
            self.get_logger().info("[TEST] Beginning descent test...")
            self.descend(self.descent_duration)
            
            # Step 6: Wait a moment at neutral
            self.get_logger().info("[TEST] Holding position for 2 seconds...")
            self.sleep(10.0)
            
            # Step 7: Final safety check - ensure all channels are neutral
            self.send_neutral_pwm()
            self.sleep(1.0)
            
            self.get_logger().info("[INFO] ✅ Descent test completed successfully!")
            return True

        except KeyboardInterrupt:
            self.get_logger().info("[INFO] Descent test interrupted")
            return False
        except Exception as e:
            self.get_logger().error(f"[ERROR] Descent test failed: {e}")
            return False
        finally:
            # Ensure all channels are neutral
            self.send_neutral_pwm()
            self.get_logger().info("[INFO] All channels set to neutral.")

    def sleep(self, seconds):
        """Sleep while allowing ROS to continue spinning"""
        end_time = time.time() + seconds
        while rclpy.ok() and time.time() < end_time:
            rclpy.spin_once(self, timeout_sec=0.1)

    def cleanup(self):
        """Clean up the mission by ensuring all channels are neutral"""
        self.send_neutral_pwm()
        self.get_logger().info("[INFO] Descent test cleanup complete")


def main(args=None):
    rclpy.init(args=args)
    node = DescentTestMission()
    
    success = False
    try:
        success = node.run()
        
        if success:
            node.get_logger().info("[INFO] 🏁 Descent test completed successfully!")
        else:
            node.get_logger().error("[ERROR] 🚫 Descent test failed!")
            
    except Exception as e:
        node.get_logger().error(f"[ERROR] Exception occurred: {e}")
    finally:
        # Disarm the vehicle using imported utility
        node.get_logger().info("[INFO] Disarming vehicle...")
        try:
            disarm_node = disarm.DisarmerNode()
            time.sleep(2.0)  # Give time for disarming to complete
            node.get_logger().info("[INFO] ✅ Vehicle disarmed")
        except Exception as e:
            node.get_logger().error(f"[ERROR] Failed to disarm: {e}")
        
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
