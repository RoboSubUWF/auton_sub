#!/usr/bin/env python3
"""
DVL-aware keyboard control that doesn't require root privileges
Uses terminal input instead of global keyboard hooks
"""

import rclpy
from rclpy.node import Node
import threading
import time
import sys
import termios
import tty
import select

from auton_sub.motion.robot_control import RobotControl
from auton_sub.motion import servo
from auton_sub.utils import arm, disarm
from auton_sub.utils.guided import set_guided_mode


class DVLAwareKeyboardControlNode(Node):
    def __init__(self):
        super().__init__('dvl_aware_keyboard_control_node')
        
        self.rc = RobotControl()
        self.servo = servo

        self.forward = 0
        self.lateral = 0
        self.yaw = 0

        self.manual_control = False
        self.function_control = False
        self.flag = True
        self.data_lock = threading.Lock()
        
        # Store terminal settings
        self.old_settings = termios.tcgetattr(sys.stdin)
        
        # Always initialize movement thread
        self.movement_thread = threading.Thread(target=self.send_data_loop)
        self.movement_thread.daemon = True
        
        # Check system status before attempting modes
        self.check_system_status()

    def check_dvl_status(self):
        """Check if DVL is providing valid data"""
        self.get_logger().info("🌊 Checking DVL status...")
        
        # Subscribe to DVL topics to check data quality
        from geometry_msgs.msg import TwistWithCovarianceStamped
        
        self.dvl_data_received = False
        self.dvl_data_valid = False
        
        def dvl_callback(msg):
            self.dvl_data_received = True
            # Check if we have valid velocity data (not all zeros)
            vel = msg.twist.twist.linear
            if abs(vel.x) > 0.001 or abs(vel.y) > 0.001 or abs(vel.z) > 0.001:
                self.dvl_data_valid = True
            else:
                # Could still be valid if stationary - check covariance
                cov = msg.twist.covariance
                if cov[0] < 1000 and cov[7] < 1000:  # Reasonable covariance values
                    self.dvl_data_valid = True
        
        # Try to subscribe to common DVL topic names
        dvl_topics = [
            '/dvl/velocity',
            '/mavros/vision_speed/speed_twist', 
            '/auton_sub/dvl/velocity'
        ]
        
        subscription = None
        for topic in dvl_topics:
            try:
                subscription = self.create_subscription(
                    TwistWithCovarianceStamped,
                    topic,
                    dvl_callback,
                    10
                )
                self.get_logger().info(f"📡 Listening to {topic}")
                break
            except:
                continue
        
        if not subscription:
            self.get_logger().warn("⚠️  Could not find DVL velocity topic")
            return False
        
        # Wait for DVL data
        start_time = time.time()
        timeout = 3.0
        
        while time.time() - start_time < timeout and not self.dvl_data_received:
            rclpy.spin_once(self, timeout_sec=0.1)
        
        if self.dvl_data_received:
            if self.dvl_data_valid:
                self.get_logger().info("✅ DVL providing valid data")
                return True
            else:
                self.get_logger().warn("⚠️  DVL connected but no valid data (out of water?)")
                return False
        else:
            self.get_logger().warn("⚠️  No DVL data received")
            return False

    def check_mavros_connection(self):
        """Check if MAVROS is connected to flight controller"""
        self.get_logger().info("🔗 Checking MAVROS connection...")
        
        from mavros_msgs.msg import State
        
        self.mavros_connected = False
        self.current_mode = "UNKNOWN"
        self.is_armed = False
        
        def state_callback(msg):
            self.mavros_connected = msg.connected
            self.current_mode = msg.mode
            self.is_armed = msg.armed
        
        subscription = self.create_subscription(
            State,
            '/mavros/state',
            state_callback,
            10
        )
        
        # Wait for state message
        start_time = time.time()
        timeout = 3.0
        
        while time.time() - start_time < timeout and not hasattr(self, 'mavros_connected'):
            rclpy.spin_once(self, timeout_sec=0.1)
        
        if hasattr(self, 'mavros_connected'):
            if self.mavros_connected:
                self.get_logger().info(f"✅ MAVROS connected - Mode: {self.current_mode}, Armed: {self.is_armed}")
                return True
            else:
                self.get_logger().warn("⚠️  MAVROS not connected to flight controller")
                return False
        else:
            self.get_logger().warn("⚠️  No MAVROS state received")
            return False

    def set_manual_mode(self):
        """Set MANUAL mode for out-of-water testing"""
        from mavros_msgs.srv import SetMode
        
        self.get_logger().info("🔧 Setting MANUAL mode for out-of-water testing...")
        
        try:
            mode_client = self.create_client(SetMode, '/mavros/set_mode')
            
            if mode_client.wait_for_service(timeout_sec=3.0):
                request = SetMode.Request()
                request.custom_mode = "MANUAL"
                
                future = mode_client.call_async(request)
                rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
                
                if future.result() and future.result().mode_sent:
                    self.get_logger().info("✅ MANUAL mode set successfully")
                    return True
                else:
                    self.get_logger().warn("⚠️  Could not set MANUAL mode")
                    return False
            else:
                self.get_logger().warn("⚠️  MAVROS set_mode service not available")
                return False
        except Exception as e:
            self.get_logger().warn(f"Manual mode setting failed: {e}")
            return False

    def force_arm(self):
        """Force arm the vehicle with proper error handling"""
        self.get_logger().info("🔫 Arming vehicle...")
        try:
            arm.ArmerNode()
            time.sleep(3.0)  # Wait longer for arming to complete
            
            # Verify armed status
            rclpy.spin_once(self, timeout_sec=0.5)
            if self.is_armed:
                self.get_logger().info("✅ Vehicle armed successfully")
            else:
                self.get_logger().warn("⚠️  Arm command sent but status unclear")
                # Continue anyway - sometimes status takes time to update
        except Exception as e:
            self.get_logger().error(f"❌ Arming failed: {e}")
            # Don't return False - try to continue

    def check_system_status(self):
        """Check overall system status and decide what modes are available"""
        self.get_logger().info("🔍 Checking system status...")
        
        # Check MAVROS first
        mavros_ok = self.check_mavros_connection()
        
        # Check DVL
        dvl_ok = self.check_dvl_status()
        
        # Determine available modes
        if mavros_ok and dvl_ok:
            self.get_logger().info("🎉 Full system operational - attempting GUIDED mode")
            if set_guided_mode():
                self.get_logger().info("✅ GUIDED mode set successfully")
                time.sleep(2.0)  # Wait for mode to settle
                self.force_arm()
            else:
                self.get_logger().error("❌ Could not set GUIDED mode")
        
        elif mavros_ok and not dvl_ok:
            self.get_logger().warn("⚠️  DVL issues detected - switching to MANUAL mode")
            self.get_logger().warn("   Put DVL in water for position control")
            
            # Force MANUAL mode for out-of-water testing
            if self.set_manual_mode():
                # Wait for mode to change and settle
                time.sleep(2.0)
                
                # Check if we're actually in MANUAL mode
                rclpy.spin_once(self, timeout_sec=0.5)  # Update state
                if hasattr(self, 'current_mode') and self.current_mode == "MANUAL":
                    self.get_logger().info("✅ Confirmed in MANUAL mode")
                    self.get_logger().info("⚠️  WARNING: Thrusters will spin - be careful!")
                    
                    # Try to arm in MANUAL mode
                    if self.force_arm():
                        # Double-check armed status
                        time.sleep(1.0)
                        rclpy.spin_once(self, timeout_sec=0.5)
                        if hasattr(self, 'is_armed') and self.is_armed:
                            self.get_logger().info("✅ Successfully armed in MANUAL mode")
                        else:
                            self.get_logger().warn("⚠️  Arm command sent but status unclear")
                else:
                    self.get_logger().warn(f"⚠️  Mode is {getattr(self, 'current_mode', 'UNKNOWN')}, not MANUAL")
            else:
                self.get_logger().error("❌ Could not set MANUAL mode")
        
        elif not mavros_ok:
            self.get_logger().error("❌ MAVROS connection issues")
            self.get_logger().error("   Check flight controller connection")
            self.get_logger().warn("   Continuing in direct control mode...")
        
        else:
            self.get_logger().warn("⚠️  Multiple system issues - using direct control")

    def send_data_loop(self):
        while self.flag:
            with self.data_lock:
                self.rc.direct_input = [self.lateral, self.forward, self.yaw, 0, 0, 0]
                self.rc.mode = "direct"
            time.sleep(0.05)

    def get_char(self):
        """Get a single character from stdin without requiring Enter"""
        try:
            tty.setraw(sys.stdin.fileno())
            if select.select([sys.stdin], [], [], 0.1):
                ch = sys.stdin.read(1)
                return ch
            return None
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)

    def restore_terminal(self):
        """Restore terminal to normal mode"""
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)

    def start(self):
        print("\nRoboSub Keyboard Control (DVL-Aware)")
        print("====================================")
        
        while not self.manual_control and not self.function_control:
            setting = input("Type 'm' for manual control, 'f' for function control: ").strip()
            if setting == 'm':
                self.manual_control = True
            elif setting == 'f':
                self.function_control = True
            else:
                print("[ERROR] Invalid input.")

        self.movement_thread.start()

        try:
            if self.manual_control:
                self.manual_control_loop()
            elif self.function_control:
                self.function_control_loop()
        except KeyboardInterrupt:
            pass
        finally:
            self.flag = False
            self.restore_terminal()
            self.movement_thread.join()
            with self.data_lock:
                self.rc.direct_input = [0, 0, 0, 0, 0, 0]
                self.rc.mode = "direct"
            disarm.DisarmerNode()
            self.get_logger().info("Node shutdown complete.")

    def manual_control_loop(self):
        print("\n🎮 Manual Control Mode")
        print("======================")
        print("Controls:")
        print("  W/S: Forward/Backward")
        print("  A/D: Left/Right") 
        print("  Q/E: Rotate Left/Right")
        print("  SPACE: Emergency stop")
        print("  B: Stop and exit")
        print("\n⚠️  WARNING: If armed, thrusters will spin!")
        print("Hold keys and release to stop movement")
        print("Focus this terminal and press keys...\n")

        # Set terminal to raw mode for immediate key detection
        tty.setraw(sys.stdin.fileno())
        
        try:
            while self.manual_control:
                # Check for available input
                if select.select([sys.stdin], [], [], 0.1)[0]:
                    char = sys.stdin.read(1).lower()
                    
                    with self.data_lock:
                        if char == 'w':
                            self.forward = 0.5  # Reduced from 1.0 for safety
                            print("↑ Forward (50%)", end='\r')
                        elif char == 's':
                            self.forward = -0.5
                            print("↓ Backward (50%)", end='\r')
                        elif char == 'a':
                            self.lateral = -0.5
                            print("← Left (50%)", end='\r')
                        elif char == 'd':
                            self.lateral = 0.5
                            print("→ Right (50%)", end='\r')
                        elif char == 'q':
                            self.yaw = -0.5
                            print("↺ Rotate Left (50%)", end='\r')
                        elif char == 'e':
                            self.yaw = 0.5
                            print("↻ Rotate Right (50%)", end='\r')
                        elif char == ' ':  # Spacebar
                            self.forward = 0
                            self.lateral = 0
                            self.yaw = 0
                            print("⏹ Emergency Stop", end='\r')
                        elif char == 'b':
                            self.forward = 0
                            self.lateral = 0
                            self.yaw = 0
                            print("\n⏹ Exiting manual control")
                            self.manual_control = False
                            break
                        elif char == '\x03':  # Ctrl+C
                            print("\n⏹ Keyboard interrupt")
                            self.manual_control = False
                            break
                        else:
                            print(f"❌ Unknown key: {char}", end='\r')
                else:
                    # No input - gradually stop movement
                    with self.data_lock:
                        self.forward = 0
                        self.lateral = 0
                        self.yaw = 0
                        
        finally:
            # Always restore terminal
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)

    def function_control_loop(self):
        print("\n🔧 Function Control Mode")
        print("========================")
        print("Commands:")
        print("  fire <num>  - Fire torpedo (1-2)")
        print("  drop <num>  - Drop marker (1-2)")
        print("  open        - Open gripper")
        print("  close       - Close gripper")
        print("  exit        - Exit function mode")
        
        self.restore_terminal()  # Use normal terminal for function input
        
        while self.function_control:
            try:
                cmd = input("> ").strip().lower()
                if cmd.startswith("fire"):
                    parts = cmd.split()
                    if len(parts) == 2 and parts[1].isdigit():
                        num = int(parts[1])
                        self.servo.torpedo(num)
                        print(f"🚀 Fired torpedo {num}")
                    else:
                        print("❌ Usage: fire <number>")
                elif cmd.startswith("drop"):
                    parts = cmd.split()
                    if len(parts) == 2 and parts[1].isdigit():
                        num = int(parts[1])
                        self.servo.dropper(num)
                        print(f"💧 Dropped marker {num}")
                    else:
                        print("❌ Usage: drop <number>")
                elif cmd == "open":
                    self.servo.gripper(True)
                    print("✋ Gripper opened")
                elif cmd == "close":
                    self.servo.gripper(False)
                    print("👊 Gripper closed")
                elif cmd == "exit":
                    self.function_control = False
                    print("👋 Exiting function control")
                else:
                    print("❌ Unknown command")
            except KeyboardInterrupt:
                break
        self.function_control = False


def main(args=None):
    rclpy.init(args=args)
    node = DVLAwareKeyboardControlNode()
    
    try:
        node.start()
    finally:
        node.restore_terminal()
        
    rclpy.shutdown()


if __name__ == '__main__':
    main()