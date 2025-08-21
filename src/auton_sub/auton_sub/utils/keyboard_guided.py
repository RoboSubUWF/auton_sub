#!/usr/bin/env python3
"""
ArduSub GUIDED mode keyboard control with DVL position control
Uses setpoint commands for position/velocity control
again hasnt ever worked so idk
"""

import rclpy
from rclpy.node import Node
import threading
import time
import sys
import termios
import tty
import select
import math

from geometry_msgs.msg import PoseStamped, TwistStamped, Point
from mavros_msgs.msg import State, PositionTarget
from mavros_msgs.srv import CommandBool, SetMode
from auton_sub.motion import servo
from auton_sub.utils import arm, disarm
from auton_sub.utils.guided import set_guided_mode


class GuidedModeKeyboardControl(Node):
    def __init__(self):
        super().__init__('guided_keyboard_control_node')
        
        # Control variables
        self.current_position = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        self.current_orientation = {'yaw': 0.0}
        self.target_position = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'yaw': 0.0} # so if T is pressed the sub returns to OG position
        
        # Control increments
        self.position_step = 0.5  # meters
        self.yaw_step = 0.2       # radians (~11 degrees)
        self.depth_step = 0.3     # meters
        
        # Control modes
        self.manual_control = False
        self.function_control = False
        self.position_control = True  # Default for GUIDED mode
        self.velocity_control = False
        
        self.flag = True
        self.data_lock = threading.Lock()
        
        # Store terminal settings
        self.old_settings = termios.tcgetattr(sys.stdin)
        
        # System status
        self.mavros_connected = False
        self.current_mode = "UNKNOWN"
        self.is_armed = False
        self.dvl_available = False
        
        # Publishers for GUIDED mode control
        self.position_pub = self.create_publisher(
            PoseStamped, 
            '/mavros/setpoint_position/local', 
            10
        )
        
        self.velocity_pub = self.create_publisher(
            TwistStamped,
            '/mavros/setpoint_velocity/cmd_vel',
            10
        )
        
        # Raw setpoint publisher (most flexible)
        self.setpoint_raw_pub = self.create_publisher(
            PositionTarget,
            '/mavros/setpoint_raw/local',
            10
        )
        
        # Subscribers
        self.state_sub = self.create_subscription(
            State,
            '/mavros/state',
            self.state_callback,
            10
        )
        
        self.local_pos_sub = self.create_subscription(
            PoseStamped,
            '/mavros/local_position/pose',
            self.position_callback,
            10
        )
        
        # DVL velocity subscription
        from geometry_msgs.msg import TwistWithCovarianceStamped
        self.dvl_sub = self.create_subscription(
            TwistWithCovarianceStamped,
            '/mavros/vision_speed/speed_twist',
            self.dvl_callback,
            10
        )
        
        # Servo control
        self.servo = servo
        
        # Control thread
        self.control_thread = threading.Thread(target=self.control_loop)
        self.control_thread.daemon = True
        
        # Initialize system
        self.initialize_system()

    def state_callback(self, msg):
        """Update MAVROS connection state"""
        self.mavros_connected = msg.connected
        self.current_mode = msg.mode
        self.is_armed = msg.armed

    def position_callback(self, msg):
        """Update current position from local position estimate"""
        with self.data_lock:
            self.current_position['x'] = msg.pose.position.x
            self.current_position['y'] = msg.pose.position.y
            self.current_position['z'] = msg.pose.position.z
            
            # Convert quaternion to yaw (simplified)
            # For full 3D orientation, use proper quaternion math
            q = msg.pose.orientation
            self.current_orientation['yaw'] = math.atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            )

    def dvl_callback(self, msg):
        """DVL velocity data callback"""
        self.dvl_available = True
        # Could use this for velocity feedback if needed

    def initialize_system(self):
        """Initialize ArduSub system for GUIDED mode operation"""
        self.get_logger().info("🚁 Initializing ArduSub GUIDED Mode Control")
        self.get_logger().info("=========================================")
        
        # Wait for MAVROS connection
        self.get_logger().info("🔗 Waiting for MAVROS connection...")
        timeout = 10.0
        start_time = time.time()
        
        while not self.mavros_connected and (time.time() - start_time) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        
        if not self.mavros_connected:
            self.get_logger().error("❌ MAVROS not connected!")
            return False
        
        self.get_logger().info(f"✅ MAVROS connected - Mode: {self.current_mode}")
        
        # Wait for position data
        self.get_logger().info("📍 Waiting for position data...")
        start_time = time.time()
        while (abs(self.current_position['x']) < 0.001 and 
               abs(self.current_position['y']) < 0.001 and 
               (time.time() - start_time) < 5.0):
            rclpy.spin_once(self, timeout_sec=0.1)
        
        # Check DVL
        self.get_logger().info("🌊 Checking DVL data...")
        start_time = time.time()
        while not self.dvl_available and (time.time() - start_time) < 3.0:
            rclpy.spin_once(self, timeout_sec=0.1)
        
        if self.dvl_available:
            self.get_logger().info("✅ DVL data available")
        else:
            self.get_logger().warn("⚠️  No DVL data - position control may be limited")
        
        # Arm the vehicle FIRST (before setting GUIDED mode)
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
        
        # Set GUIDED mode AFTER arming
        if self.current_mode != "GUIDED":
            self.get_logger().info("🎯 Setting GUIDED mode...")
            if set_guided_mode():
                self.get_logger().info("✅ GUIDED mode set")
                time.sleep(2.0)  # Wait for mode to settle
                
                # Re-check armed status after mode change
                rclpy.spin_once(self, timeout_sec=0.5)
                if not self.is_armed:
                    self.get_logger().warn("⚠️  Vehicle disarmed after mode change - re-arming...")
                    try:
                        arm.ArmerNode()
                        time.sleep(2.0)
                        rclpy.spin_once(self, timeout_sec=0.5)
                    except Exception as e:
                        self.get_logger().warn(f"Re-arm failed: {e}")
            else:
                self.get_logger().error("❌ Failed to set GUIDED mode")
                return False
        
        # Initialize target position to current position
        with self.data_lock:
            self.target_position = self.current_position.copy()
            self.target_position['yaw'] = self.current_orientation['yaw']
        
        self.get_logger().info(f"📍 Starting position: X={self.current_position['x']:.2f}, "
                              f"Y={self.current_position['y']:.2f}, Z={self.current_position['z']:.2f}")
        
        # Final status check
        self.get_logger().info(f"🚁 System ready - Mode: {self.current_mode}, Armed: {self.is_armed}")
        
        return True

    def control_loop(self):
        """Main control loop for sending setpoints"""
        rate = self.create_rate(20)  # 20 Hz
        
        while self.flag and rclpy.ok():
            if self.position_control:
                self.send_position_setpoint()
            elif self.velocity_control:
                self.send_velocity_setpoint()
            
            try:
                rate.sleep()
            except:
                break

    def send_position_setpoint(self):
        """Send position setpoint to ArduSub"""
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        
        with self.data_lock:
            msg.pose.position.x = float(self.target_position['x'])
            msg.pose.position.y = float(self.target_position['y'])
            msg.pose.position.z = float(self.target_position['z'])
            
            # Convert yaw to quaternion (simplified, yaw-only)
            yaw = self.target_position['yaw']
            msg.pose.orientation.x = 0.0
            msg.pose.orientation.y = 0.0
            msg.pose.orientation.z = math.sin(yaw / 2.0)
            msg.pose.orientation.w = math.cos(yaw / 2.0)
        
        self.position_pub.publish(msg)

    def send_velocity_setpoint(self):
        """Send velocity setpoint (for future use)"""
        # Implementation for velocity control if needed
        pass

    def restore_terminal(self):
        """Restore terminal to normal mode"""
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)

    def start(self):
        """Start the keyboard control interface"""
        print("\n🚁 ArduSub GUIDED Mode Keyboard Control")
        print("======================================")
        print("DVL-based position control for underwater operations")
        
        # Wait for initialization
        if not self.initialize_system():
            self.get_logger().error("❌ System initialization failed")
            return
        
        # Start control loop
        self.control_thread.start()
        
        while not self.manual_control and not self.function_control:
            print("\nControl Modes:")
            print("  p - Position control (recommended)")
            print("  f - Function control (torpedoes, markers, etc.)")
            
            setting = input("Select mode: ").strip().lower()
            if setting == 'p':
                self.manual_control = True
                self.position_control = True
            elif setting == 'f':
                self.function_control = True
            else:
                print("[ERROR] Invalid input.")

        try:
            if self.manual_control:
                self.position_control_loop()
            elif self.function_control:
                self.function_control_loop()
        except KeyboardInterrupt:
            pass
        finally:
            self.shutdown()

    def position_control_loop(self):
        """Position control using keyboard"""
        print("\n🎯 Position Control Mode (GUIDED)")
        print("=================================")
        print("Controls:")
        print("  W/S: Forward/Backward (+/-Y)")
        print("  A/D: Left/Right (-/+X)")
        print("  Q/E: Rotate Left/Right")
        print("  R/F: Up/Down (-/+Z)")
        print("  H: Hold position (hover)")
        print("  I: Show current position")
        print("  T: Return to start position")
        print("  SPACE: Emergency stop")
        print("  B: Exit")
        print(f"\nStep sizes: Position={self.position_step}m, Depth={self.depth_step}m, Yaw={math.degrees(self.yaw_step):.1f}°")
        print("Commands are incremental - press key to move that amount")
        print("Focus this terminal and press keys...\n")

        # Set terminal to raw mode
        tty.setraw(sys.stdin.fileno())
        
        try:
            while self.manual_control:
                if select.select([sys.stdin], [], [], 0.1)[0]:
                    char = sys.stdin.read(1).lower()
                    
                    with self.data_lock:
                        if char == 'w':  # Forward (+Y in NED/body frame)
                            self.target_position['y'] += self.position_step
                            print(f"↑ Forward to Y={self.target_position['y']:.2f}", end='\r')
                            
                        elif char == 's':  # Backward (-Y)
                            self.target_position['y'] -= self.position_step
                            print(f"↓ Backward to Y={self.target_position['y']:.2f}", end='\r')
                            
                        elif char == 'a':  # Left (-X in NED frame)
                            self.target_position['x'] -= self.position_step
                            print(f"← Left to X={self.target_position['x']:.2f}", end='\r')
                            
                        elif char == 'd':  # Right (+X)
                            self.target_position['x'] += self.position_step
                            print(f"→ Right to X={self.target_position['x']:.2f}", end='\r')
                            
                        elif char == 'q':  # Rotate left
                            self.target_position['yaw'] -= self.yaw_step
                            yaw_deg = math.degrees(self.target_position['yaw'])
                            print(f"↺ Rotate left to {yaw_deg:.1f}°", end='\r')
                            
                        elif char == 'e':  # Rotate right
                            self.target_position['yaw'] += self.yaw_step
                            yaw_deg = math.degrees(self.target_position['yaw'])
                            print(f"↻ Rotate right to {yaw_deg:.1f}°", end='\r')
                            
                        elif char == 'r':  # Up (-Z in NED frame, negative is up)
                            self.target_position['z'] -= self.depth_step
                            print(f"⬆ Up to Z={self.target_position['z']:.2f}", end='\r')
                            
                        elif char == 'f':  # Down (+Z in NED frame)
                            self.target_position['z'] += self.depth_step
                            print(f"⬇ Down to Z={self.target_position['z']:.2f}", end='\r')
                            
                        elif char == 'h':  # Hold current position
                            self.target_position = self.current_position.copy()
                            self.target_position['yaw'] = self.current_orientation['yaw']
                            print("🛑 Holding current position", end='\r')
                            
                        elif char == 'i':  # Show info
                            print(f"\n📍 Current: X={self.current_position['x']:.2f}, Y={self.current_position['y']:.2f}, Z={self.current_position['z']:.2f}, Yaw={math.degrees(self.current_orientation['yaw']):.1f}°")
                            print(f"🎯 Target:  X={self.target_position['x']:.2f}, Y={self.target_position['y']:.2f}, Z={self.target_position['z']:.2f}, Yaw={math.degrees(self.target_position['yaw']):.1f}°")
                            
                        elif char == 't':  # Return to start
                            self.target_position = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'yaw': 0.0}
                            print("🏠 Returning to start position", end='\r')
                            
                        elif char == ' ':  # Emergency stop - hold position
                            self.target_position = self.current_position.copy()
                            self.target_position['yaw'] = self.current_orientation['yaw']
                            print("🚨 Emergency stop - holding position", end='\r')
                            
                        elif char == 'b':  # Exit
                            print("\n👋 Exiting position control")
                            self.manual_control = False
                            break
                            
                        elif char == '\x03':  # Ctrl+C
                            print("\n⏹ Keyboard interrupt")
                            self.manual_control = False
                            break
                            
                        else:
                            print(f"❌ Unknown key: {char}", end='\r')
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)

    def function_control_loop(self):
        """Function control for servos and actuators"""
        print("\n🔧 Function Control Mode")
        print("========================")
        print("Commands:")
        print("  fire <num>  - Fire torpedo (1-2)")
        print("  drop <num>  - Drop marker (1-2)")
        print("  open        - Open gripper")
        print("  close       - Close gripper")
        print("  arm         - Arm vehicle")
        print("  disarm      - Disarm vehicle")
        print("  status      - Show system status")
        print("  exit        - Exit function mode")
        
        self.restore_terminal()
        
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
                    
                elif cmd == "arm":
                    try:
                        arm.ArmerNode()
                        print("🔫 Arm command sent")
                    except Exception as e:
                        print(f"❌ Arm failed: {e}")
                        
                elif cmd == "disarm":
                    try:
                        disarm.DisarmerNode()
                        print("🔒 Disarm command sent")
                    except Exception as e:
                        print(f"❌ Disarm failed: {e}")
                        
                elif cmd == "status":
                    print(f"📊 Mode: {self.current_mode}, Armed: {self.is_armed}, Connected: {self.mavros_connected}")
                    print(f"📍 Position: X={self.current_position['x']:.2f}, Y={self.current_position['y']:.2f}, Z={self.current_position['z']:.2f}")
                    print(f"🧭 Yaw: {math.degrees(self.current_orientation['yaw']):.1f}°")
                    print(f"🌊 DVL: {'Available' if self.dvl_available else 'Not available'}")
                    
                elif cmd == "exit":
                    self.function_control = False
                    print("👋 Exiting function control")
                    
                else:
                    print("❌ Unknown command")
                    
            except KeyboardInterrupt:
                break
                
        self.function_control = False

    def shutdown(self):
        """Clean shutdown"""
        print("\n🛑 Shutting down...")
        
        self.flag = False
        self.restore_terminal()
        
        if self.control_thread.is_alive():
            self.control_thread.join(timeout=2.0)
        
        # Hold current position before disarming
        with self.data_lock:
            self.target_position = self.current_position.copy()
            self.target_position['yaw'] = self.current_orientation['yaw']
        
        # Send hold command a few times
        for _ in range(10):
            self.send_position_setpoint()
            time.sleep(0.05)
        
        # Disarm
        try:
            disarm.DisarmerNode()
            self.get_logger().info("🔒 Disarmed")
        except Exception as e:
            self.get_logger().warn(f"Disarm failed: {e}")
        
        self.get_logger().info("✅ Shutdown complete")


def main(args=None):
    rclpy.init(args=args)
    
    node = GuidedModeKeyboardControl()
    
    try:
        node.start()
    finally:
        node.restore_terminal()
        node.destroy_node()
        
    rclpy.shutdown()


if __name__ == '__main__':
    main()
