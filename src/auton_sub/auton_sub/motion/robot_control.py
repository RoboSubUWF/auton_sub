# ROS 2 version of RobotControl with hardcoded configuration for a single submarine

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import OverrideRCIn, ManualControl
from std_srvs.srv import Trigger

from simple_pid import PID
import numpy as np
import math
import time
import threading


def clip(pwm_min: int, pwm_max: int, value: int):
    value = min(pwm_max, value)
    value = max(pwm_min, value)
    return value


class RobotControl(Node):
    def __init__(self):
        super().__init__('robot_control')

        self.rate = self.create_rate(20)  # 20 Hz loop rate
        self.lock = threading.Lock()
        self.debug = False

        # Hardcoded config (since no deviceHelper file)
        self.pwm_scale = 200  # Increased from 80 for more responsive control
        self.pwm_neutral = 1500
        self.pwm_min = 1100
        self.pwm_max = 1900

        # Robot state
        self.position = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        self.orientation = {'yaw': 0.0, 'pitch': 0.0, 'roll': 0.0}
        self.desired_point = {'x': None, 'y': None, 'z': None, 'yaw': None, 'pitch': None, 'roll': None}
        self.direct_input = [0] * 6
        self.mode = "pid"

        # Publishers and subscribers
        self.pose_sub = self.create_subscription(PoseStamped, '/pose', self.pose_callback, 10)
        self.thruster_pub = self.create_publisher(OverrideRCIn, '/mavros/rc/override', 10)
        
        # Also publish manual control for MANUAL mode
        self.manual_control_pub = self.create_publisher(ManualControl, '/mavros/manual_control/control', 10)

        # PID controllers
        self.PIDs = {
            "yaw": PID(12.0, 0.01, 0.0, setpoint=0, output_limits=(-1, 1)),
            "pitch": PID(0.5, 0.1, 0.1, setpoint=0, output_limits=(-5, 5)),
            "roll": PID(0.5, 0.1, 0.1, setpoint=0, output_limits=(-5, 5)),
            "surge": PID(2.0, 0.05, 0.01, setpoint=0, output_limits=(-2, 2)),
            "lateral": PID(2.0, 0.05, 0.01, setpoint=0, output_limits=(-2, 2)),
            "depth": PID(100.0, 10.0, 0.75, setpoint=0),
        }

        # Start the main control thread
        self.running = True
        self.control_thread = threading.Thread(target=self.control_loop)
        self.control_thread.start()
        
        self.get_logger().info("RobotControl initialized - PWM scale: %d" % self.pwm_scale)

    def pose_callback(self, msg):
        self.position['x'] = msg.pose.position.x
        self.position['y'] = msg.pose.position.y
        self.position['z'] = msg.pose.position.z
        # Assuming quaternion is [0, 0, sin(yaw/2), cos(yaw/2)] for yaw only (simplified)
        self.orientation['yaw'] = msg.pose.orientation.z  # You should convert properly if using full 3D orientation
        
    def set_depth(self, target_depth):
        """Set the target depth for the submarine"""
        with self.lock:
            self.desired_point['z'] = target_depth
            # Ensure we're in PID mode for depth control
            if self.mode != "pid":
                self.get_logger().info("Switching to PID mode for depth control")
                self.mode = "pid"
        
        self.get_logger().info(f"Target depth set to: {target_depth}m")

    def set_position(self, x=None, y=None, z=None, yaw=None):
        """Set target position and orientation"""
        with self.lock:
            if x is not None:
                self.desired_point['x'] = x
            if y is not None:
                self.desired_point['y'] = y
            if z is not None:
                self.desired_point['z'] = z
            if yaw is not None:
                self.desired_point['yaw'] = yaw
                
            # Switch to PID mode for position control
            if self.mode != "pid":
                self.get_logger().info("Switching to PID mode for position control")
                self.mode = "pid"

    def get_current_depth(self):
        """Get current depth"""
        return self.position['z']

    def get_current_position(self):
        """Get current position"""
        return self.position.copy()
    
    def control_loop(self):
        while rclpy.ok() and self.running:
            if self.mode == "pid":
                self.update_pid_control()
            elif self.mode == "direct":
                self.send_pwm_from_direct_input()
                self.send_manual_control()  # Also send manual control commands
            time.sleep(0.05)

    def update_pid_control(self):
        desired = {
            'x': self.desired_point["x"] if self.desired_point["x"] is not None else self.position['x'],
            'y': self.desired_point["y"] if self.desired_point["y"] is not None else self.position['y'],
            'z': self.desired_point["z"] if self.desired_point["z"] is not None else self.position['z'],
            'yaw': self.desired_point["yaw"] if self.desired_point["yaw"] is not None else self.orientation['yaw'],
        }

        errors = {
            'x': desired['x'] - self.position['x'],
            'y': desired['y'] - self.position['y'],
            'z': desired['z'] - self.position['z'],
            'yaw': desired['yaw'] - self.orientation['yaw'],
        }

        pwm = OverrideRCIn()
        pwm.channels = [self.pwm_neutral] * 18

        # Fixed channel assignments for typical ArduSub configuration
        # Channels 1-6 (index 0-5) for thrusters
        pwm.channels[0] = clip(self.pwm_min, self.pwm_max, int(self.PIDs["pitch"](0) * self.pwm_scale + self.pwm_neutral))
        pwm.channels[1] = clip(self.pwm_min, self.pwm_max, int(self.PIDs["roll"](0) * self.pwm_scale + self.pwm_neutral))  # FIXED
        pwm.channels[2] = clip(self.pwm_min, self.pwm_max, int(self.PIDs["depth"](errors['z']) * self.pwm_scale + self.pwm_neutral))
        pwm.channels[3] = clip(self.pwm_min, self.pwm_max, int(self.PIDs["yaw"](errors['yaw']) * self.pwm_scale + self.pwm_neutral))
        pwm.channels[4] = clip(self.pwm_min, self.pwm_max, int(self.PIDs["surge"](errors['y']) * self.pwm_scale + self.pwm_neutral))
        pwm.channels[5] = clip(self.pwm_min, self.pwm_max, int(self.PIDs["lateral"](errors['x']) * self.pwm_scale + self.pwm_neutral))

        self.thruster_pub.publish(pwm)

    def send_pwm_from_direct_input(self):
        pwm = OverrideRCIn()
        pwm.channels = [self.pwm_neutral] * 18
        
        with self.lock:
            # Map direct input to thruster channels
            # direct_input: [lateral, forward, yaw, 0, 0, 0] from keyboard
            # Map to ArduSub motor outputs:
            lateral = self.direct_input[0]    # Left/Right
            forward = self.direct_input[1]    # Forward/Backward  
            yaw = self.direct_input[2]        # Rotation
            
            # For typical vectored frame:
            # Motors 1-4: Vertical (up/down) - index 0-3
            # Motors 5-6: Horizontal forward/back - index 4-5
            
            # Vertical thrusters (all receive same signal for now)
            vertical_pwm = self.pwm_neutral  # No vertical control in this basic setup
            
            # Forward/backward thrusters
            forward_pwm = int(forward * self.pwm_scale + self.pwm_neutral)
            
            # Lateral movement via differential thrust (if using vectored config)
            left_thrust = int((forward + yaw + lateral) * self.pwm_scale/3 + self.pwm_neutral)
            right_thrust = int((forward - yaw - lateral) * self.pwm_scale/3 + self.pwm_neutral)
            
            # Channel mapping for BlueROV2/typical configuration:
            pwm.channels[0] = clip(self.pwm_min, self.pwm_max, vertical_pwm)     # Vertical 1
            pwm.channels[1] = clip(self.pwm_min, self.pwm_max, vertical_pwm)     # Vertical 2
            pwm.channels[2] = clip(self.pwm_min, self.pwm_max, vertical_pwm)     # Vertical 3
            pwm.channels[3] = clip(self.pwm_min, self.pwm_max, vertical_pwm)     # Vertical 4
            pwm.channels[4] = clip(self.pwm_min, self.pwm_max, left_thrust)      # Left forward
            pwm.channels[5] = clip(self.pwm_min, self.pwm_max, right_thrust)     # Right forward
            
            if self.debug:
                self.get_logger().info(f"Direct input: L={lateral:.2f} F={forward:.2f} Y={yaw:.2f}")
                self.get_logger().info(f"PWM: L={left_thrust} R={right_thrust}")
        
        self.thruster_pub.publish(pwm)

    def send_manual_control(self):
        """Send manual control commands for MANUAL flight mode"""
        manual = ManualControl()
        
        with self.lock:
            # Scale from -1,1 to -1000,1000 for manual control (use float values)
            manual.x = float(self.direct_input[1] * 1000.0)    # Forward/backward
            manual.y = float(self.direct_input[0] * 1000.0)    # Left/right  
            manual.z = 500.0  # Neutral throttle
            manual.r = float(self.direct_input[2] * 1000.0)    # Yaw
            
        self.manual_control_pub.publish(manual)

    def stop(self):
        self.running = False
        self.control_thread.join()
        self.get_logger().info("RobotControl stopped.")


def main(args=None):
    rclpy.init(args=args)
    rc = RobotControl()
    try:
        rclpy.spin(rc)
    except KeyboardInterrupt:
        rc.get_logger().info("Keyboard interrupt - shutting down")
    finally:
        rc.stop()
        rc.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()