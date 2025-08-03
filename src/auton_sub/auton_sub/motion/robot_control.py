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
import tf_transformations


def clip(pwm_min: int, pwm_max: int, value: int):
    value = min(pwm_max, value)  #chooses lowest value between max and value
    value = max(pwm_min, value)  # chooses maximum value between min and value
    return value


class RobotControl(Node):
    def __init__(self):
        super().__init__('robot_control')

        self.rate = self.create_rate(20)  # 20 Hz loop rate (do this 20 times per second)
        self.lock = threading.Lock() #make sure different threads arent posting different data in same place same time
        self.debug = False # does not print debug info

        # Hardcoded config (since no deviceHelper file)
        self.pwm_scale = 400  # 1*400+1500 = full speed
        self.pwm_neutral = 1500 #off
        self.pwm_min = 1100 #backwards full speed
        self.pwm_max = 1900 #forwards full speed

        # Robot state
        self.position = {'x': 0.0, 'y': 0.0, 'z': 0.0}  #sets starting position as home position
        self.orientation = {'yaw': 0.0, 'pitch': 0.0, 'roll': 0.0}  #sets starting values
        self.desired_point = {'x': None, 'y': None, 'z': None, 'yaw': None, 'pitch': None, 'roll': None} #also initializes
        self.movement_command = {'lateral': 0.0, 'forward': 0.0, 'yaw': 0.0}
        self.max_descent_mode = False 
        self.mode = "hybrid"

        # Publishers and subscribers
        self.pose_sub = self.create_subscription(PoseStamped, '/pose', self.pose_callback, 10) #creates subscription to recieve x,y,z
        self.thruster_pub = self.create_publisher(OverrideRCIn, '/mavros/rc/override', 10) #publishes where to move
        
        # Also publish manual control for MANUAL mode
        self.manual_control_pub = self.create_publisher(ManualControl, '/mavros/manual_control/control', 10) #direct control

        ##kp=how much it reacts to current error *0.1to 100, 
        # Ki=reacts to past error *0 to 5
        #kd = reacts to to future predicted error 0 to 10
        #setpoint (wants error to be zero)
        #
        # PID controllers
        self.PIDs = {   
            "yaw": PID(12.0, 0.01, 0.0, setpoint=0, output_limits=(-2, 2)),
            "pitch": PID(0.5, 0.1, 0.1, setpoint=0, output_limits=(-2, 2)),
            "roll": PID(0.5, 0.1, 0.1, setpoint=0, output_limits=(-2, 2)),
            "surge": PID(2.0, 0.05, 0.01, setpoint=0, output_limits=(-2, 2)),
            "lateral": PID(2.0, 0.05, 0.01, setpoint=0, output_limits=(-2, 2)),
            "depth": PID(100.0, 10.0, 0.75, setpoint=0, output_limits=(-2,2)),
        }

        # Start the main control thread
        self.running = True #keep running as long as ros2 is running
        self.control_thread = threading.Thread(target=self.control_loop) #runs control loop when called
        self.control_thread.start() #starts the thread
        
        self.get_logger().info("RobotControl initialized - PWM scale: %d" % self.pwm_scale)

    def pose_callback(self, msg): #called anytime a pose message is recieved
        self.position['x'] = msg.pose.position.x #copies the recieved positions x y z to position dictionary
        self.position['y'] = msg.pose.position.y
        self.position['z'] = msg.pose.position.z
        # gets quaternian euler 
        q = msg.pose.orientation
        quaternion = [q.x, q.y, q.z, q.w]
        (_, _, yaw) = tf_transformations.euler_from_quaternion(quaternion)

        self.orientation['yaw'] = yaw #extracts only yaw
        #self.orientation['roll'] = roll #not used
        #self.orientation['pitch'] = pitch #not used
        
    def set_depth(self, target_depth):
        """Set the target depth for the submarine"""
        with self.lock:
            self.desired_point['z'] = target_depth
        self.get_logger().info(f"Target depth set to: {target_depth}m")

    def set_max_descent_rate(self, enable):
        """Enable/disable maximum descent rate for initial descent"""
        with self.lock:
            self.max_descent_mode = enable
        if enable:
            self.get_logger().info("Maximum descent rate enabled")
        else:
            self.get_logger().info("Normal PID depth control resumed")

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

    def set_movement_command(self, lateral=0.0, forward=0.0, yaw=0.0):
        """Set movement commands while maintaining depth control"""
        with self.lock:
            self.movement_command['lateral'] = lateral
            self.movement_command['forward'] = forward
            self.movement_command['yaw'] = yaw
        
        if abs(lateral) > 0.01 or abs(forward) > 0.01 or abs(yaw) > 0.01:
            self.get_logger().info(f"Movement command: lateral={lateral:.2f}, forward={forward:.2f}, yaw={yaw:.2f}")            
    
    def get_current_depth(self):
        """Get current depth"""
        return self.position['z']

    def get_current_position(self):
        """Get current position"""
        return self.position.copy()
    
    def control_loop(self):
        while rclpy.ok() and self.running:
            self.update_hybrid_control()
            time.sleep(0.05)

    def update_hybrid_control(self):
        """Hybrid control: continuous depth PID + movement commands OR position control"""
        with self.lock:
            # Check if we have position setpoints (for waypoint navigation)
            has_position_targets = (self.desired_point['x'] is not None or 
                                  self.desired_point['y'] is not None or
                                  self.desired_point['yaw'] is not None)
            
            if has_position_targets:
                # Position control mode - use PID to reach specific coordinates
                #lateral_cmd = 0.0
                forward_cmd = 0.0 
                yaw_cmd = 0.0
                
                # X position control (lateral movement)
                if self.desired_point['x'] is not None:
                    x_error = self.desired_point['x'] - self.position['x']
                    #lateral_cmd = self.PIDs["lateral"](x_error)
                    if abs(x_error) > 0.1:
                        self.get_logger().warn(f"Cannot move laterally - no lateral thrusters. X error: {x_error:.2f}m")
                        
                # Y position control (forward/backward movement)  
                if self.desired_point['y'] is not None:
                    y_error = self.desired_point['y'] - self.position['y']
                    forward_cmd = self.PIDs["surge"](y_error)
                
                # Yaw control
                if self.desired_point['yaw'] is not None:
                    yaw_error = self.desired_point['yaw'] - self.orientation['yaw']
                    yaw_cmd = self.PIDs["yaw"](yaw_error)
                    
            else:
                # Movement command mode - direct velocity commands
                #lateral_cmd = self.movement_command['lateral']
                forward_cmd = self.movement_command['forward']
                yaw_cmd = self.movement_command['yaw']
            
            # Calculate depth error and PID output (always active)
            if self.desired_point['z'] is not None:
                depth_error = self.desired_point['z'] - self.position['z']
                
                if self.max_descent_mode and depth_error > 0.1:
                    # Maximum descent - force full downward thrust
                    depth_output = 1.0  # Full power down
                else:
                    # Normal PID control
                    depth_output = self.PIDs["depth"](depth_error)
                    # Clamp depth output to reasonable range
                    depth_output = max(-1.0, min(1.0, depth_output))
            else:
                depth_output = 0.0

        # Create PWM message
        pwm = OverrideRCIn()
        pwm.channels = [self.pwm_neutral] * 18

        # Calculate thruster outputs for vectored configuration
        # Assuming typical BlueROV2 configuration:
        # - 4 vertical thrusters (channels 0-3) for depth control
        # - 2 horizontal thrusters (channels 4-5) for forward/yaw/lateral

        # Vertical thrusters - all get same depth control signal
        vertical_pwm = int(depth_output * self.pwm_scale + self.pwm_neutral)
        vertical_pwm = clip(self.pwm_min, self.pwm_max, vertical_pwm)
        
        # Horizontal thrusters - combine forward, lateral, and yaw commands
        # Simple differential thrust for lateral and yaw
        left_horizontal = forward_cmd + yaw_cmd * 0.5
        right_horizontal = forward_cmd - yaw_cmd * 0.5
        
        # Clamp horizontal commands
        left_horizontal = max(-1.0, min(1.0, left_horizontal))
        right_horizontal = max(-1.0, min(1.0, right_horizontal))
        
        left_pwm = int(left_horizontal * self.pwm_scale + self.pwm_neutral)
        right_pwm = int(right_horizontal * self.pwm_scale + self.pwm_neutral)
        
        left_pwm = clip(self.pwm_min, self.pwm_max, left_pwm)
        right_pwm = clip(self.pwm_min, self.pwm_max, right_pwm)

        # Assign to channels (typical BlueROV2 mapping)
        pwm.channels[0] = vertical_pwm    # Vertical thruster 1
        pwm.channels[1] = vertical_pwm    # Vertical thruster 2  
        pwm.channels[2] = vertical_pwm    # Vertical thruster 3
        pwm.channels[3] = vertical_pwm    # Vertical thruster 4
        pwm.channels[4] = left_pwm        # Left horizontal thruster
        pwm.channels[5] = right_pwm       # Right horizontal thruster

        # Debug logging
        if self.debug and (abs(depth_output) > 0.1 or abs(forward_cmd) > 0.1 or abs(yaw_cmd) > 0.1):
            current_depth = self.position['z']
            target_depth = self.desired_point['z'] if self.desired_point['z'] is not None else current_depth
            self.get_logger().info(f"Control: depth={current_depth:.2f}→{target_depth:.2f} (PWM:{vertical_pwm}) "
                                 f"move: F={forward_cmd:.2f} Y={yaw_cmd:.2f} (PWM: L={left_pwm} R={right_pwm})")

        self.thruster_pub.publish(pwm)

    def stop(self):
        """Stop the control system"""
        self.running = False
        
        # Send neutral PWM to all channels
        pwm = OverrideRCIn()
        pwm.channels = [self.pwm_neutral] * 18
        self.thruster_pub.publish(pwm)
        
        if self.control_thread.is_alive():
            self.control_thread.join()
        self.get_logger().info("RobotControl stopped.")


def main(args=None): #function can optionally accept command-line arguments (passed in from ROS 2 or the terminal).
    rclpy.init(args=args) #initializes the ROS 2 Python library (rclpy)
    rc = RobotControl()
    try:
        rclpy.spin(rc) #tells ROS to keep the program running and wait for messages
    except KeyboardInterrupt:
        rc.get_logger().info("Keyboard interrupt - shutting down")
    finally:
        rc.stop()
        rc.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()