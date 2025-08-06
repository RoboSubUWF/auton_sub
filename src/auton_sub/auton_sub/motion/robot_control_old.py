# ROS 2 version of RobotControl with hardcoded configuration for a single submarine

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import OverrideRCIn, ManualControl
from sensor_msgs.msg import FluidPressure
from std_srvs.srv import Trigger

# Try to import VFR_HUD, fall back to alternative if not available
try:
    from mavros_msgs.msg import VFR_HUD
    VFR_HUD_AVAILABLE = True
except ImportError:
    print("Warning: VFR_HUD not available in mavros_msgs, using pressure sensor only")
    VFR_HUD_AVAILABLE = False

from simple_pid import PID
import numpy as np
import math
import time
import threading



def clip(pwm_min: int, pwm_max: int, value: int):
    value = min(pwm_max, value)  #chooses lowest value between max and value
    value = max(pwm_min, value)  # chooses maximum value between min and value
    return value

def quaternion_to_yaw(x, y, z, w):
    """Convert quaternion to yaw angle (in radians)"""
    # Formula for yaw from quaternion
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return yaw

class RobotControl(Node):
    def __init__(self):
        super().__init__('robot_control')

        self.rate = self.create_rate(20)  # 20 Hz loop rate (do this 20 times per second)
        self.lock = threading.Lock() #make sure different threads arent posting different data in same place same time
        self.debug = True # Enable debug info to see what's happening

        # Hardcoded config (since no deviceHelper file)
        self.pwm_scale = 400  # 1*400+1500 = full speed
        self.pwm_neutral = 1500 #off
        self.pwm_min = 1100 #backwards full speed
        self.pwm_max = 1900 #forwards full speed

        # Robot state - separate DVL position from pressure depth
        self.dvl_position = {'x': 0.0, 'y': 0.0, 'z': 0.0}  # DVL position (when available)
        self.pressure_depth = 0.0  # Pressure-based depth (continuous)
        self.orientation = {'yaw': 0.0, 'pitch': 0.0, 'roll': 0.0}  #sets starting values
        self.desired_point = {'x': None, 'y': None, 'z': None, 'yaw': None, 'pitch': None, 'roll': None} #also initializes
        self.movement_command = {'forward': 0.0, 'yaw': 0.0}
        self.max_descent_mode = False 
        self.mode = "hybrid"

        # Add flags to track which sensors are available
        self.dvl_valid = False
        self.pressure_valid = False
        
        # Dead reckoning state
        self.estimated_position = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        self.last_update_time = time.time()
        self.last_movement_command = {'forward': 0.0, 'yaw': 0.0}

        # Publishers and subscribers
        self.pose_sub = self.create_subscription(PoseStamped, '/pose', self.pose_callback, 10) #creates subscription to recieve x,y,z
        self.thruster_pub = self.create_publisher(OverrideRCIn, '/mavros/rc/override', 10) #publishes where to move
        
        # Add pressure sensor subscription for continuous depth
        self.pressure_sub = self.create_subscription(
            FluidPressure, '/mavros/imu/static_pressure', self.pressure_callback, 10)
        
        # Alternative: use VFR_HUD which includes altitude/depth (if available)
        if VFR_HUD_AVAILABLE:
            self.vfr_sub = self.create_subscription(
                VFR_HUD, '/mavros/vfr_hud', self.vfr_callback, 10)
        
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
            "depth": PID(2.0, 0.1, 0.5, setpoint=0, output_limits=(-2.0, 2.0))
        }

        # Start the main control thread
        self.running = True #keep running as long as ros2 is running
        self.control_thread = threading.Thread(target=self.control_loop) #runs control loop when called
        self.control_thread.start() #starts the thread
        
        self.get_logger().info("RobotControl initialized - PWM scale: %d" % self.pwm_scale)

    def pressure_callback(self, msg):
        """Convert pressure to depth using standard formula"""
        # Pressure to depth conversion (approximate)
        # 1 meter of water ≈ 9800 Pa additional pressure
        # Depth = (pressure - atmospheric_pressure) / (water_density * gravity)
        atmospheric_pressure = 101325.0  # Pa at sea level
        water_density = 1025.0  # kg/m³ (saltwater)
        gravity = 9.81  # m/s²
        
        pressure_diff = msg.fluid_pressure - atmospheric_pressure
        self.pressure_depth = pressure_diff / (water_density * gravity)
        self.pressure_valid = True
        
        if self.debug:
            self.get_logger().info(f"Pressure depth: {self.pressure_depth:.2f}m")

    def vfr_callback(self, msg):
        """Alternative: use VFR_HUD altitude as depth (only if VFR_HUD is available)"""
        if VFR_HUD_AVAILABLE:
            # VFR_HUD altitude is often negative depth in ArduSub
            self.pressure_depth = -msg.altitude  # Convert altitude to positive depth
            self.pressure_valid = True
            
            if self.debug:
                self.get_logger().info(f"VFR depth: {self.pressure_depth:.2f}m")

    def pose_callback(self, msg):
        """DVL position callback - only updates when DVL has valid data"""
        # Check if position data is valid (not NaN)
        if (not math.isnan(msg.pose.position.x) and 
            not math.isnan(msg.pose.position.y) and 
            not math.isnan(msg.pose.position.z)):
            
            self.dvl_position['x'] = msg.pose.position.x
            self.dvl_position['y'] = msg.pose.position.y
            self.dvl_position['z'] = msg.pose.position.z
            self.dvl_valid = True
            
            if self.debug:
                self.get_logger().info(f"DVL position: x={self.dvl_position['x']:.2f}, y={self.dvl_position['y']:.2f}, z={self.dvl_position['z']:.2f}")
        else:
            self.dvl_valid = False
            if self.debug:
                self.get_logger().warn("DVL position invalid (NaN)")
        
        # Always try to update orientation (usually from IMU, more reliable)
        q = msg.pose.orientation
        if not math.isnan(q.w):
            self.orientation['yaw'] = quaternion_to_yaw(q.x, q.y, q.z, q.w)
        
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
            self.movement_command['forward'] = forward
            self.movement_command['yaw'] = yaw
            self.last_movement_command = self.movement_command.copy()
        
        # Update position estimate when movement changes
        self.update_estimated_position()
        
        if abs(lateral) > 0.01 or abs(forward) > 0.01 or abs(yaw) > 0.01:
            self.get_logger().info(f"Movement command: lateral={lateral:.2f}, forward={forward:.2f}, yaw={yaw:.2f}")            
    
    def update_estimated_position(self):
        """Update position estimate based on movement commands"""
        current_time = time.time()
        dt = current_time - self.last_update_time
        
        if dt > 0.1:  # Update every 100ms
            # Estimate forward movement (assume we move at commanded speed)
            forward_speed = self.last_movement_command['forward']
            yaw = self.orientation['yaw']
            
            # Convert forward movement to x,y displacement
            dx = forward_speed * math.cos(yaw) * dt
            dy = forward_speed * math.sin(yaw) * dt
            
            self.estimated_position['x'] += dx
            self.estimated_position['y'] += dy
            
            # Use pressure depth for z
            self.estimated_position['z'] = self.get_current_depth()
            
            self.last_update_time = current_time
    
    def get_current_depth(self):
        """Get current depth - prefer pressure sensor over DVL"""
        if self.pressure_valid:
            return self.pressure_depth
        elif self.dvl_valid:
            return self.dvl_position['z']
        else:
            return 0.0  # Fallback

    def get_current_position(self):
        """Get current position - use DVL when available, otherwise dead reckoning"""
        self.update_estimated_position()
        
        if self.dvl_valid:
            # Use DVL position when available
            return {
                'x': self.dvl_position['x'],
                'y': self.dvl_position['y'],
                'z': self.get_current_depth(),  # Always use pressure for depth
                'yaw': self.orientation['yaw']
            }
        else:
            # Use estimated position
            return {
                'x': self.estimated_position['x'],
                'y': self.estimated_position['y'],
                'z': self.estimated_position['z'],
                'yaw': self.orientation['yaw']
            }
    
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
                forward_cmd = 0.0 
                yaw_cmd = 0.0
                
                # X position control (lateral movement)
                if self.desired_point['x'] is not None:
                    current_pos = self.get_current_position()
                    x_error = self.desired_point['x'] - current_pos['x']
                    if abs(x_error) > 0.1:
                        self.get_logger().warn(f"Cannot move laterally - no lateral thrusters. X error: {x_error:.2f}m")
                        
                # Y position control (forward/backward movement)  
                if self.desired_point['y'] is not None:
                    current_pos = self.get_current_position()
                    y_error = self.desired_point['y'] - current_pos['y']
                    forward_cmd = self.PIDs["surge"](y_error)
                
                # Yaw control
                if self.desired_point['yaw'] is not None:
                    yaw_error = self.desired_point['yaw'] - self.orientation['yaw']
                    yaw_cmd = self.PIDs["yaw"](yaw_error)
                    
            else:
                # Movement command mode - direct velocity commands
                forward_cmd = self.movement_command['forward']
                yaw_cmd = self.movement_command['yaw']
            
            # Calculate depth error and PID output (always active)
            if self.desired_point['z'] is not None:
                current_depth = self.get_current_depth()
                depth_error = self.desired_point['z'] - current_depth
                
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
        # Standard BlueROV2 Heavy configuration:
        # - Channels 2,3,4,5 (indices 1,2,3,4) for vertical thrusters
        # - Channels 0,1 (indices 0,1) for horizontal thrusters (forward/yaw)

        # Vertical thrusters - all get same depth control signal
        vertical_pwm = int(depth_output * self.pwm_scale + self.pwm_neutral)
        vertical_pwm = clip(self.pwm_min, self.pwm_max, vertical_pwm)
        
        # Horizontal thrusters - combine forward and yaw commands
        # Simple differential thrust for forward and yaw
        left_horizontal = forward_cmd + yaw_cmd * 0.5
        right_horizontal = forward_cmd - yaw_cmd * 0.5
        
        # Clamp horizontal commands
        left_horizontal = max(-1.0, min(1.0, left_horizontal))
        right_horizontal = max(-1.0, min(1.0, right_horizontal))
        
        left_pwm = int(left_horizontal * self.pwm_scale + self.pwm_neutral)
        right_pwm = int(right_horizontal * self.pwm_scale + self.pwm_neutral)
        
        left_pwm = clip(self.pwm_min, self.pwm_max, left_pwm)
        right_pwm = clip(self.pwm_min, self.pwm_max, right_pwm)

        # Standard BlueROV2 Heavy channel mapping (0-indexed for array):
        pwm.channels[4] = left_pwm      # Channel 1: Left horizontal thruster
        pwm.channels[5] = right_pwm     # Channel 2: Right horizontal thruster  
        pwm.channels[0] = vertical_pwm  # Channel 3: Front-left vertical
        pwm.channels[1] = vertical_pwm  # Channel 4: Front-right vertical
        pwm.channels[2] = vertical_pwm  # Channel 5: Back-left vertical
        pwm.channels[3] = vertical_pwm  # Channel 6: Back-right vertical

        # Debug logging
        if self.debug and (abs(depth_output) > 0.1 or abs(forward_cmd) > 0.1 or abs(yaw_cmd) > 0.1):
            current_depth = self.get_current_depth()
            target_depth = self.desired_point['z'] if self.desired_point['z'] is not None else current_depth
            dvl_status = "DVL" if self.dvl_valid else "EST"
            pressure_status = "PRESS" if self.pressure_valid else "DVL"
            self.get_logger().info(f"Control: depth={current_depth:.2f}→{target_depth:.2f} ({pressure_status}, PWM:{vertical_pwm}) "
                                 f"move: F={forward_cmd:.2f} Y={yaw_cmd:.2f} ({dvl_status}, PWM: L={left_pwm} R={right_pwm})")

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
