# ROS 2 version of RobotControl using heading-based control with DVL integration

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import ManualControl, AttitudeTarget, PositionTarget
from mavros_msgs.srv import SetMode, CommandBool
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger

from simple_pid import PID
import numpy as np
import math
import time
import threading


def quaternion_to_yaw(x, y, z, w):
    """Convert quaternion to yaw angle (in radians)"""
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return yaw

class RobotControl(Node):
    def __init__(self):
        super().__init__('robot_control')

        self.rate = self.create_rate(20)  # 20 Hz loop rate
        self.lock = threading.Lock()#make sure different threads arent posting different data in same place same time
        self.debug = True  # Enable debug info

        # Robot state - Use ArduSub EKF fused position (includes DVL + IMU)
        self.position = {'x': 0.0, 'y': 0.0, 'z': 0.0}  # EKF fused position from ArduSub
        self.velocity = {'x': 0.0, 'y': 0.0, 'z': 0.0}  # EKF fused velocity
        self.orientation = {'yaw': 0.0, 'pitch': 0.0, 'roll': 0.0}
        self.desired_point = {'x': None, 'y': None, 'z': None, 'yaw': None}
        self.movement_command = {'forward': 0.0, 'yaw': 0.0}
        self.max_descent_mode = False 
        self.mode = "guided"

        # Status flags
        self.position_valid = False
        self.velocity_valid = False
        self.last_position_time = time.time()
        self.position_timeout = 2.0  # seconds

        # Publishers and subscribers - Use proper MAVROS topics for DVL integration
        # LOCAL_POSITION_NED from ArduSub EKF (fuses DVL + IMU + barometer)
        self.pose_sub = self.create_subscription(
            PoseStamped, '/mavros/local_position/pose', self.ekf_pose_callback, 10) #gets position and stores the last 10 messages in pose
        
        # Get velocity from EKF for better dead reckoning
        self.velocity_sub = self.create_subscription(
            TwistStamped, '/mavros/local_position/velocity_local', self.ekf_velocity_callback, 10)
        
        # Publishers for GUIDED mode control
        # Use velocity commands for GUIDED mode (preferred method)
        self.velocity_pub = self.create_publisher(Twist, '/mavros/setpoint_velocity/cmd_vel_unstamped', 10)
        
        # Alternative: Use position targets for waypoint navigation
        self.position_target_pub = self.create_publisher(PositionTarget, '/mavros/setpoint_raw/local', 10)
        
        # ManualControl for MANUAL mode fallback (if needed)
        self.manual_control_pub = self.create_publisher(ManualControl, '/mavros/manual_control/control', 10)

        #kp=how much it reacts to current error *0.1to 100, 
        # Ki=reacts to past error *0 to 5
        #kd = reacts to to future predicted error 0 to 10
        #setpoint (wants error to be zero)
        # PID controllers - tuned for heading-based control
        self.PIDs = {   
            "yaw": PID(1.5, 0.02, 0.05, setpoint=0, output_limits=(-1.0, 1.0)),
            "depth": PID(1.0, 0.1, 0.2, setpoint=0, output_limits=(-1.0, 1.0)),
            "surge": PID(0.8, 0.03, 0.02, setpoint=0, output_limits=(-1.0, 1.0)),
            "lateral": PID(0.8, 0.03, 0.02, setpoint=0, output_limits=(-1.0, 1.0)),
        }

        # Start the main control thread
        self.running = True
        self.control_thread = threading.Thread(target=self.control_loop)
        self.control_thread.start()
        
        self.get_logger().info("RobotControl initialized with DVL-based EKF positioning")

    def ekf_pose_callback(self, msg):
        """EKF fused position callback from ArduSub (includes DVL + IMU + barometer)"""
        # This is the LOCAL_POSITION_NED message from ArduSub's EKF
        # Contains fused DVL, IMU, and barometer data
        # makes sure it can process the nan data when dvl not pinging
        if (not math.isnan(msg.pose.position.x) and 
            not math.isnan(msg.pose.position.y) and 
            not math.isnan(msg.pose.position.z)):
            
            with self.lock:
                self.position['x'] = msg.pose.position.x
                self.position['y'] = msg.pose.position.y
                self.position['z'] = msg.pose.position.z  # Fused depth (barometer + DVL altitude)
                self.position_valid = True
                self.last_position_time = time.time()
            
            if self.debug and time.time() % 5 < 0.1:  # Log position every 5 seconds
                self.get_logger().info(f"EKF Position: x={self.position['x']:.2f}, y={self.position['y']:.2f}, z={self.position['z']:.2f}")
        else:
            with self.lock:
                self.position_valid = False
            if self.debug:
                self.get_logger().warn("EKF position invalid (NaN values)")
        
        # Always update orientation (from IMU)
        q = msg.pose.orientation
        if not math.isnan(q.w):
            with self.lock:
                self.orientation['yaw'] = quaternion_to_yaw(q.x, q.y, q.z, q.w)
                
                self.orientation['pitch'] = math.asin(2.0 * (q.w * q.y - q.z * q.x))
                self.orientation['roll'] = math.atan2(2.0 * (q.w * q.x + q.y * q.z), 1.0 - 2.0 * (q.x * q.x + q.y * q.y))

    def ekf_velocity_callback(self, msg):
        """EKF fused velocity callback from ArduSub"""
        if (not math.isnan(msg.twist.linear.x) and  #twist is tyoe of logger
            not math.isnan(msg.twist.linear.y) and 
            not math.isnan(msg.twist.linear.z)):
            
            with self.lock:
                self.velocity['x'] = msg.twist.linear.x
                self.velocity['y'] = msg.twist.linear.y
                self.velocity['z'] = msg.twist.linear.z
                self.velocity_valid = True
            
            if self.debug and abs(self.velocity['x']) > 0.1 or abs(self.velocity['y']) > 0.1:
                self.get_logger().info(f"EKF Velocity: x={self.velocity['x']:.2f}, y={self.velocity['y']:.2f}, z={self.velocity['z']:.2f}")
        
    def check_position_timeout(self):
        """Check if position data is stale"""
        return (time.time() - self.last_position_time) < self.position_timeout
        
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
        
        if abs(lateral) > 0.01 or abs(forward) > 0.01 or abs(yaw) > 0.01:
            self.get_logger().info(f"Movement command: lateral={lateral:.2f}, forward={forward:.2f}, yaw={yaw:.2f}")            
    
    def get_current_depth(self):
        """Get current depth from ArduSub EKF (fused barometer + DVL altitude)"""
        with self.lock:
            return self.position['z']

    def get_current_position(self):
        """Get current position from ArduSub EKF (fused DVL + IMU)"""
        with self.lock:
            pos = self.position.copy()
            pos['yaw'] = self.orientation['yaw']
            pos['valid'] = self.position_valid and self.check_position_timeout()
        return pos
    
    def get_current_velocity(self):
        """Get current velocity from ArduSub EKF"""
        with self.lock:
            return self.velocity.copy()
    
    def control_loop(self):
        while rclpy.ok() and self.running:
            self.update_heading_control()
            time.sleep(0.05)

    def update_heading_control(self):
        """Send control commands appropriate for GUIDED mode"""
        with self.lock:
            # Check position validity
            position_ok = self.position_valid and self.check_position_timeout()
            
            # Initialize control outputs
            forward_cmd = 0.0
            lateral_cmd = 0.0
            yaw_cmd = 0.0
            depth_cmd = 0.0
            
            # Check if we have position targets (waypoint navigation)
            has_position_targets = (self.desired_point['x'] is not None or 
                                  self.desired_point['y'] is not None or
                                  self.desired_point['yaw'] is not None)
            
            if has_position_targets and position_ok:
                # Position control mode - use PositionTarget for GUIDED mode
                self.send_position_target()
                return
            elif not has_position_targets:
                # Velocity control mode using DVL-based EKF position
                current_pos = self.position
                
                # Movement command mode (direct velocity commands)
                forward_cmd = self.movement_command['forward']
                yaw_cmd = self.movement_command['yaw']
            else:
                # Position targets set but no valid position data
                if self.debug:
                    self.get_logger().warn("Position targets set but no valid DVL/EKF position data")
            
            # Depth control (always active when target is set)
            if self.desired_point['z'] is not None:
                current_depth = self.position['z']
                depth_error = self.desired_point['z'] - current_depth
                
                if self.max_descent_mode and depth_error > 0.1:
                    # Maximum descent rate
                    depth_cmd = 1.0  # Full downward velocity
                else:
                    # Normal PID depth control
                    depth_cmd = self.PIDs["depth"](depth_error)

        # Send velocity commands for GUIDED mode
        self.send_velocity_command(forward_cmd, lateral_cmd, depth_cmd, yaw_cmd)
    #option one
    def send_velocity_command(self, forward, lateral, vertical, yaw_rate):
        """Send velocity commands for GUIDED mode - use actual target velocities"""
        # Create Twist message for velocity control in GUIDED mode
        vel_cmd = Twist()
        
        # Convert normalized commands to actual velocity targets (m/s)
        max_forward_velocity = 2.0   # m/s - adjust based on your sub's max speed
        max_lateral_velocity = 1.0   # m/s - adjust based on thruster config
        max_vertical_velocity = 1.0  # m/s - depth change rate
        max_yaw_rate = 1.5          # rad/s - yaw rotation rate
        
        # Scale commands to actual velocities (full speed when active)
        if abs(forward) > 0.01:
            vel_cmd.linear.x = max_forward_velocity if forward > 0 else -max_forward_velocity
        else:
            vel_cmd.linear.x = 0.0
            
        if abs(lateral) > 0.01:
            vel_cmd.linear.y = max_lateral_velocity if lateral > 0 else -max_lateral_velocity  
        else:
            vel_cmd.linear.y = 0.0
            
        if abs(vertical) > 0.01:
            vel_cmd.linear.z = max_vertical_velocity if vertical > 0 else -max_vertical_velocity
        else:
            vel_cmd.linear.z = 0.0
        
        if abs(yaw_rate) > 0.01:
            vel_cmd.angular.z = max_yaw_rate if yaw_rate > 0 else -max_yaw_rate
        else:
            vel_cmd.angular.z = 0.0
            
        # Other axes (typically not used in subs)
        vel_cmd.angular.x = 0.0       # Roll rate
        vel_cmd.angular.y = 0.0       # Pitch rate

        # Debug logging
        if self.debug and (abs(vertical) > 0.01 or abs(forward) > 0.01 or abs(yaw_rate) > 0.01):
            position_ok = self.position_valid and self.check_position_timeout()
            current_depth = self.position['z']
            target_depth = self.desired_point['z'] if self.desired_point['z'] is not None else current_depth
            ekf_status = "EKF_OK" if position_ok else "EKF_STALE"
            vel = self.velocity
            self.get_logger().info(f"GUIDED Vel (FULL SPEED): depth={current_depth:.2f}→{target_depth:.2f} (Z:{vel_cmd.linear.z:.1f}m/s) "
                                 f"pos: x={self.position['x']:.1f},y={self.position['y']:.1f} vel: x={vel['x']:.2f},y={vel['y']:.2f} "
                                 f"cmd: F={vel_cmd.linear.x:.1f}m/s Y={vel_cmd.angular.z:.1f}rad/s ({ekf_status})")

        # Publish velocity command
        self.velocity_pub.publish(vel_cmd)

        #option 2
    def send_position_target(self):
        """Send position targets for waypoint navigation in GUIDED mode"""
        position_target = PositionTarget()
        position_target.header.stamp = self.get_clock().now().to_msg()
        position_target.header.frame_id = "base_link" #shows x,y,z position
        
        # Coordinate frame (body frame NED)
        position_target.coordinate_frame = PositionTarget.FRAME_LOCAL_NED #uses North, East, Down coordinates
        
        # Set what to control (position and yaw) and ignore velocity commands since pixhawk handels the position
        position_target.type_mask = (
            PositionTarget.IGNORE_VX | PositionTarget.IGNORE_VY | PositionTarget.IGNORE_VZ |
            PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ
        )
        
        # Set position targets (if specified)
        if self.desired_point['x'] is not None:
            position_target.position.x = self.desired_point['x']
        else:
            position_target.type_mask |= PositionTarget.IGNORE_PX #ignores if not relevant
            
        if self.desired_point['y'] is not None:
            position_target.position.y = self.desired_point['y'] 
        else:
            position_target.type_mask |= PositionTarget.IGNORE_PY
            
        if self.desired_point['z'] is not None:
            position_target.position.z = self.desired_point['z']
        else:
            position_target.type_mask |= PositionTarget.IGNORE_PZ
            
        # Set yaw target (if specified)
        if self.desired_point['yaw'] is not None:
            position_target.yaw = self.desired_point['yaw']
        else:
            position_target.type_mask |= PositionTarget.IGNORE_YAW
            
        # Publish position target
        self.position_target_pub.publish(position_target)
        
        if self.debug:
            self.get_logger().info(f"GUIDED Position Target: x={position_target.position.x:.2f}, "
                                 f"y={position_target.position.y:.2f}, z={position_target.position.z:.2f}, "
                                 f"yaw={position_target.yaw:.2f}")
        #option 3
    def send_manual_control(self, forward, lateral, vertical, yaw_rate):
        """Send manual control commands (for MANUAL mode fallback) - FULL SPEED WHEN ACTIVE"""
        manual_cmd = ManualControl()
        manual_cmd.header.stamp = self.get_clock().now().to_msg()
        
        # For your PWM requirements: 1100=full back, 1500=off, 1900=full forward
        # ManualControl expects -1000 to 1000, which maps to your 1100-1900 range
        
        # Full speed when active (binary on/off control)
        if abs(forward) > 0.01:
            manual_cmd.x = 1000 if forward > 0 else -1000  # Full forward (1900) or full back (1100)
        else:
            manual_cmd.x = 0  # Off (1500)
            
        if abs(lateral) > 0.01:
            manual_cmd.y = 1000 if lateral > 0 else -1000  # Full lateral
        else:
            manual_cmd.y = 0  # Off
            
        if abs(vertical) > 0.01:
            manual_cmd.z = 1000 if vertical > 0 else -1000  # Full up/down (1900/1100)
        else:
            manual_cmd.z = 0  # Off (1500)
            
        if abs(yaw_rate) > 0.01:
            manual_cmd.r = 1000 if yaw_rate > 0 else -1000  # Full yaw rotation
        else:
            manual_cmd.r = 0  # Off
        
        # Values are already at max (-1000/1000), so no clamping needed
        
        if self.debug:
            pwm_x = int(manual_cmd.x * 0.4 + 1500)  # Convert back to actual PWM for logging
            pwm_z = int(manual_cmd.z * 0.4 + 1500) 
            pwm_r = int(manual_cmd.r * 0.4 + 1500)
            self.get_logger().info(f"MANUAL Control (FULL SPEED): "
                                 f"Forward: {manual_cmd.x} → PWM {pwm_x}, "
                                 f"Depth: {manual_cmd.z} → PWM {pwm_z}, "
                                 f"Yaw: {manual_cmd.r} → PWM {pwm_r}")
        
        # Publish manual control command
        self.manual_control_pub.publish(manual_cmd)

    def stop(self):
        """Stop the control system"""
        self.running = False
        
        # Send stop commands for GUIDED mode
        stop_vel = Twist()
        stop_vel.linear.x = 0.0
        stop_vel.linear.y = 0.0
        stop_vel.linear.z = 0.0
        stop_vel.angular.x = 0.0
        stop_vel.angular.y = 0.0
        stop_vel.angular.z = 0.0
        self.velocity_pub.publish(stop_vel)
        
        # Also send neutral manual control (for safety)
        manual_cmd = ManualControl()
        manual_cmd.header.stamp = self.get_clock().now().to_msg()
        manual_cmd.x = 0
        manual_cmd.y = 0
        manual_cmd.z = 0
        manual_cmd.r = 0
        self.manual_control_pub.publish(manual_cmd)
        
        if self.control_thread.is_alive():
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