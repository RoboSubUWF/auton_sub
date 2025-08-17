# ROS 2 version of RobotControl using MAVROS Vision Topics (from DVL bridge)

import rclpy #for ros2
from rclpy.node import Node 
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, TwistStamped, PoseWithCovarianceStamped #message definitions for representing geometric data
from mavros_msgs.msg import ManualControl, AttitudeTarget, PositionTarget #from robotcontrol
from mavros_msgs.srv import SetMode, CommandBool #for pixhawk modes
from geometry_msgs.msg import Twist 
from nav_msgs.msg import Odometry #from dvl
from sensor_msgs.msg import Range # from dvl
from std_srvs.srv import Trigger #for signaling or triggering actions
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy #for qos stuff
from rclpy.qos import qos_profile_system_default
from simple_pid import PID #to use library for pid
import numpy as np 
import math
import time
import threading

# Create QoS profile that matches MAVROS (could probably be inside function instead of calling if separately
def create_mavros_qos():
    """Create QoS profile compatible with MAVROS topics"""
    qos = QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,  # MAVROS vision topics use RELIABLE
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=10
    )
    return qos
#calculates yaw
def quaternion_to_yaw(x, y, z, w):
    """Convert quaternion to yaw angle (in radians)"""
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return yaw

class RobotControl(Node):
    def __init__(self):
        super().__init__('robot_control')
        # In your RobotControl.__init__(), add this at the very beginning:
        time.sleep(5.0)  # Wait for MAVROS to fully initialize
        self.get_logger().info("🔄 Waiting for MAVROS to initialize...")
        
        self.lock = threading.Lock() #prevents multiple services from calling the same things at the same time.
        self.debug = True  # Enable debug info

        # Robot state - Use MAVROS Vision Topics (from DVL bridge)
        self.position = {'x': 0.0, 'y': 0.0, 'z': 0.0}  # DVL position via MAVROS vision
        self.velocity = {'x': 0.0, 'y': 0.0, 'z': 0.0}  # DVL velocity via MAVROS vision
        self.orientation = {'yaw': 0.0, 'pitch': 0.0, 'roll': 0.0}  # IMU orientation
        self.range_to_bottom = float('nan')  # DVL range sensor
        self.desired_point = {'x': None, 'y': None, 'z': None, 'yaw': None} #initialized desired point
        self.movement_command = {'lateral': 0.0, 'forward': 0.0, 'yaw': 0.0} #initialized movement commands
        self.max_descent_mode = False  #ensures max descent is off until needed. 
        self.mode = "guided" #

        # Status flags for MAVROS Vision data (from DVL bridge)
        self.vision_pose_valid = False
        self.vision_velocity_valid = False
        self.imu_valid = False
        self.last_vision_time = time.time()
        self.last_imu_time = time.time()
        self.data_timeout = 3.0  # seconds

        # QoS profiles
        mavros_qos = create_mavros_qos()
        dvl_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        
        # MAVROS Vision Topic Subscribers (from DVL bridge output)
        self.vision_pose_sub = self.create_subscription(
            PoseStamped,
            '/mavros/vision_pose/pose',  # DVL position via bridge
            self.mavros_pose_callback,
            mavros_qos
        )
        
        self.vision_speed_sub = self.create_subscription(
            TwistStamped,
            '/mavros/vision_speed/speed_twist',  # DVL velocity via bridge
            self.mavros_velocity_callback,
            mavros_qos
        )
        
        # Optional: Direct DVL range subscription (if available)
        self.dvl_range_sub = self.create_subscription(
            Range,
            'dvl/range',
            self.dvl_range_callback,
            dvl_qos
        )
        
        # IMU data for orientation (still from MAVROS directly)
        self.imu_sub = self.create_subscription(
            PoseStamped, 
            '/mavros/local_position/pose', 
            self.imu_orientation_callback, 
            qos_profile_system_default
        )
        
        # Publishers for GUIDED mode control
        self.velocity_pub = self.create_publisher(Twist, '/mavros/setpoint_velocity/cmd_vel_unstamped', 10)
        self.position_target_pub = self.create_publisher(PositionTarget, '/mavros/setpoint_raw/local', 10)
        self.manual_control_pub = self.create_publisher(ManualControl, '/mavros/manual_control/control', 10)

        # PID controllers - tuned for DVL-based control via MAVROS vision
        self.PIDs = {   
            "yaw": PID(0.8, .001, 0.05, setpoint=0, output_limits=(-0.5, 0.5)),
            "depth": PID(1.2, 0.05, 0.1, setpoint=0, output_limits=(-0.5, 0.5)),
            "surge": PID(0.5, 0.01, 0.02, setpoint=0, output_limits=(-1.0, 1.0)),
            "lateral": PID(0.5, 0, 0.02, setpoint=0, output_limits=(-1.0, 1.0)),
        }

        # Start the main control thread
        self.running = True
        self.control_thread = threading.Thread(target=self.control_loop)
        self.control_thread.start()
        
        self.get_logger().info("RobotControl initialized with MAVROS Vision Topics (DVL via Bridge)")

    def mavros_pose_callback(self, msg: PoseStamped):
        """MAVROS vision pose callback - DVL position data via bridge"""
        
        try:
            self.get_logger().info("Pose CB")
            with self.lock:
                # Position from MAVROS vision pose (DVL data via bridge)
                self.position['x'] = msg.pose.position.x
                self.position['y'] = msg.pose.position.y
                self.position['z'] = msg.pose.position.z
                
                # Mark pose data as valid
                self.vision_pose_valid = True
                self.last_vision_time = time.time()
            
            if self.debug and time.time() % 3 < 0.1:  # Log every 3 seconds
                self.get_logger().info(f"MAVROS Vision Pose (DVL): x={self.position['x']:.3f}m, y={self.position['y']:.3f}m, z={self.position['z']:.3f}m")
                
        except Exception as e:
            self.get_logger().error(f"MAVROS vision pose callback error: {e}")

    def mavros_velocity_callback(self, msg: TwistStamped):
        """MAVROS vision speed callback - DVL velocity data via bridge"""
        try:
            with self.lock:
                # Velocity from MAVROS vision speed (DVL data via bridge)
                self.velocity['x'] = msg.twist.linear.x
                self.velocity['y'] = msg.twist.linear.y
                self.velocity['z'] = msg.twist.linear.z
                
                # Mark velocity data as valid
                self.vision_velocity_valid = True
                self.last_vision_time = time.time()
            
            # Log velocity changes (only when significant movement)
            if self.debug and (abs(self.velocity['x']) > 0.05 or abs(self.velocity['y']) > 0.05 or abs(self.velocity['z']) > 0.05):
                self.get_logger().info(f"MAVROS Vision Speed (DVL): x={self.velocity['x']:.3f}m/s, y={self.velocity['y']:.3f}m/s, z={self.velocity['z']:.3f}m/s")
                
        except Exception as e:
            self.get_logger().error(f"MAVROS vision speed callback error: {e}")

    def dvl_range_callback(self, msg):
        """DVL range to bottom callback (direct from DVL if available)"""
        try:
            with self.lock:
                self.range_to_bottom = msg.range
            
            if self.debug and not math.isnan(self.range_to_bottom) and time.time() % 5 < 0.1:
                self.get_logger().info(f"DVL Range to bottom: {self.range_to_bottom:.2f}m")
                
        except Exception as e:
            self.get_logger().error(f"DVL range callback error: {e}")

    def imu_orientation_callback(self, msg):
        """IMU orientation callback - still need this for heading control"""
        try:
            q = msg.pose.orientation
            if not math.isnan(q.w):
                with self.lock:
                    self.orientation['yaw'] = quaternion_to_yaw(q.x, q.y, q.z, q.w)
                    self.orientation['pitch'] = math.asin(2.0 * (q.w * q.y - q.z * q.x))
                    self.orientation['roll'] = math.atan2(2.0 * (q.w * q.x + q.y * q.z), 1.0 - 2.0 * (q.x * q.x + q.y * q.y))
                    self.imu_valid = True
                    self.last_imu_time = time.time()
                    
        except Exception as e:
            self.get_logger().error(f"IMU orientation callback error: {e}")

    def check_vision_data_timeout(self):
        """Check if MAVROS vision data is stale"""
        return (time.time() - self.last_vision_time) < self.data_timeout
        
    def check_imu_data_timeout(self):
        """Check if IMU data is stale"""
        return (time.time() - self.last_imu_time) < self.data_timeout
        
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
        """Get current depth from MAVROS vision pose (DVL data via bridge)"""
        with self.lock:
            return float(self.position['z'])

    def get_current_position(self):
        """Get current position from MAVROS vision topics (DVL data via bridge)"""
        with self.lock:
            pos = self.position.copy()
            pos['yaw'] = self.orientation['yaw']
            pos['valid'] = (self.vision_pose_valid and self.check_vision_data_timeout())
        return pos
    
    def get_current_velocity(self):
        """Get current velocity from MAVROS vision speed (DVL data via bridge)"""
        with self.lock:
            vel = self.velocity.copy()
            vel['valid'] = self.vision_velocity_valid and self.check_vision_data_timeout()
        return vel
    
    def control_loop(self):
        while rclpy.ok() and self.running:
            self.update_heading_control()
            time.sleep(0.05)

    def update_heading_control(self):
        """Send control commands appropriate for GUIDED mode - FIXED logic"""
        with self.lock:
            # Check data validity from MAVROS vision topics
            vision_pose_ok = self.vision_pose_valid and self.check_vision_data_timeout()
            vision_speed_ok = self.vision_velocity_valid and self.check_vision_data_timeout()
            imu_ok = self.imu_valid and self.check_imu_data_timeout()
            
            # Initialize control outputs
            forward_cmd = 0.0
            lateral_cmd = 0.0
            yaw_cmd = 0.0
            depth_cmd = 0.0
            
            # FIXED: Always use velocity control mode for submarine operations
            # Get movement commands (these should always be used)
            forward_cmd = self.movement_command['forward']
            lateral_cmd = self.movement_command['lateral'] 
            yaw_cmd = self.movement_command['yaw']
            
            # Depth control (always active when target is set) - FIXED coordinate frame
            if self.desired_point['z'] is not None and vision_pose_ok:
                current_depth = self.position['z']  # Use direct z value from DVL bridge
                depth_error = self.desired_point['z'] - current_depth
                
                # Log depth status for debugging
                if abs(depth_error) > 0.02:  # Only log when there's meaningful error
                    self.get_logger().info(f"DEPTH CONTROL: Current={current_depth:.3f}m, Target={self.desired_point['z']:.3f}m, Error={depth_error:.3f}m")

                if self.max_descent_mode and depth_error > 0.1:  # FIXED: descent means going down (negative z)
                    depth_cmd = -0.6  # Negative for downward movement
                    self.get_logger().info(f"MAX DESCENT MODE: depth_cmd={depth_cmd}")
                else:
                    depth_cmd = self.PIDs["depth"](depth_error)
                    if abs(depth_cmd) > 0.02:  # Only log significant depth commands
                        self.get_logger().info(f"PID DEPTH: depth_cmd={depth_cmd:.3f}")

            # FIXED: Always send velocity commands, don't skip based on position targets
            self.send_velocity_command(forward_cmd, lateral_cmd, depth_cmd, yaw_cmd)
    def send_velocity_command(self, forward, lateral, vertical, yaw_rate):
        """FIXED velocity command logic"""
        vel_cmd = Twist()

        # FIXED velocity limits for submarine
        max_forward_velocity = 1.0
        max_lateral_velocity = 0.5
        max_vertical_velocity = -0.8
        max_yaw_rate = 0.5  # Reduced for stability

        # Forward/backward - FIXED logic
        if abs(forward) > 0.01:
            vel_cmd.linear.x = forward * max_forward_velocity  # Direct mapping
            self.get_logger().info(f"FORWARD COMMAND: input={forward:.2f}, output={vel_cmd.linear.x:.2f}")
        else:
            vel_cmd.linear.x = 0.0

        # Lateral (left/right)
        if abs(lateral) > 0.01:
            vel_cmd.linear.y = lateral * max_lateral_velocity
        else:
            vel_cmd.linear.y = 0.0

        # Vertical - FIXED logic for submarine depth control
        if abs(vertical) > 0.01:
            vel_cmd.linear.z = vertical * max_vertical_velocity  # Direct mapping
            self.get_logger().info(f"VERTICAL COMMAND: input={vertical:.2f}, output={vel_cmd.linear.z:.2f}")
        else:
            min_vertical_thrust = 0.2 * max_vertical_velocity
            vel_cmd.linear.z = min_vertical_thrust if vertical > 0 else -min_vertical_thrust
            self.get_logger().info(f"MINIMAL VERTICAL: input={vertical:.3f}, output={vel_cmd.linear.z:.2f} (min thrust)")

        # Yaw - FIXED logic
        if abs(yaw_rate) > 0.01:
            vel_cmd.angular.z = 0.0
        else:
            vel_cmd.angular.z = 0.0

        vel_cmd.angular.x = 0.0
        vel_cmd.angular.y = 0.0

        # Log all non-zero commands for debugging
        if (abs(vel_cmd.linear.x) > 0.01 or abs(vel_cmd.linear.y) > 0.01 or 
            abs(vel_cmd.linear.z) > 0.01 or abs(vel_cmd.angular.z) > 0.01):
            self.get_logger().info(f"VELOCITY CMD: x={vel_cmd.linear.x:.2f}, y={vel_cmd.linear.y:.2f}, "
                                 f"z={vel_cmd.linear.z:.2f}, yaw={vel_cmd.angular.z:.2f}")

        self.velocity_pub.publish(vel_cmd)

    def stop(self):
        """Stop the robot and cleanup"""
        self.get_logger().info("Stopping robot control...")
        self.running = False
        if hasattr(self, 'control_thread'):
            self.control_thread.join(timeout=2.0)
        
        # Send zero velocity commands to stop
        vel_cmd = Twist()
        for _ in range(5):  # Send multiple stop commands
            self.velocity_pub.publish(vel_cmd)
            time.sleep(0.1)

    def send_position_target(self):
        """Send position targets for waypoint navigation in GUIDED mode"""
        position_target = PositionTarget()
        position_target.header.stamp = self.get_clock().now().to_msg()
        position_target.header.frame_id = "odom"
        
        # Coordinate frame (body frame NED)
        position_target.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        
        # Set what to control (position and yaw) and ignore velocity commands
        position_target.type_mask = (
            PositionTarget.IGNORE_VX | PositionTarget.IGNORE_VY | PositionTarget.IGNORE_VZ |
            PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ
        )
        
        # Set position targets (if specified) - using MAVROS vision pose data
        if self.desired_point['x'] is not None:
            position_target.position.x = self.desired_point['x']
        else:
            position_target.type_mask |= PositionTarget.IGNORE_PX
            
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
            self.get_logger().info(f"GUIDED Position Target (MAVROS-VISION): x={position_target.position.x:.2f}, "
                                 f"y={position_target.position.y:.2f}, z={position_target.position.z:.2f}, "
                                 f"yaw={position_target.yaw:.2f}")

    def send_manual_control(self, forward, lateral, vertical, yaw_rate):
        """Send manual control commands (for MANUAL mode fallback)"""
        manual_cmd = ManualControl()
        manual_cmd.header.stamp = self.get_clock().now().to_msg()
        
        # Convert to float values for ROS message fields
        # ManualControl expects -1000.0 to 1000.0 (float type)
        
        # Full speed when active (binary on/off control)
        if abs(forward) > 0.01:
            manual_cmd.x = float(1000 if forward > 0 else -1000)  # Ensure float type
        else:
            manual_cmd.x = float(0)  # Off
            
        if abs(lateral) > 0.01:
            manual_cmd.y = float(1000 if lateral > 0 else -1000)  # Ensure float type
        else:
            manual_cmd.y = float(0)  # Off
            
        if abs(vertical) > 0.01:
            manual_cmd.z = float(1000 if vertical > 0 else -1000)  # Ensure float type
        else:
            manual_cmd.z = float(0)  # Off
            
        if abs(yaw_rate) > 0.01:
            manual_cmd.r = float(1000 if yaw_rate > 0 else -1000)  # Ensure float type
        else:
            manual_cmd.r = float(0)  # Off
        
        if self.debug:
            pwm_x = int(manual_cmd.x * 0.4 + 1500)  # Convert back to actual PWM for logging
            pwm_z = int(manual_cmd.z * 0.4 + 1500) 
            pwm_r = int(manual_cmd.r * 0.4 + 1500)
            self.get_logger().info(f"MANUAL Control (FULL SPEED): "
                                f"Forward: {manual_cmd.x} ? PWM {pwm_x}, "
                                f"Depth: {manual_cmd.z} ? PWM {pwm_z}, "
                                f"Yaw: {manual_cmd.r} ? PWM {pwm_r}")
        
        # Publish manual control command
        self.manual_control_pub.publish(manual_cmd)


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
