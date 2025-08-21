#this integration file was given by the dvl company and converted to a ros2 node. You probably dont need to change anything here.
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import TwistStamped, TwistWithCovarianceStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import FluidPressure, Range
import numpy as np
import math
import datetime as dt
import time
import threading
import json
import socket
import csv

# Try to import Teledyne driver, fall back to TCP if not available
try:
    from dvl.dvl import Dvl
    from dvl.system import OutputData
    TELEDYNE_DRIVER_AVAILABLE = True
except ImportError:
    TELEDYNE_DRIVER_AVAILABLE = False

class DVLNode(Node):
    def __init__(self):
        super().__init__('dvl_node')
        
        # Parameters
        self.declare_parameter('connection_type', 'serial')  # 'serial' or 'tcp'
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('tcp_ip', '192.168.2.10')
        self.declare_parameter('tcp_port', 14777)
        self.declare_parameter('csv_logging', False)
        self.declare_parameter('csv_file_path', '/tmp/dvl_data.csv')
        self.declare_parameter('frame_id', 'base_link')
        self.declare_parameter('publish_rate', 10.0)  # Hz
        
        # QoS Profile for MAVROS compatibility
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE
        )
        
        # Publishers - Updated for MAVROS integration
        self.velocity_pub = self.create_publisher(
            TwistStamped, 
            'dvl/velocity', 
            qos_profile
        )
        
        # MAVROS-compatible velocity publisher
        self.velocity_cov_pub = self.create_publisher(
            TwistWithCovarianceStamped, 
            'dvl/velocity_with_covariance', 
            qos_profile
        )
        
        # MAVROS-compatible position publisher  
        self.position_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            'dvl/position',
            qos_profile
        )
        
        self.odometry_pub = self.create_publisher(
            Odometry, 
            'dvl/odometry', 
            qos_profile
        )
        
        # Range to bottom publisher
        self.range_pub = self.create_publisher(
            Range,
            'dvl/range',
            qos_profile
        )
        
        # Configuration
        self.connection_type = self.get_parameter('connection_type').get_parameter_value().string_value
        self.wayfinderPort = self.get_parameter('serial_port').get_parameter_value().string_value
        self.tcp_ip = self.get_parameter('tcp_ip').get_parameter_value().string_value
        self.tcp_port = self.get_parameter('tcp_port').get_parameter_value().integer_value
        self.csv_logging = self.get_parameter('csv_logging').get_parameter_value().bool_value
        self.csv_file_path = self.get_parameter('csv_file_path').get_parameter_value().string_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        
        # Sensor objects
        self.sensor = None
        self.tcp_socket = None
        self.csv_writer = None
        
        # Coordinate frame transformation
        self.roll = np.radians(180)
        self.pitch = np.radians(0)
        self.yaw = np.radians(135)
        self.setup_transform()
        
        # Position tracking with better initialization
        self.position_x = 0.0
        self.position_y = 0.0
        self.position_z = 0.0
        self.last_time = None
        
        # Velocity covariance estimation
        self.velocity_samples = []
        self.max_samples = 10
        
        # Data quality tracking
        self.last_valid_data_time = time.time()
        self.data_timeout = 5.0  # seconds
        
        # Initialize CSV logging if requested
        if self.csv_logging:
            self.setup_csv_logging()
        
        # Initialize DVL based on connection type
        if self.connection_type == 'serial':
            if not TELEDYNE_DRIVER_AVAILABLE:
                self.get_logger().error("❌ Teledyne driver not available, cannot use serial connection")
                return
            self.setup_dvl_serial()
        elif self.connection_type == 'tcp':
            self.setup_dvl_tcp()
        else:
            self.get_logger().error(f"❌ Unknown connection type: {self.connection_type}")
            return
        
        # Health monitoring timer
        self.health_timer = self.create_timer(1.0, self.check_health)
        
        self.get_logger().info("✅ DVL Node initialized and ready!")
    
    def setup_transform(self):
        """Setup coordinate transformation matrix"""
        cos_yaw = np.cos(self.yaw)
        sin_yaw = np.sin(self.yaw)
        cos_pitch = np.cos(self.pitch)
        sin_pitch = np.sin(self.pitch)
        cos_roll = np.cos(self.roll)
        sin_roll = np.sin(self.roll)
        
        self.rot_matrix = np.array([
            [cos_yaw * cos_pitch, cos_yaw * sin_pitch * sin_roll - sin_yaw * cos_roll, cos_yaw * sin_pitch * cos_roll + sin_yaw * sin_roll],
            [sin_yaw * cos_pitch, sin_yaw * sin_pitch * sin_roll + cos_yaw * cos_roll, sin_yaw * sin_pitch * cos_roll - cos_yaw * sin_roll],
            [-sin_pitch, cos_pitch * sin_roll, cos_pitch * cos_roll]
        ])
    
    def setup_csv_logging(self):
        """Setup CSV logging for data analysis"""
        try:
            self.csv_file = open(self.csv_file_path, 'w', newline='')
            fieldnames = ['timestamp', 'vx', 'vy', 'vz', 'fom', 'velocity_valid', 'status', 'range_to_bottom']
            self.csv_writer = csv.DictWriter(self.csv_file, fieldnames=fieldnames)
            self.csv_writer.writeheader()
            self.get_logger().info(f"✅ CSV logging enabled: {self.csv_file_path}")
        except Exception as e:
            self.get_logger().error(f"❌ Failed to setup CSV logging: {e}")
            self.csv_logging = False
    
    def setup_dvl_serial(self):
        """Initialize DVL via serial connection (original method)"""
        try:
            self.sensor = Dvl()
            self.get_logger().info("🔄 Connecting to DVL via serial...")
            while not self.sensor.connect(self.wayfinderPort, 115200):
                self.get_logger().warn("⚠️ Failed to connect to DVL, retrying...")
                time.sleep(1)
            
            self.get_logger().info("✅ Connected to DVL via serial!")
            
            # Reset and configure
            self.sensor.reset_to_defaults()
            self.sensor.enter_command_mode()
            twoseconds = dt.datetime.now() + dt.timedelta(seconds=2)
            timetarget = dt.datetime(twoseconds.year, twoseconds.month, twoseconds.day, 
                                   twoseconds.hour, twoseconds.minute, twoseconds.second)
            now = dt.datetime.now()
            time.sleep((timetarget - now).total_seconds())
            self.sensor.set_time(dt.datetime.now())
            self.sensor.exit_command_mode()
            
            # Register callback
            self.sensor.register_ondata_callback(self.dvl_serial_callback, None)
            
        except Exception as e:
            self.get_logger().error(f"❌ Serial DVL setup failed: {e}")
            return False
        
        return True
    
    def setup_dvl_tcp(self):
        """Initialize DVL via TCP connection"""
        try:
            self.get_logger().info(f"🔄 Connecting to DVL via TCP at {self.tcp_ip}:{self.tcp_port}...")
            self.tcp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.tcp_socket.connect((self.tcp_ip, self.tcp_port))
            self.get_logger().info("✅ Connected to DVL via TCP!")
            
            # Start TCP processing in separate thread
            self.tcp_thread = threading.Thread(target=self.process_tcp_messages, daemon=True)
            self.tcp_thread.start()
            
        except Exception as e:
            self.get_logger().error(f"❌ TCP DVL setup failed: {e}")
            return False
        
        return True
    
    def process_tcp_messages(self):
        """Process TCP messages (adapted from the TCP parser)"""
        buffer_size = 4096
        message = ""
        
        while rclpy.ok():
            try:
                buffer = self.tcp_socket.recv(buffer_size).decode()
                if not buffer:
                    continue
                
                message_parts = buffer.split("\r\n")
                if len(message_parts) == 1:
                    message += message_parts[0]
                    continue
                
                for message_part in message_parts[:-1]:
                    message = message + message_part
                    self.handle_tcp_message(message)
                    message = ""
                
                if message_parts[-1]:
                    message = message_parts[-1]
                    
            except Exception as e:
                self.get_logger().error(f"❌ TCP processing error: {e}")
                break
    
    def handle_tcp_message(self, message):
        """Handle individual TCP message"""
        if not message:
            return
        
        try:
            report = json.loads(message)
        except json.decoder.JSONDecodeError:
            self.get_logger().debug(f"Could not parse JSON: {message}")
            return
        
        if report.get("type") != "velocity":
            return
        
        # Add log time
        report["log_time"] = int(time.time() * 1e6)
        
        # Process as DVL data
        self.dvl_tcp_callback(report)
    
    def calculate_velocity_covariance(self, current_vel):
    #"""Calculate velocity covariance based on recent measurements"""
    # Ensure current_vel is clean
        if np.any(np.isnan(current_vel)) or np.any(np.isinf(current_vel)):
            self.get_logger().warn("⚠️ Invalid velocity data in covariance calculation")
            return self.get_default_covariance()
    
        self.velocity_samples.append(current_vel.copy())
        if len(self.velocity_samples) > self.max_samples:
            self.velocity_samples.pop(0)
    
        if len(self.velocity_samples) < 3:
            # Default covariance for insufficient samples
            return self.get_default_covariance()
    
        try:
            # Calculate sample covariance
            samples = np.array(self.velocity_samples)
        
            # Check for any NaN or inf in samples
            if np.any(np.isnan(samples)) or np.any(np.isinf(samples)):
                self.get_logger().warn("⚠️ Invalid samples in covariance calculation")
                return self.get_default_covariance()
        
            cov_matrix = np.cov(samples.T)
        
            # Ensure minimum covariance and handle potential issues
            min_var = 0.01  # 0.1 m/s std dev minimum
            if cov_matrix.shape == ():  # Single value case
                cov_matrix = np.array([[cov_matrix, 0, 0], [0, cov_matrix, 0], [0, 0, cov_matrix]])
            elif cov_matrix.shape == (3,):  # 1D case
                cov_matrix = np.diag(cov_matrix)
        
            # Ensure it's a proper 3x3 matrix
            if cov_matrix.shape != (3, 3):
                return self.get_default_covariance()
        
            # Check for NaN or inf in covariance matrix
            if np.any(np.isnan(cov_matrix)) or np.any(np.isinf(cov_matrix)):
                return self.get_default_covariance()
        
            np.fill_diagonal(cov_matrix, np.maximum(np.diag(cov_matrix), min_var))
        
            # Convert to ROS covariance format (6x6 matrix flattened)
            ros_cov = np.zeros((6, 6))
            ros_cov[:3, :3] = cov_matrix
            ros_cov[3:, 3:] = np.eye(3) * 0.01  # Angular velocity covariance
        
            #   Ensure all values are finite floats
            covariance_flat = ros_cov.flatten()
            covariance_flat = np.nan_to_num(covariance_flat, nan=0.1, posinf=1.0, neginf=1.0)
        
            # Convert to Python floats (important for ROS message compatibility)
            return [float(x) for x in covariance_flat]
        
        except Exception as e:
            self.get_logger().error(f"❌ Covariance calculation error: {e}")
            return self.get_default_covariance()
    def get_default_covariance(self):
        #"""Return a safe default covariance matrix"""
        ros_cov = np.zeros((6, 6))
        # Conservative default values
        ros_cov[0, 0] = 0.1  # x velocity variance
        ros_cov[1, 1] = 0.1  # y velocity variance  
        ros_cov[2, 2] = 0.1  # z velocity variance
        ros_cov[3, 3] = 0.01  # x angular velocity variance
        ros_cov[4, 4] = 0.01  # y angular velocity variance
        ros_cov[5, 5] = 0.01  # z angular velocity variance
    
        # Convert to Python floats
        return [float(x) for x in ros_cov.flatten()]
    
    def dvl_serial_callback(self, data_obj: OutputData, *args):
        """Process DVL data from serial connection"""
        try:
            # Check validity
            if (math.isnan(data_obj.vel_x) or math.isnan(data_obj.vel_y) or 
                math.isnan(data_obj.vel_z) or math.isnan(data_obj.vel_err)):
                self.get_logger().warn("⚠️ Invalid DVL velocities (NaN)")
                return
            
            if not data_obj.is_velocity_valid():
                self.get_logger().warn("⚠️ DVL reports invalid velocities")
                return
            
            # Get velocities and range
            raw_vels = np.array([data_obj.vel_x, data_obj.vel_y, data_obj.vel_z])
            range_to_bottom = getattr(data_obj, 'range_to_bottom', float('nan'))
            
            # Update data health
            self.last_valid_data_time = time.time()
            
            # CSV logging
            if self.csv_logging and self.csv_writer:
                csv_row = {
                    'timestamp': time.time(),
                    'vx': data_obj.vel_x,
                    'vy': data_obj.vel_y,
                    'vz': data_obj.vel_z,
                    'fom': data_obj.vel_err,
                    'velocity_valid': data_obj.is_velocity_valid(),
                    'status': getattr(data_obj, 'status', 0),
                    'range_to_bottom': range_to_bottom
                }
                self.csv_writer.writerow(csv_row)
                self.csv_file.flush()
            
            # Process data
            self.process_velocity_data(raw_vels, data_obj.vel_err, range_to_bottom)
            
        except Exception as e:
            self.get_logger().error(f"❌ Serial DVL callback error: {e}")
    
    def dvl_tcp_callback(self, data_dict):
        """Process DVL data from TCP connection"""
        try:
            # Extract velocities from JSON
            vx = data_dict.get('vx', 0.0)
            vy = data_dict.get('vy', 0.0)
            vz = data_dict.get('vz', 0.0)
            fom = data_dict.get('fom', float('inf'))
            velocity_valid = data_dict.get('velocity_valid', False)
            range_to_bottom = data_dict.get('range_to_bottom', float('nan'))
            
            if not velocity_valid:
                self.get_logger().warn("⚠️ TCP DVL reports invalid velocities")
                return
            
            raw_vels = np.array([vx, vy, vz])
            
            # Update data health
            self.last_valid_data_time = time.time()
            
            # CSV logging
            if self.csv_logging and self.csv_writer:
                csv_row = {
                    'timestamp': time.time(),
                    'vx': vx,
                    'vy': vy,
                    'vz': vz,
                    'fom': fom,
                    'velocity_valid': velocity_valid,
                    'status': data_dict.get('status', 0),
                    'range_to_bottom': range_to_bottom
                }
                self.csv_writer.writerow(csv_row)
                self.csv_file.flush()
            
            # Process data
            self.process_velocity_data(raw_vels, fom, range_to_bottom)
            
        except Exception as e:
            self.get_logger().error(f"❌ TCP DVL callback error: {e}")
    
    def process_velocity_data(self, raw_velocities, fom, range_to_bottom):
        """Common velocity processing for both connection types"""
        # Transform to vehicle frame
        vels = np.matmul(self.rot_matrix, raw_velocities)
        
        # Get timestamp
        timestamp = self.get_clock().now().to_msg()
        
        # Publish all data
        self.publish_velocity(vels, timestamp)
        self.publish_velocity_with_covariance(vels, fom, timestamp)
        self.publish_range(range_to_bottom, timestamp)
        
        # Update and publish odometry
        self.update_odometry(vels, timestamp)
        
        # Log data (less frequently to avoid spam)
        if time.time() % 2 < 0.1:  # Log every ~2 seconds
            self.get_logger().info(f"DVL: V=[{vels[0]:.3f}, {vels[1]:.3f}, {vels[2]:.3f}] m/s, FOM={fom:.3f}, Range={range_to_bottom:.2f}m")
    
    def publish_velocity(self, velocities, timestamp):
        """Publish velocity data as TwistStamped"""
        msg = TwistStamped()
        msg.header.stamp = timestamp
        msg.header.frame_id = self.frame_id
        
        msg.twist.linear.x = float(velocities[0])
        msg.twist.linear.y = float(velocities[1])
        msg.twist.linear.z = float(velocities[2])
        
        self.velocity_pub.publish(msg)
    
    def publish_velocity_with_covariance(self, velocities, fom, timestamp):
        #"""Publish velocity with covariance for MAVROS integration"""
        msg = TwistWithCovarianceStamped()
        msg.header.stamp = timestamp
        msg.header.frame_id = self.frame_id
    
        # Ensure velocities are finite
        safe_velocities = np.nan_to_num(velocities, nan=0.0, posinf=0.0, neginf=0.0)
    
        msg.twist.twist.linear.x = float(safe_velocities[0])
        msg.twist.twist.linear.y = float(safe_velocities[1])
        msg.twist.twist.linear.z = float(safe_velocities[2])
    
        # Calculate covariance - this now returns a list of 36 floats
        covariance_list = self.calculate_velocity_covariance(safe_velocities)
    
        # Verify covariance format
        if len(covariance_list) != 36:
            self.get_logger().error(f"❌ Invalid covariance length: {len(covariance_list)}")
            covariance_list = self.get_default_covariance()
    
        msg.twist.covariance = covariance_list
    
        self.velocity_cov_pub.publish(msg)
    
    def publish_range(self, range_to_bottom, timestamp):
        """Publish range to bottom"""
        if math.isnan(range_to_bottom) or range_to_bottom <= 0:
            return
        
        msg = Range()
        msg.header.stamp = timestamp
        msg.header.frame_id = self.frame_id
        msg.radiation_type = Range.ULTRASOUND
        msg.field_of_view = 0.1  # Beam angle (adjust for your DVL)
        msg.min_range = 0.1
        msg.max_range = 100.0
        msg.range = float(range_to_bottom)
        
        self.range_pub.publish(msg)
    
    def update_odometry(self, velocities, timestamp):
        """Update position using dead reckoning and publish odometry"""
        current_time = time.time()
        
        if self.last_time is not None:
            dt = current_time - self.last_time
            
            # Simple dead reckoning
            self.position_x += velocities[0] * dt
            self.position_y += velocities[1] * dt
            self.position_z += velocities[2] * dt
        
        self.last_time = current_time
        
        # Publish odometry
        odom_msg = Odometry()
        odom_msg.header.stamp = timestamp
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = self.frame_id
        
        # Position
        odom_msg.pose.pose.position.x = self.position_x
        odom_msg.pose.pose.position.y = self.position_y
        odom_msg.pose.pose.position.z = self.position_z
        
        # Orientation (identity for now)
        odom_msg.pose.pose.orientation.w = 1.0
        
        # Velocity
        odom_msg.twist.twist.linear.x = float(velocities[0])
        odom_msg.twist.twist.linear.y = float(velocities[1])
        odom_msg.twist.twist.linear.z = float(velocities[2])
        
        # Publish position for MAVROS
        self.publish_position(odom_msg.pose, timestamp)
        
        self.odometry_pub.publish(odom_msg)
    
    def publish_position(self, pose, timestamp):
        #"""Publish position with covariance for MAVROS"""
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = timestamp
        msg.header.frame_id = "odom"
    
        msg.pose.pose = pose.pose
    
        # Initialize covariance array with zeros
        covariance = [0.0] * 36
    
        # Position covariance (grows with time due to dead reckoning)
        pos_cov = 0.1 + (time.time() - (self.last_valid_data_time or time.time())) * 0.01
        pos_cov = max(0.01, min(pos_cov, 10.0))  # Clamp between reasonable bounds
    
        covariance[0] = float(pos_cov)   # x position variance
        covariance[7] = float(pos_cov)   # y position variance
        covariance[14] = float(pos_cov)  # z position variance
        covariance[21] = 0.1  # roll variance
        covariance[28] = 0.1  # pitch variance  
        covariance[35] = 0.1  # yaw variance
    
        msg.pose.covariance = covariance
    
        self.position_pub.publish(msg)
    
    def check_health(self):
        """Monitor DVL health and connection status"""
        current_time = time.time()
        time_since_data = current_time - self.last_valid_data_time
        
        if time_since_data > self.data_timeout:
            self.get_logger().warn(f"⚠️ No valid DVL data for {time_since_data:.1f}s")
    
    def shutdown(self):
        """Clean shutdown of connections"""
        if self.sensor:
            try:
                self.sensor.disconnect()
                self.get_logger().info("🔄 Serial DVL disconnected")
            except:
                pass
        
        if self.tcp_socket:
            try:
                self.tcp_socket.close()
                self.get_logger().info("🔄 TCP DVL disconnected")
            except:
                pass
        
        if self.csv_logging and hasattr(self, 'csv_file'):
            try:
                self.csv_file.close()
                self.get_logger().info("🔄 CSV logging stopped")
            except:
                pass

def main(args=None):
    rclpy.init(args=args)
    node = DVLNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 DVL Node shutting down...")
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
