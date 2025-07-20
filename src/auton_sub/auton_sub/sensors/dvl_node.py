#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import FluidPressure
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
        
        # Publishers
        self.velocity_pub = self.create_publisher(TwistStamped, '/dvl/velocity', 10)
        self.odometry_pub = self.create_publisher(Odometry, '/dvl/odometry', 10)
        
        # Configuration
        self.connection_type = self.get_parameter('connection_type').get_parameter_value().string_value
        self.wayfinderPort = self.get_parameter('serial_port').get_parameter_value().string_value
        self.tcp_ip = self.get_parameter('tcp_ip').get_parameter_value().string_value
        self.tcp_port = self.get_parameter('tcp_port').get_parameter_value().integer_value
        self.csv_logging = self.get_parameter('csv_logging').get_parameter_value().bool_value
        self.csv_file_path = self.get_parameter('csv_file_path').get_parameter_value().string_value
        
        # Sensor objects
        self.sensor = None
        self.tcp_socket = None
        self.csv_writer = None
        
        # Coordinate frame transformation
        self.roll = np.radians(180)
        self.pitch = np.radians(0)
        self.yaw = np.radians(135)
        self.setup_transform()
        
        # Position tracking
        self.position_x = 0.0
        self.position_y = 0.0
        self.position_z = 0.0
        self.last_time = None
        
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
            fieldnames = ['timestamp', 'vx', 'vy', 'vz', 'fom', 'velocity_valid', 'status']
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
            
            # Get velocities
            raw_vels = np.array([data_obj.vel_x, data_obj.vel_y, data_obj.vel_z])
            
            # CSV logging
            if self.csv_logging and self.csv_writer:
                csv_row = {
                    'timestamp': time.time(),
                    'vx': data_obj.vel_x,
                    'vy': data_obj.vel_y,
                    'vz': data_obj.vel_z,
                    'fom': data_obj.vel_err,
                    'velocity_valid': data_obj.is_velocity_valid(),
                    'status': getattr(data_obj, 'status', 0)
                }
                self.csv_writer.writerow(csv_row)
                self.csv_file.flush()
            
            # Process data
            self.process_velocity_data(raw_vels)
            
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
            
            if not velocity_valid:
                self.get_logger().warn("⚠️ TCP DVL reports invalid velocities")
                return
            
            raw_vels = np.array([vx, vy, vz])
            
            # CSV logging
            if self.csv_logging and self.csv_writer:
                csv_row = {
                    'timestamp': time.time(),
                    'vx': vx,
                    'vy': vy,
                    'vz': vz,
                    'fom': fom,
                    'velocity_valid': velocity_valid,
                    'status': data_dict.get('status', 0)
                }
                self.csv_writer.writerow(csv_row)
                self.csv_file.flush()
            
            # Process data
            self.process_velocity_data(raw_vels)
            
        except Exception as e:
            self.get_logger().error(f"❌ TCP DVL callback error: {e}")
    
    def process_velocity_data(self, raw_velocities):
        """Common velocity processing for both connection types"""
        # Transform to vehicle frame
        vels = np.matmul(self.rot_matrix, raw_velocities)
        
        # Get timestamp
        timestamp = self.get_clock().now().to_msg()
        
        # Publish velocity
        self.publish_velocity(vels, timestamp)
        
        # Update and publish odometry
        self.update_odometry(vels, timestamp)
        
        # Log data
        self.get_logger().info(f"DVL Velocity: [{vels[0]:.3f}, {vels[1]:.3f}, {vels[2]:.3f}] m/s")
    
    def publish_velocity(self, velocities, timestamp):
        """Publish velocity data as TwistStamped"""
        msg = TwistStamped()
        msg.header.stamp = timestamp
        msg.header.frame_id = "base_link"
        
        msg.twist.linear.x = float(velocities[0])
        msg.twist.linear.y = float(velocities[1])
        msg.twist.linear.z = float(velocities[2])
        
        self.velocity_pub.publish(msg)
    
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
        odom_msg.child_frame_id = "base_link"
        
        # Position
        odom_msg.pose.pose.position.x = self.position_x
        odom_msg.pose.pose.position.y = self.position_y
        odom_msg.pose.pose.position.z = self.position_z
        
        # Velocity
        odom_msg.twist.twist.linear.x = float(velocities[0])
        odom_msg.twist.twist.linear.y = float(velocities[1])
        odom_msg.twist.twist.linear.z = float(velocities[2])
        
        self.odometry_pub.publish(odom_msg)
    
    def shutdown(self):
        """Clean shutdown of connections"""
        if self.sensor:
            self.sensor.disconnect()
            self.get_logger().info("🔄 Serial DVL disconnected")
        
        if self.tcp_socket:
            self.tcp_socket.close()
            self.get_logger().info("🔄 TCP DVL disconnected")
        
        if self.csv_logging and hasattr(self, 'csv_file'):
            self.csv_file.close()
            self.get_logger().info("🔄 CSV logging stopped")

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