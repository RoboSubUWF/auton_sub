#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import Float32MultiArray, String, Bool
from sensor_msgs.msg import Image, NavSatFix, FluidPressure
from geometry_msgs.msg import TwistStamped, Pose, PoseStamped
from nav_msgs.msg import Odometry
import time
import math
import numpy as np
from pymavlink import mavutil

class GuidedMissionControl(Node):
    def __init__(self):
        super().__init__('guided_mission_control')
        
        # ✅ FIXED: QoS Profile to match DVL node
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,  # Match DVL node
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE
        )
        
        # Connect to Pixhawk via MAVLink
        self.master = mavutil.mavlink_connection('/dev/ttyTHS1', baud=57600)
        
        # Wait for heartbeat
        self.get_logger().info("🔄 Waiting for heartbeat from Pixhawk...")
        self.master.wait_heartbeat()
        self.get_logger().info("✅ Heartbeat received! Connected to Pixhawk.")
        
        # Mission parameters
        self.mission_active = False
        self.current_mission_step = 0
        self.auto_start_delay = 2.0  # 2 seconds after startup
        self.target_distance = 3.048  # 10 feet in meters
        
        # Mission waypoints (relative to starting position)
        self.mission_waypoints = [
            {"x": 0.0, "y": 0.0, "z": -1.0, "description": "Dive to 1m depth"},
            {"x": 3.048, "y": 0.0, "z": -1.0, "description": "Move forward 10ft"},
            {"x": 3.048, "y": 0.0, "z": 0.0, "description": "Surface"}
        ]
        
        # State tracking
        self.current_depth = 0.0
        self.leak_detected = False
        self.home_position = None
        self.current_position = np.array([0.0, 0.0, 0.0])
        
        # DVL data
        self.dvl_velocity = np.array([0.0, 0.0, 0.0])
        self.dvl_position = np.array([0.0, 0.0, 0.0])
        
        # Control parameters
        self.position_tolerance = 0.3  # meters
        self.waypoint_timeout = 5.0  # seconds
        self.waypoint_start_time = 0.0
        
        # ✅ FIXED: Subscribers with matching QoS
        self.leak_sub = self.create_subscription(
            Bool,
            '/leak_detected',
            self.leak_callback,
            10  # Default QoS is fine for leak detection
        )
        
        self.dvl_velocity_sub = self.create_subscription(
            TwistStamped,
            '/dvl/velocity',
            self.dvl_velocity_callback,
            qos_profile  # ✅ Use matching QoS
        )
        
        self.dvl_odometry_sub = self.create_subscription(
            Odometry,
            '/dvl/odometry',
            self.dvl_odometry_callback,
            qos_profile  # ✅ Use matching QoS
        )
        
        self.pressure_sub = self.create_subscription(
            FluidPressure,
            '/pressure',
            self.pressure_callback,
            10  # Default QoS is fine for pressure sensor
        )
        
        # Timers
        self.mission_timer = self.create_timer(0.5, self.mission_step)  # 2 Hz
        self.auto_start_timer = self.create_timer(1.0, self.auto_start_check)
        self.dvl_sender_timer = self.create_timer(0.1, self.send_dvl_to_pixhawk)  # 10 Hz
        
        # Auto-start tracking
        self.start_time = time.time()
        
        self.get_logger().info("✅ GUIDED Mission Control initialized with QoS matching!")
        self.get_logger().info(f"🚀 Will auto-start mission in {self.auto_start_delay} seconds")
    
    def auto_start_check(self):
        """Check if it's time to auto-start the mission"""
        if not self.mission_active and time.time() - self.start_time > self.auto_start_delay:
            self.get_logger().info("🚀 Auto-starting GUIDED mission!")
            self.start_mission()
            self.auto_start_timer.cancel()
    
    def start_mission(self):
        """Start the autonomous mission in GUIDED mode"""
        # Set to guided mode and arm
        self.set_guided_mode()
        self.arm_vehicle()
        
        # Set home position
        self.set_home_position()
        
        # Start mission
        self.mission_active = True
        self.current_mission_step = 0
        self.waypoint_start_time = time.time()
        
        self.get_logger().info("🚀 GUIDED mission started!")
    
    def stop_mission(self):
        """Stop the autonomous mission"""
        self.mission_active = False
        self.get_logger().info("🛑 Mission stopped!")
    
    def set_guided_mode(self):
        """Set Pixhawk to GUIDED mode"""
        self.get_logger().info("🛠️ Setting GUIDED mode...")
        
        # For ArduSub, GUIDED mode is mode 4
        self.master.mav.set_mode_send(
            self.master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            4  # GUIDED mode for ArduSub
        )
        
        time.sleep(2)
        self.get_logger().info("✅ GUIDED mode set!")
    
    def arm_vehicle(self):
        """Arm the vehicle"""
        self.get_logger().info("🛠️ Arming vehicle...")
        
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1, 0, 0, 0, 0, 0, 0  # 1 = Arm
        )
        
        time.sleep(3)
        self.get_logger().info("✅ Vehicle armed!")
    
    def set_home_position(self):
        """Set current position as home"""
        self.get_logger().info("🏠 Setting home position...")
        
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_HOME,
            0,
            1, 0, 0, 0, 0, 0, 0  # 1 = use current position
        )
        
        time.sleep(1)
        self.get_logger().info("✅ Home position set!")
    
    def send_waypoint(self, x, y, z):
        """Send waypoint command to Pixhawk in GUIDED mode"""
        self.get_logger().info(f"📍 Sending waypoint: [{x:.2f}, {y:.2f}, {z:.2f}]")
        
        # Send position target in local NED frame
        self.master.mav.set_position_target_local_ned_send(
            0,  # time_boot_ms
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            0b110111111000,  # Use position only (ignore velocity/acceleration)
            x, y, z,  # Position in meters (NED frame)
            0, 0, 0,  # Velocity (ignored)
            0, 0, 0,  # Acceleration (ignored)
            0, 0  # Yaw (ignored)
        )
    
    def leak_callback(self, msg):
        """Handle leak detection - emergency surface"""
        if msg.data:
            self.leak_detected = True
            self.get_logger().error("🚨 LEAK DETECTED! Emergency surface!")
            self.emergency_surface()
            self.stop_mission()
    
    def emergency_surface(self):
        """Emergency surface command"""
        self.get_logger().warn("🚨 EMERGENCY SURFACING!")
        # Send surface waypoint immediately
        self.send_waypoint(self.current_position[0], self.current_position[1], 0.0)
    
    def dvl_velocity_callback(self, msg):
        """Process DVL velocity data"""
        self.dvl_velocity = np.array([
            msg.twist.linear.x,
            msg.twist.linear.y,
            msg.twist.linear.z
        ])
        
        # ✅ Added: Log velocity reception for debugging
        if hasattr(self, '_vel_msg_count'):
            self._vel_msg_count += 1
        else:
            self._vel_msg_count = 1
            
        if self._vel_msg_count % 50 == 0:  # Log every 50 messages
            self.get_logger().info(f"📡 DVL velocity received: [{self.dvl_velocity[0]:.3f}, {self.dvl_velocity[1]:.3f}, {self.dvl_velocity[2]:.3f}] m/s")
    
    def dvl_odometry_callback(self, msg):
        """Process DVL odometry data"""
        self.dvl_position = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        ])
        
        # ✅ Added: Log position reception for debugging
        if hasattr(self, '_pos_msg_count'):
            self._pos_msg_count += 1
        else:
            self._pos_msg_count = 1
            
        if self._pos_msg_count % 20 == 0:  # Log every 20 messages
            self.get_logger().info(f"📍 DVL position received: [{self.dvl_position[0]:.3f}, {self.dvl_position[1]:.3f}, {self.dvl_position[2]:.3f}] m")
    
    def send_dvl_to_pixhawk(self):
        """Send DVL data to Pixhawk for navigation feedback"""
        if not self.mission_active:
            return
        
        # Send DVL velocity as vision speed estimate
        current_time_us = int(time.time() * 1e6)
        
        self.master.mav.vision_speed_estimate_send(
            current_time_us,  # timestamp
            self.dvl_velocity[0],  # vx
            self.dvl_velocity[1],  # vy
            self.dvl_velocity[2],  # vz
            0,  # reset counter
            0   # quality (0-100)
        )
        
        # Also send position estimate if available
        self.master.mav.vision_position_estimate_send(
            current_time_us,  # timestamp
            self.dvl_position[0],  # x
            self.dvl_position[1],  # y
            self.dvl_position[2],  # z
            0, 0, 0,  # roll, pitch, yaw
            0,  # reset counter
            0   # quality
        )
    
    def pressure_callback(self, msg):
        """Calculate depth from pressure"""
        pressure_pascal = msg.fluid_pressure
        self.current_depth = (pressure_pascal - 101325) / (1000 * 9.8)
        
        # Update current position Z with depth
        self.current_position[2] = -self.current_depth  # Negative for NED frame
    
    def mission_step(self):
        """Execute current mission step using waypoints"""
        if not self.mission_active:
            return
        
        if self.current_mission_step >= len(self.mission_waypoints):
            self.get_logger().info("✅ Mission completed!")
            self.stop_mission()
            return
        
        waypoint = self.mission_waypoints[self.current_mission_step]
        
        # Check if we've reached the waypoint
        target_pos = np.array([waypoint["x"], waypoint["y"], waypoint["z"]])
        current_pos = np.array([self.dvl_position[0], self.dvl_position[1], -self.current_depth])
        
        distance_to_target = np.linalg.norm(target_pos - current_pos)
        
        # Check if waypoint reached
        if distance_to_target < self.position_tolerance:
            self.get_logger().info(f"✅ Reached waypoint {self.current_mission_step}: {waypoint['description']}")
            self.current_mission_step += 1
            self.waypoint_start_time = time.time()
            return
        
        # Check for timeout
        if time.time() - self.waypoint_start_time > self.waypoint_timeout:
            self.get_logger().warn(f"⏰ Waypoint {self.current_mission_step} timed out, moving to next")
            self.current_mission_step += 1
            self.waypoint_start_time = time.time()
            return
        
        # Send waypoint command
        self.send_waypoint(waypoint["x"], waypoint["y"], waypoint["z"])
        
        self.get_logger().info(f"🎯 Moving to waypoint {self.current_mission_step}: {waypoint['description']} "
                              f"(distance: {distance_to_target:.2f}m)")

def main(args=None):
    rclpy.init(args=args)
    node = GuidedMissionControl()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 GUIDED Mission Control shutting down...")
    finally:
        node.stop_mission()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()