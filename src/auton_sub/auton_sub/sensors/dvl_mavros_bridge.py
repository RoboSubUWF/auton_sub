#!/usr/bin/env python3
"""
Fixed DVL to MAVROS Bridge Node

This node bridges DVL data to MAVROS-compatible topics for ArduSub integration.
Updated with correct MAVROS topic names, proper error handling, and QoS matching.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import TwistStamped, PoseStamped
from nav_msgs.msg import Odometry
import time

class DVLMAVROSBridge(Node):
    def __init__(self):
        super().__init__('dvl_mavros_bridge')
        
        # Add delay to ensure MAVROS starts first
        time.sleep(3.0)
        
        # ✅ FIXED: Match QoS profile with DVL node
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,  # Match DVL node
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE
        )
        
        # MAVROS-compatible QoS (for publishers to MAVROS)
        mavros_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,     # MAVROS prefers reliable
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE
        )
        
        # Subscribers to DVL data - Use matching QoS
        self.velocity_sub = self.create_subscription(
            TwistStamped,
            '/dvl/velocity',
            self.velocity_callback,
            qos_profile  # ✅ Use matching QoS
        )
        
        self.odometry_sub = self.create_subscription(
            Odometry,
            '/dvl/odometry',
            self.odometry_callback,
            qos_profile  # ✅ Use matching QoS
        )
        
        # Publishers to MAVROS topics - Use MAVROS-compatible QoS
        self.vision_speed_pub = self.create_publisher(
            TwistStamped,
            '/mavros/vision_speed/speed_twist',
            mavros_qos  # Use reliable QoS for MAVROS
        )
        
        self.vision_pose_pub = self.create_publisher(
            PoseStamped,
            '/mavros/vision_pose/pose',
            mavros_qos  # Use reliable QoS for MAVROS
        )
        
        # Status tracking
        self.last_velocity_time = 0.0
        self.last_position_time = 0.0
        self.velocity_count = 0
        self.position_count = 0
        
        self.get_logger().info("🌉 DVL-MAVROS Bridge started with QoS matching!")
        self.get_logger().info("📡 Bridging /dvl/* → /mavros/vision_*")
        
        # Create timer for status reporting
        self.status_timer = self.create_timer(5.0, self.status_report)
        
        # Wait for MAVROS to be fully ready
        self.mavros_check_timer = self.create_timer(2.0, self.check_mavros_ready)
    
    def check_mavros_ready(self):
        """Check if MAVROS topics are available"""
        try:
            topic_names = [name for name, _ in self.get_topic_names_and_types()]
            mavros_topics = [topic for topic in topic_names if '/mavros/' in topic]
            
            if len(mavros_topics) > 5:  # MAVROS typically has many topics
                self.get_logger().info(f"✅ MAVROS topics detected ({len(mavros_topics)} topics) - bridge ready")
                self.mavros_check_timer.destroy()  # Stop checking
            else:
                self.get_logger().warn(f"⚠️ Waiting for MAVROS topics... (found {len(mavros_topics)})")
                
        except Exception as e:
            self.get_logger().error(f"❌ Error checking MAVROS: {e}")
    
    def velocity_callback(self, msg: TwistStamped):
        """Bridge DVL velocity to MAVROS vision speed"""
        try:
            self.velocity_count += 1
            
            # Create vision speed message
            vision_speed_msg = TwistStamped()
            vision_speed_msg.header = msg.header
            vision_speed_msg.header.frame_id = "base_link"  # MAVROS expects base_link
            
            # Validate velocity data (check for NaN and reasonable bounds)
            vx, vy, vz = msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z
            
            if (vx == vx and vy == vy and vz == vz and  # Check for NaN
                abs(vx) < 10.0 and abs(vy) < 10.0 and abs(vz) < 5.0):  # Reasonable bounds
                
                vision_speed_msg.twist = msg.twist
                
                # Publish to MAVROS
                self.vision_speed_pub.publish(vision_speed_msg)
                
                self.last_velocity_time = time.time()
                
                if self.velocity_count % 20 == 0:  # Log every 20th message
                    self.get_logger().info(f"🚀 Velocity bridged: [{vx:.3f}, {vy:.3f}, {vz:.3f}] m/s")
            else:
                self.get_logger().warn(f"⚠️ Invalid velocity data: [{vx}, {vy}, {vz}]")
                
        except Exception as e:
            self.get_logger().error(f"❌ Velocity bridge error: {e}")
    
    def odometry_callback(self, msg: Odometry):
        """Bridge DVL odometry to MAVROS vision pose"""
        try:
            self.position_count += 1
            
            # Create vision pose message
            vision_pose_msg = PoseStamped()
            vision_pose_msg.header = msg.header
            vision_pose_msg.header.frame_id = "map"  # MAVROS expects map frame
            
            # Validate position data
            pos = msg.pose.pose.position
            if (pos.x == pos.x and pos.y == pos.y and pos.z == pos.z and  # Check for NaN
                abs(pos.x) < 1000.0 and abs(pos.y) < 1000.0):  # Reasonable bounds
                
                # Copy pose data
                vision_pose_msg.pose = msg.pose.pose
                
                # Publish to MAVROS vision pose
                self.vision_pose_pub.publish(vision_pose_msg)
                
                self.last_position_time = time.time()
                
                if self.position_count % 10 == 0:  # Log every 10th message
                    self.get_logger().info(f"🎯 Position bridged: [{pos.x:.3f}, {pos.y:.3f}, {pos.z:.3f}] m")
            else:
                self.get_logger().warn(f"⚠️ Invalid position data: [{pos.x}, {pos.y}, {pos.z}]")
            
        except Exception as e:
            self.get_logger().error(f"❌ Odometry bridge error: {e}")
    
    def status_report(self):
        """Report bridge status periodically"""
        current_time = time.time()
        vel_age = current_time - self.last_velocity_time if self.last_velocity_time > 0 else float('inf')
        pos_age = current_time - self.last_position_time if self.last_position_time > 0 else float('inf')
        
        # Report data flow rates
        vel_rate = self.velocity_count / 5.0 if current_time > 5.0 else 0
        pos_rate = self.position_count / 5.0 if current_time > 5.0 else 0
        
        if vel_age < 2.0 and pos_age < 2.0:
            self.get_logger().info(f"✅ Bridge healthy - Vel: {vel_rate:.1f}Hz, Pos: {pos_rate:.1f}Hz")
        elif vel_age < 10.0 or pos_age < 10.0:
            self.get_logger().warn(f"⚠️ Data intermittent - vel: {vel_age:.1f}s ago, pos: {pos_age:.1f}s ago")
        else:
            self.get_logger().error(f"❌ Data stale - vel: {vel_age:.1f}s ago, pos: {pos_age:.1f}s ago")
        
        # Reset counters for next interval
        self.velocity_count = 0
        self.position_count = 0

def main(args=None):
    rclpy.init(args=args)
    bridge = DVLMAVROSBridge()
    
    try:
        rclpy.spin(bridge)
    except KeyboardInterrupt:
        bridge.get_logger().info("🛑 DVL-MAVROS Bridge shutting down...")
    finally:
        bridge.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()