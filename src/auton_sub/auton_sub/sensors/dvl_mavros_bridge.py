#!/usr/bin/env python3
"""
Fixed DVL to MAVROS Bridge Node

This node bridges DVL data to MAVROS-compatible topics for ArduSub integration.
Updated to use correct MAVROS topic names and avoid conflicts.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped, PoseStamped
from nav_msgs.msg import Odometry
import tf_transformations
import numpy as np
import time

class DVLMAVROSBridge(Node):
    def __init__(self):
        super().__init__('dvl_mavros_bridge')
        
        # Add delay to ensure MAVROS starts first
        time.sleep(2.0)
        
        # Subscribers to DVL data
        self.velocity_sub = self.create_subscription(
            TwistStamped,
            '/dvl/velocity',
            self.velocity_callback,
            10
        )
        
        self.odometry_sub = self.create_subscription(
            Odometry,
            '/dvl/odometry',
            self.odometry_callback,
            10
        )
        
        # ✅ FIXED: Use correct MAVROS topic names
        # Publishers to MAVROS topics - Updated topic names
        self.vision_speed_pub = self.create_publisher(
            TwistStamped,
            '/mavros/vision_speed_estimate/speed_twist',  # Fixed topic name
            10
        )
        
        self.vision_pose_pub = self.create_publisher(
            PoseStamped,
            '/mavros/vision_pose_estimate/pose',  # Fixed topic name
            10
        )
        
        # Status tracking
        self.last_velocity_time = 0.0
        self.last_position_time = 0.0
        
        self.get_logger().info("🌉 DVL-MAVROS Bridge started!")
        self.get_logger().info("📡 Bridging /dvl/* → /mavros/vision_*_estimate")
        
        # Create timer for status reporting
        self.status_timer = self.create_timer(5.0, self.status_report)
    
    def velocity_callback(self, msg: TwistStamped):
        """Bridge DVL velocity to MAVROS vision speed estimate"""
        try:
            # Create vision speed message
            vision_speed_msg = TwistStamped()
            vision_speed_msg.header = msg.header
            vision_speed_msg.header.frame_id = "base_link"  # MAVROS expects base_link
            
            # Copy velocity data
            vision_speed_msg.twist = msg.twist
            
            # Publish to MAVROS
            self.vision_speed_pub.publish(vision_speed_msg)
            
            self.last_velocity_time = time.time()
            
            self.get_logger().debug(f"🚀 Velocity bridged: [{msg.twist.linear.x:.3f}, {msg.twist.linear.y:.3f}, {msg.twist.linear.z:.3f}]")
            
        except Exception as e:
            self.get_logger().error(f"❌ Velocity bridge error: {e}")
    
    def odometry_callback(self, msg: Odometry):
        """Bridge DVL odometry to MAVROS vision pose estimate"""
        try:
            # Create vision pose message
            vision_pose_msg = PoseStamped()
            vision_pose_msg.header = msg.header
            vision_pose_msg.header.frame_id = "map"  # MAVROS expects map frame for global position
            
            # Copy pose data
            vision_pose_msg.pose = msg.pose.pose
            
            # Publish to MAVROS vision pose
            self.vision_pose_pub.publish(vision_pose_msg)
            
            self.last_position_time = time.time()
            
            self.get_logger().debug(f"🎯 Position bridged: [{msg.pose.pose.position.x:.3f}, {msg.pose.pose.position.y:.3f}, {msg.pose.pose.position.z:.3f}]")
            
        except Exception as e:
            self.get_logger().error(f"❌ Odometry bridge error: {e}")
    
    def status_report(self):
        """Report bridge status periodically"""
        current_time = time.time()
        vel_age = current_time - self.last_velocity_time if self.last_velocity_time > 0 else float('inf')
        pos_age = current_time - self.last_position_time if self.last_position_time > 0 else float('inf')
        
        if vel_age < 2.0 and pos_age < 2.0:
            self.get_logger().info("✅ DVL-MAVROS bridge healthy - data flowing")
        else:
            self.get_logger().warn(f"⚠️ DVL data stale - vel: {vel_age:.1f}s ago, pos: {pos_age:.1f}s ago")

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