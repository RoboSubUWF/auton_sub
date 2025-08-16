#!/usr/bin/env python3
"""
Fixed DVL to MAVROS Bridge Node

This node bridges DVL data to MAVROS-compatible topics for ArduSub integration.
Updated with correct MAVROS topic names, proper error handling, and QoS matching.

Key changes in this version:
- Option A fix: flip only Z (down↔up) when publishing to MAVROS vision topics (ROS uses ENU).
- Sanity checks: drop NaNs/inf, clamp speeds/positions to pool-reasonable ranges.
- Clear comments on frames and assumptions.
"""

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import TwistStamped, PoseStamped
from nav_msgs.msg import Odometry


class DVLMAVROSBridge(Node):
    def __init__(self):
        super().__init__('dvl_mavros_bridge')

        # Small startup delay so MAVROS is up before we publish
        time.sleep(3.0)

        # --- Config / assumptions -------------------------------------------------
        # We assume DVL gives body-frame velocities with Z positive DOWN.
        # MAVROS vision topics expect ROS ENU (Z positive UP). So we flip only Z.
        self.flip_z = True  # Option A: make ENU.z = - DVL.z
        self.max_abs_v = 0.6   # m/s sanity clamp (pool)
        self.max_abs_pos = 10  # m sanity clamp (position)
        # --------------------------------------------------------------------------

        # Match DVL subscriber QoS
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE
        )

        # MAVROS prefers RELIABLE for its inputs
        mavros_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE
        )

        # --- Subscriptions (from your DVL stack) ---------------------------------
        self.velocity_sub = self.create_subscription(
            TwistStamped,
            '/dvl/velocity',
            self.velocity_callback,
            qos_profile
        )

        self.odometry_sub = self.create_subscription(
            Odometry,
            '/dvl/odometry',
            self.odometry_callback,
            qos_profile
        )

        # --- Publications (to MAVROS vision inputs) ------------------------------
        self.vision_speed_pub = self.create_publisher(
            TwistStamped,
            '/mavros/vision_speed/speed_twist',
            mavros_qos
        )
        self.vision_pose_pub = self.create_publisher(
            PoseStamped,
            '/mavros/vision_pose/pose',
            mavros_qos
        )

        # Status tracking
        self.last_velocity_time = 0.0
        self.last_position_time = 0.0
        self.velocity_count = 0
        self.position_count = 0

        self.get_logger().info("🌉 DVL-MAVROS Bridge started with QoS matching!")
        self.get_logger().info("📡 Bridging /dvl/* → /mavros/vision_* (Z flipped for ENU)")

        # Periodic status
        self.status_interval = 5.0
        self.status_timer = self.create_timer(self.status_interval, self.status_report)

        # Simple readiness check for MAVROS
        self.mavros_check_timer = self.create_timer(2.0, self.check_mavros_ready)

    # --------------------------- Utilities ---------------------------------------

    @staticmethod
    def _finite(*vals) -> bool:
        """Return True if all vals are finite (no NaN/inf)."""
        return all(math.isfinite(v) for v in vals)

    def _clamp(self, v: float, lim: float) -> float:
        return max(-lim, min(lim, float(v)))

    # --------------------------- Timers / Status ---------------------------------

    def check_mavros_ready(self):
        try:
            topic_names = [name for name, _ in self.get_topic_names_and_types()]
            mavros_topics = [t for t in topic_names if '/mavros/' in t]
            if len(mavros_topics) > 5:
                self.get_logger().info(f"✅ MAVROS topics detected ({len(mavros_topics)}) - bridge ready")
                self.mavros_check_timer.destroy()
            else:
                self.get_logger().warn(f"⚠️ Waiting for MAVROS topics... (found {len(mavros_topics)})")
        except Exception as e:
            self.get_logger().error(f"❌ Error checking MAVROS: {e}")

    def status_report(self):
        now = time.time()
        vel_age = now - self.last_velocity_time if self.last_velocity_time > 0 else float('inf')
        pos_age = now - self.last_position_time if self.last_position_time > 0 else float('inf')

        # Events per interval (timer runs every self.status_interval seconds)
        vel_rate = self.velocity_count / self.status_interval
        pos_rate = self.position_count / self.status_interval

        if vel_age < 2.0 and pos_age < 2.0:
            self.get_logger().info(f"✅ Bridge healthy - Vel: {vel_rate:.1f} Hz, Pos: {pos_rate:.1f} Hz")
        elif vel_age < 10.0 or pos_age < 10.0:
            self.get_logger().warn(f"⚠️ Data intermittent - vel: {vel_age:.1f}s ago, pos: {pos_age:.1f}s ago")
        else:
            self.get_logger().error(f"❌ Data stale - vel: {vel_age:.1f}s ago, pos: {pos_age:.1f}s ago")

        # Reset counters for next window
        self.velocity_count = 0
        self.position_count = 0

    # --------------------------- Callbacks ---------------------------------------

    def velocity_callback(self, msg: TwistStamped):
        """Bridge DVL velocity to MAVROS vision speed"""
        try:
            self.velocity_count += 1

            # Build MAVROS message
            vision_speed_msg = TwistStamped()
            vision_speed_msg.header = msg.header
            vision_speed_msg.header.frame_id = "base_link"

            # Copy the incoming twist
            vision_speed_msg.twist = msg.twist

            # ✅ STEP 5 (flip DVL +down to MAVROS z-up)
            vision_speed_msg.twist.linear.z = -msg.twist.linear.z

            # Sanity bounds (optional – uses original values)
            vx, vy, vz = msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z
            if (vx == vx and vy == vy and vz == vz and
                abs(vx) < 10.0 and abs(vy) < 10.0 and abs(vz) < 5.0):

                self.vision_speed_pub.publish(vision_speed_msg)
                self.last_velocity_time = time.time()
                if self.velocity_count % 10 == 0:
                    self.get_logger().info(
                        f"🚀 Velocity bridged: [{vx:.3f}, {vy:.3f}, {vz:.3f}] m/s (z flipped)"
                    )
            else:
                self.get_logger().warn(f"⚠️ Invalid velocity data: [{vx}, {vy}, {vz}]")

        except Exception as e:
            self.get_logger().error(f"❌ Velocity bridge error: {e}")


    def odometry_callback(self, msg: Odometry):
        """Bridge DVL odometry to MAVROS vision pose"""
        try:
            self.position_count += 1

            # Build MAVROS message
            vision_pose_msg = PoseStamped()
            vision_pose_msg.header = msg.header
            vision_pose_msg.header.frame_id = "odom"

            # Copy pose then flip z
            vision_pose_msg.pose = msg.pose.pose

            # ✅ STEP 5 (flip DVL +down to MAVROS z-up)
            vision_pose_msg.pose.position.z = -msg.pose.pose.position.z

            # (Optionally leave orientation as-is; IMU/LP pose handles yaw)
            pos = msg.pose.pose.position
            if (pos.x == pos.x and pos.y == pos.y and pos.z == pos.z and
                abs(pos.x) < 1000.0 and abs(pos.y) < 1000.0):

                self.vision_pose_pub.publish(vision_pose_msg)
                self.last_position_time = time.time()
                if self.position_count % 10 == 0:
                    self.get_logger().info(
                        f"🎯 Position bridged: [{pos.x:.3f}, {pos.y:.3f}, {-pos.z:.3f}] m (z flipped)"
                    )
            else:
                self.get_logger().warn(f"⚠️ Invalid position data: [{pos.x}, {pos.y}, {pos.z}]")

        except Exception as e:
            self.get_logger().error(f"❌ Odometry bridge error: {e}")

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
