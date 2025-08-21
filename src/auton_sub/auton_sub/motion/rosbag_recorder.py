#!/usr/bin/env python3
"""
ROS2 Rosbag Recorder for DVL + MAVROS Integration
Automatically records all relevant topics for analysis
Wasnt working great, there is another version of this in utils, not sure if it would work better or not.
"""

import os
import signal
import subprocess
import threading
import time
from datetime import datetime

import rclpy
from rclpy.node import Node


class RosbagRecorder(Node):
    def __init__(self):
        super().__init__('rosbag_recorder')

        # Create filename with timestamp
        now = datetime.now()
        filename = now.strftime("robosub_%Y-%m-%d_%H-%M-%S")
        
        # Save to home directory rosbags folder (easier to access)
        self.output_dir = f"/home/robosub/rosbags/{filename}"

        # Create directory if not exists
        os.makedirs("/home/robosub/rosbags", exist_ok=True)

        # All topics from your system - comprehensive logging
        self.topics = [
            # Core diagnostics
            "/diagnostics",
            "/parameter_events",
            "/rosout",
            
            # DVL Topics (your main sensor data)
            "/dvl/odometry",
            "/dvl/position", 
            "/dvl/range",
            "/dvl/velocity",
            "/dvl/velocity_with_covariance",
            
            # MAVROS Vision Topics (from DVL bridge - critical for control)
            "/mavros/vision_pose/pose",
            "/mavros/vision_speed/speed_twist",
            
            # MAVROS Core Topics
            "/mavros/altitude",
            "/mavros/cam_imu_sync/cam_imu_stamp",
            "/mavros/camera/image_captured",
            "/mavros/target_actuator_control",
            
            # MAVROS Debug and Diagnostics
            "/mavros/debug_value/debug",
            "/mavros/debug_value/debug_float_array", 
            "/mavros/debug_value/debug_vector",
            "/mavros/debug_value/named_value_float",
            "/mavros/debug_value/named_value_int",
            
            # MAVROS ADSB
            "/mavros/adsb/vehicle",
            
            # Transform data
            "/tf",
            "/tf_static",
            
            # Mavlink
            "/uas1/mavlink_source",
            
            # Add control topics that your robot_control.py publishes to
            "/mavros/setpoint_velocity/cmd_vel_unstamped",
            "/mavros/setpoint_raw/local", 
            "/mavros/manual_control/control",
            
            # Add any IMU/orientation topics if available
            "/mavros/local_position/pose",
            "/mavros/local_position/velocity_local",
            "/mavros/imu/data",
        ]

        self.get_logger().info(f"🔦 Starting rosbag2 recording to: {self.output_dir}")
        self.get_logger().info(f"🛰️ Recording {len(self.topics)} topics for DVL+MAVROS system")
        self.get_logger().info(f"💾 Bag will be saved to: {self.output_dir}")

        # Start rosbag record as subprocess with compression
        cmd = [
            "ros2", "bag", "record", 
            "-o", self.output_dir,
            "--compression-mode", "file",  # Compress to save space
            "--compression-format", "zstd",  # Fast compression
        ] + self.topics
        
        self.get_logger().info(f"📝 Command: {' '.join(cmd[:8])} ... [+{len(self.topics)} topics]")
        
        try:
            self.rosbag_proc = subprocess.Popen(cmd, 
                                              stdout=subprocess.PIPE, 
                                              stderr=subprocess.PIPE)
            self.get_logger().info("✅ Rosbag recording started successfully")
        except Exception as e:
            self.get_logger().error(f"❌ Failed to start rosbag: {str(e)}")
            return

        # Register shutdown hooks
        signal.signal(signal.SIGINT, self.shutdown_handler)
        signal.signal(signal.SIGTERM, self.shutdown_handler)
        
        # Start monitoring thread
        self.monitor_thread = threading.Thread(target=self.monitor_recording)
        self.monitor_thread.daemon = True
        self.monitor_thread.start()

    def monitor_recording(self):
        """Monitor rosbag process and log status"""
        start_time = time.time()
        
        while True:
            if self.rosbag_proc.poll() is not None:
                # Process ended
                self.get_logger().error("⚠️ Rosbag process ended unexpectedly")
                break
                
            # Log status every 30 seconds
            elapsed = time.time() - start_time
            if elapsed % 30 < 1:  # Every 30 seconds
                self.get_logger().info(f"📊 Recording active for {elapsed/60:.1f} minutes")
                # Check bag file size
                try:
                    if os.path.exists(self.output_dir):
                        size = sum(os.path.getsize(os.path.join(self.output_dir, f)) 
                                 for f in os.listdir(self.output_dir) 
                                 if os.path.isfile(os.path.join(self.output_dir, f)))
                        self.get_logger().info(f"💾 Bag size: {size/1024/1024:.1f} MB")
                except Exception as e:
                    pass  # Don't spam errors for file size checking
            
            time.sleep(1)

    def shutdown_handler(self, signum, frame):
        """Clean shutdown of rosbag recording"""
        self.get_logger().info("🛑 Shutting down rosbag recording...")
        try:
            # Send interrupt to rosbag
            if self.rosbag_proc and self.rosbag_proc.poll() is None:
                self.rosbag_proc.send_signal(signal.SIGINT)
                
                # Wait for clean shutdown
                try:
                    self.rosbag_proc.wait(timeout=10)
                    self.get_logger().info("✅ Rosbag stopped cleanly")
                except subprocess.TimeoutExpired:
                    self.get_logger().warn("⚠️ Rosbag didn't stop, forcing...")
                    self.rosbag_proc.terminate()
                    self.rosbag_proc.wait(timeout=5)
            
            # Log final info
            if os.path.exists(self.output_dir):
                size = sum(os.path.getsize(os.path.join(self.output_dir, f)) 
                         for f in os.listdir(self.output_dir) 
                         if os.path.isfile(os.path.join(self.output_dir, f)))
                self.get_logger().info(f"📦 Final bag saved: {self.output_dir}")
                self.get_logger().info(f"💾 Final size: {size/1024/1024:.1f} MB")
                self.get_logger().info(f"🔍 To play back: ros2 bag play {self.output_dir}")
            
        except Exception as e:
            self.get_logger().error(f"⚠️ Error during shutdown: {str(e)}")
        finally:
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = RosbagRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🔴 Keyboard interrupt received")
    except Exception as e:
        node.get_logger().error(f"💥 Unexpected error: {str(e)}")


if __name__ == "__main__":
    main()
