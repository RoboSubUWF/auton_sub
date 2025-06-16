#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import os

class JetsonCameraNode(Node):
    def __init__(self):
        super().__init__('jetson_camera')
        
        # Publisher for camera images
        self.image_publisher = self.create_publisher(Image, '/camera/image_raw', 10)
        
        # CV Bridge for image conversion
        self.bridge = CvBridge()
        
        # Camera parameters
        self.camera_width = 800
        self.camera_height = 600
        self.camera_fps = 30
        
        # Setup camera
        self.setup_camera()
        
        # Timer for capturing frames
        self.timer = self.create_timer(1.0/self.camera_fps, self.capture_and_publish)
        
        self.get_logger().info("📷 Jetson Camera Node Started!")

    def setup_camera(self):
        """Setup camera with multiple fallback options"""
        # Set environment variable to avoid BLAS issues
        os.environ['OPENBLAS_NUM_THREADS'] = '1'
        
        # Try multiple camera sources in order of preference
        camera_sources = [
            # Simple USB camera (most reliable)
            {"source": 0, "backend": cv2.CAP_V4L2, "name": "USB Camera (V4L2)"},
            
            # Try different video devices
            {"source": "/dev/video0", "backend": cv2.CAP_V4L2, "name": "Video0 (V4L2)"},
            {"source": "/dev/video1", "backend": cv2.CAP_V4L2, "name": "Video1 (V4L2)"},
            
            # GStreamer pipeline (for advanced users)
            {"source": self.get_gstreamer_pipeline(), "backend": cv2.CAP_GSTREAMER, "name": "GStreamer Pipeline"},
            
            # Default OpenCV
            {"source": 0, "backend": cv2.CAP_ANY, "name": "Default OpenCV"}
        ]
        
        self.cap = None
        for camera_config in camera_sources:
            try:
                source = camera_config["source"]
                backend = camera_config["backend"]
                name = camera_config["name"]
                
                self.get_logger().info(f"🔍 Trying: {name}")
                
                self.cap = cv2.VideoCapture(source, backend)
                
                if self.cap.isOpened():
                    # Configure camera properties
                    success = self.configure_camera()
                    
                    if success:
                        self.get_logger().info(f"✅ Camera initialized successfully with: {name}")
                        return
                    else:
                        self.cap.release()
                        self.cap = None
                        
            except Exception as e:
                self.get_logger().warn(f"❌ Failed with {name}: {e}")
                if self.cap:
                    self.cap.release()
                    self.cap = None
        
        self.get_logger().error("❌ Failed to open any camera source!")
        self.cap = None
    
    def get_gstreamer_pipeline(self):
        """Generate GStreamer pipeline string"""
        return (
            f"v4l2src device=/dev/video0 ! "
            f"video/x-raw,width={self.camera_width},height={self.camera_height},"
            f"format=YUY2,framerate={self.camera_fps}/1 ! "
            f"videoconvert ! video/x-raw,format=BGR ! appsink drop=1"
        )
    
    def configure_camera(self):
        """Configure camera properties and test capture"""
        try:
            # Set camera properties
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.camera_width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.camera_height)
            self.cap.set(cv2.CAP_PROP_FPS, self.camera_fps)
            
            # Set buffer size to reduce latency
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            
            # Test capture
            for i in range(5):  # Try a few times
                ret, frame = self.cap.read()
                if ret and frame is not None:
                    actual_height, actual_width = frame.shape[:2]
                    self.get_logger().info(f"📐 Actual resolution: {actual_width}x{actual_height}")
                    
                    # Verify the frame is valid
                    if actual_width > 0 and actual_height > 0:
                        return True
                
            return False
            
        except Exception as e:
            self.get_logger().error(f"Camera configuration error: {e}")
            return False

    def capture_and_publish(self):
        """Capture frame and publish to ROS2 topic"""
        if self.cap is None:
            return
            
        try:
            ret, frame = self.cap.read()
            if ret and frame is not None:
                # Verify frame is valid
                if frame.size == 0:
                    self.get_logger().warn("Received empty frame")
                    return
                
                # Convert OpenCV image to ROS2 Image message
                try:
                    img_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
                    img_msg.header.stamp = self.get_clock().now().to_msg()
                    img_msg.header.frame_id = "camera_frame"
                    
                    # Publish the image
                    self.image_publisher.publish(img_msg)
                    
                except Exception as bridge_error:
                    self.get_logger().error(f"CV Bridge error: {bridge_error}")
                    
            else:
                self.get_logger().warn("Failed to capture frame")
                # Try to reconnect camera
                self.reconnect_camera()
                
        except Exception as e:
            self.get_logger().error(f"Error in capture_and_publish: {e}")
            # Try to reconnect camera
            self.reconnect_camera()
    
    def reconnect_camera(self):
        """Attempt to reconnect camera if connection is lost"""
        self.get_logger().info("🔄 Attempting to reconnect camera...")
        if self.cap:
            self.cap.release()
            self.cap = None
        
        # Wait a bit before retrying
        import time
        time.sleep(1)
        
        # Try to setup camera again
        self.setup_camera()

    def destroy_node(self):
        """Clean up resources"""
        if self.cap:
            self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = JetsonCameraNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Camera node stopped cleanly.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()