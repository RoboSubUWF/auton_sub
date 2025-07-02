import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO
import os
import warnings
import torch
import requests
import time
import threading
from urllib.parse import urlparse

# Suppress matplotlib warnings
warnings.filterwarnings("ignore", category=UserWarning, module="matplotlib")

class GoProHTTPObjectDetection(Node):
    def __init__(self):
        super().__init__('gopro_http_object_detection')
        
        # Load YOLO model
        model_paths = [
            "src/auton_sub/auton_sub/scripts/model/best.pt",
            "./best.pt",
            "best.pt",
            os.path.expanduser("~/best.pt")
        ]
        
        self.model = None
        for path in model_paths:
            if os.path.exists(path):
                try:
                    self.model = YOLO(path)
                    if not torch.cuda.is_available():
                        self.model.model.eval()
                    self.get_logger().info(f"✅ Model loaded successfully from: {path}")
                    self.get_logger().info(f"🔍 Model classes: {self.model.names}")
                    break
                except Exception as e:
                    self.get_logger().warn(f"Failed to load model from {path}: {e}")
                    continue
        
        if self.model is None:
            self.get_logger().error("❌ Could not load YOLO model from any path!")
            raise FileNotFoundError("YOLO model file not found")

        # ROS2 Publishers
        self.image_publisher = self.create_publisher(Image, '/processed_camera_feed', 10)
        self.objects_publisher = self.create_publisher(String, '/detected_objects', 10)
        self.bridge = CvBridge()
        
        # GoPro HTTP settings
        self.gopro_ip = "172.20.151.10"  # Default GoPro WiFi IP
        self.gopro_port = "8080"
        self.preview_stream_url = f"http://{self.gopro_ip}:{self.gopro_port}/gopro/camera/stream/start"
        self.live_stream_url = f"http://{self.gopro_ip}:8080/gopro/camera/stream/start"
        
        # Frame processing variables
        self.frame_count = 0
        self.detection_enabled = True
        self.total_detections = 0
        self.frames_processed = 0
        self.cap = None
        self.stream_active = False
        
        # Initialize GoPro connection
        self.setup_gopro_connection()
        
        # Create processing timer
        self.timer = self.create_timer(1.0/30.0, self.process_camera_frame)
        
        # Initialize display window
        cv2.namedWindow("GoPro HTTP Object Detection", cv2.WINDOW_AUTOSIZE)

        self.get_logger().info("🎥 GoPro Hero 12 HTTP Stream Object Detection Node Initialized!")
        self.get_logger().info("📡 Connect your GoPro to WiFi and ensure it's accessible")
        self.get_logger().info("💡 Press 'q' to quit, 's' to toggle detection, 'r' to reconnect")

    def setup_gopro_connection(self):
        """Setup GoPro HTTP stream connection"""
        try:
            self.get_logger().info("🔍 Attempting to connect to GoPro via HTTP...")
            
            # Try to wake up GoPro and start preview
            wake_url = f"http://{self.gopro_ip}/gopro/camera/stream/start"
            
            try:
                response = requests.get(wake_url, timeout=5)
                if response.status_code == 200:
                    self.get_logger().info("✅ GoPro HTTP connection established!")
                else:
                    self.get_logger().warn(f"⚠️ GoPro responded with status: {response.status_code}")
            except requests.exceptions.RequestException as e:
                self.get_logger().warn(f"⚠️ Could not reach GoPro at {self.gopro_ip}: {e}")
                self.get_logger().info("💡 Make sure GoPro is connected to WiFi and accessible")
            
            # Set up OpenCV capture from GoPro stream
            # Try different stream URLs
            stream_urls = [
                f"http://{self.gopro_ip}:8080/gopro/camera/stream/start",
                f"http://{self.gopro_ip}:8080/live/amba.m3u8",  # Try HLS stream
                f"udp://@0.0.0.0:8554",  # UDP stream if enabled
            ]
            
            for stream_url in stream_urls:
                self.get_logger().info(f"🔍 Trying stream URL: {stream_url}")
                try:
                    self.cap = cv2.VideoCapture(stream_url)
                    if self.cap.isOpened():
                        # Test frame capture
                        ret, test_frame = self.cap.read()
                        if ret and test_frame is not None:
                            self.get_logger().info(f"✅ GoPro stream established: {stream_url}")
                            self.get_logger().info(f"📐 Frame shape: {test_frame.shape}")
                            self.stream_active = True
                            return
                        else:
                            self.get_logger().warn(f"⚠️ Stream opened but no frames: {stream_url}")
                            self.cap.release()
                            self.cap = None
                    else:
                        self.get_logger().warn(f"⚠️ Could not open stream: {stream_url}")
                except Exception as e:
                    self.get_logger().warn(f"⚠️ Stream error for {stream_url}: {e}")
            
            # If HTTP streams fail, provide manual instructions
            if not self.stream_active:
                self.get_logger().error("❌ Could not establish GoPro HTTP stream!")
                self.get_logger().error("🔧 Manual setup required:")
                self.get_logger().error("   1. Connect GoPro to same WiFi as Jetson")
                self.get_logger().error("   2. Enable GoPro WiFi hotspot if needed")
                self.get_logger().error("   3. Find GoPro IP address (usually 172.20.151.10)")
                self.get_logger().error("   4. Test with: curl http://172.20.151.10/gopro/camera/stream/start")
                
        except Exception as e:
            self.get_logger().error(f"GoPro HTTP setup error: {e}")

    def setup_gopro_commands(self):
        """Send commands to GoPro to prepare for streaming"""
        commands = [
            "/gopro/camera/stream/start",  # Start preview stream
            "/gopro/camera/control/wired_usb?p=1",  # Enable wired control
        ]
        
        for command in commands:
            try:
                url = f"http://{self.gopro_ip}{command}"
                response = requests.get(url, timeout=3)
                self.get_logger().info(f"📡 GoPro command: {command} -> {response.status_code}")
            except Exception as e:
                self.get_logger().warn(f"⚠️ GoPro command failed {command}: {e}")

    def process_camera_frame(self):
        """Process GoPro HTTP stream frames"""
        if not self.stream_active or self.cap is None or not self.cap.isOpened():
            self.get_logger().warn("📡 GoPro stream not active, attempting reconnection...")
            self.setup_gopro_connection()
            return
            
        try:
            # Capture frame from GoPro stream
            ret, frame = self.cap.read()
            if not ret or frame is None:
                self.get_logger().warn("📡 Failed to capture frame from GoPro stream")
                return
            
            self.frame_count += 1
            self.frames_processed += 1
            
            # Process frame with object detection
            if self.detection_enabled:
                processed_frame = self.detect_objects(frame)
            else:
                processed_frame = frame.copy()
                
            # Add frame info overlay
            self.add_overlay_info(processed_frame)
            
            # Display frame
            cv2.imshow("GoPro HTTP Object Detection", processed_frame)
            key = cv2.waitKey(1) & 0xFF
            
            if key == ord('q'):
                self.get_logger().info("🛑 Quit key pressed. Shutting down...")
                rclpy.shutdown()
            elif key == ord('s'):
                self.detection_enabled = not self.detection_enabled
                status = "ENABLED" if self.detection_enabled else "DISABLED"
                self.get_logger().info(f"🎯 Detection {status}")
            elif key == ord('r'):
                self.get_logger().info("🔄 Reconnecting to GoPro...")
                self.setup_gopro_connection()
            elif key == ord('d'):
                self.print_debug_info()
            
            # Publish processed image to ROS2 topic
            try:
                processed_msg = self.bridge.cv2_to_imgmsg(processed_frame, "bgr8")
                self.image_publisher.publish(processed_msg)
            except Exception as e:
                self.get_logger().debug(f"ROS image publish error: {e}")
                
        except Exception as e:
            self.get_logger().error(f"Frame processing error: {e}")

    def detect_objects(self, frame):
        """Run YOLO detection on GoPro frame"""
        try:
            device = '0' if torch.cuda.is_available() else 'cpu'
            
            # Run object detection
            results = self.model(
                frame, 
                stream=True, 
                verbose=False, 
                device=device,
                conf=0.25,
                iou=0.45,
                imgsz=640
            )
            
            detected_objects = set()
            detection_count = 0
            
            for result in results:
                boxes = result.boxes
                if boxes is not None:
                    for box in boxes:
                        cls = int(box.cls[0])
                        conf = float(box.conf[0])
                        
                        if conf > 0.25:
                            detection_count += 1
                            self.total_detections += 1
                            
                            class_name = self.model.names[cls] if cls < len(self.model.names) else f"class_{cls}"
                            label = f"{class_name} {conf:.2f}"
                            
                            # Bounding box coordinates
                            x1, y1, x2, y2 = map(int, box.xyxy[0])
                            
                            # Draw bounding box
                            color = self.get_class_color(cls)
                            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 3)
                            
                            # Text background
                            font_scale = 0.7
                            text_size = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, font_scale, 2)[0]
                            cv2.rectangle(frame, (x1, y1-text_size[1]-10), (x1+text_size[0]+10, y1), color, -1)
                            cv2.putText(frame, label, (x1+5, y1-5), 
                                        cv2.FONT_HERSHEY_SIMPLEX, font_scale, (255, 255, 255), 2)
                            
                            detected_objects.add(class_name)

            # Publish detected objects
            if detected_objects:
                obj_msg = String()
                obj_msg.data = ",".join(detected_objects)
                self.objects_publisher.publish(obj_msg)
                
            return frame
            
        except Exception as e:
            self.get_logger().error(f"Detection error: {e}")
            return frame

    def print_debug_info(self):
        """Print comprehensive debug information"""
        self.get_logger().info("🔧 === GOPRO HTTP DEBUG INFORMATION ===")
        self.get_logger().info(f"📊 Total frames processed: {self.frames_processed}")
        self.get_logger().info(f"🎯 Total detections: {self.total_detections}")
        self.get_logger().info(f"📈 Detection rate: {self.total_detections/max(1, self.frames_processed):.4f} per frame")
        self.get_logger().info(f"📡 Stream active: {self.stream_active}")
        self.get_logger().info(f"🏷️  Model classes: {list(self.model.names.values())}")
        self.get_logger().info(f"🖥️  Device: {'GPU' if torch.cuda.is_available() else 'CPU'}")

    def add_overlay_info(self, frame):
        """Add overlay information"""
        # Background for text
        overlay = frame.copy()
        cv2.rectangle(overlay, (10, 10), (400, 100), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.7, frame, 0.3, 0, overlay)
        
        # Frame info
        detection_rate = self.total_detections / max(1, self.frames_processed)
        info_lines = [
            f"GoPro HTTP Frame: {self.frame_count}",
            f"Stream: {'ACTIVE' if self.stream_active else 'INACTIVE'}",
            f"Detection: {'ON' if self.detection_enabled else 'OFF'}",
            f"Total Detections: {self.total_detections}",
            f"Detection Rate: {detection_rate:.3f}/frame"
        ]
        
        for i, line in enumerate(info_lines):
            cv2.putText(frame, line, (15, 30 + i*15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

    def get_class_color(self, class_id):
        """Generate different colors for different classes"""
        colors = [
            (0, 255, 0),    # Green
            (255, 0, 0),    # Blue  
            (0, 0, 255),    # Red
            (255, 255, 0),  # Cyan
            (255, 0, 255),  # Magenta
            (0, 255, 255),  # Yellow
            (128, 0, 128),  # Purple
            (255, 165, 0),  # Orange
        ]
        return colors[class_id % len(colors)]

    def destroy_node(self):
        """Clean up resources"""
        if self.cap is not None:
            self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = GoProHTTPObjectDetection()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Node stopped cleanly.")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()