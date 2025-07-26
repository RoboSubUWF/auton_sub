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

# Suppress matplotlib warnings
warnings.filterwarnings("ignore", category=UserWarning, module="matplotlib")

class JetsonObjectDetection(Node):
    def __init__(self):
        super().__init__('jetson_object_detection')
        
        # Custom trained model - try multiple possible paths
        model_paths = [
            "src/auton_sub/auton_sub/cv/model/best.pt",
            "./best.pt",
            "best.pt",
            os.path.expanduser("~/best.pt")
        ]
        
        self.model = None
        for path in model_paths:
            if os.path.exists(path):
                try:
                    # Load model with CPU optimization
                    self.model = YOLO(path)
                    # Optimize for CPU if no CUDA
                    import torch
                    if not torch.cuda.is_available():
                        self.model.model.eval()  # Set to evaluation mode for faster inference
                    self.get_logger().info(f"✅ Model loaded successfully from: {path}")
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
        
        # Camera configuration - matching DeepStream settings
        self.camera_width = 640
        self.camera_height = 480
        self.camera_fps = 60
        
        # Initialize direct camera access (like DeepStream)
        self.cap = None
        self.setup_direct_camera()
        
        # Frame processing variables
        self.frame_count = 0
        self.detection_enabled = True
        
        # Create processing timer instead of ROS subscription
        self.timer = self.create_timer(1.0/self.camera_fps, self.process_camera_frame)
        
        # Initialize display window
        cv2.namedWindow("Jetson Object Detection", cv2.WINDOW_AUTOSIZE)

        self.get_logger().info("🤖 Jetson Object Detection Node Initialized!")
        self.get_logger().info("📷 Using direct camera access (DeepStream style)")
        self.get_logger().info("💡 Press 'q' to quit, 's' to toggle detection")

    def setup_direct_camera(self):
        """Setup direct camera access similar to DeepStream approach"""
        try:
            # Try multiple approaches, starting with the most compatible
            camera_configs = [
                # Standard OpenCV camera access
                {
                    'source': 0,
                    'backend': cv2.CAP_V4L2,
                    'name': 'V4L2 Direct'
                },
                # GStreamer pipeline (Jetson optimized)
                {
                    'source': f"v4l2src device=/dev/video0 ! video/x-raw,width={self.camera_width},height={self.camera_height},framerate={self.camera_fps}/1 ! videoconvert ! appsink",
                    'backend': cv2.CAP_GSTREAMER,
                    'name': 'GStreamer V4L2'
                },
                # Simple GStreamer
                {
                    'source': f"v4l2src device=/dev/video0 ! videoconvert ! appsink",
                    'backend': cv2.CAP_GSTREAMER,  
                    'name': 'GStreamer Simple'
                },
                # Fallback to default
                {
                    'source': 0,
                    'backend': cv2.CAP_ANY,
                    'name': 'Default'
                }
            ]
            
            for config in camera_configs:
                self.get_logger().info(f"🔍 Trying {config['name']} camera access...")
                self.cap = cv2.VideoCapture(config['source'], config['backend'])
                
                if self.cap.isOpened():
                    # Configure camera properties
                    self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.camera_width)
                    self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.camera_height)
                    self.cap.set(cv2.CAP_PROP_FPS, self.camera_fps)
                    
                    # Test frame capture
                    ret, test_frame = self.cap.read()
                    if ret and test_frame is not None:
                        self.get_logger().info(f"✅ {config['name']} camera initialized successfully!")
                        self.get_logger().info(f"📐 Frame size: {test_frame.shape}")
                        return
                    else:
                        self.get_logger().warn(f"⚠️ {config['name']} opened but cannot read frames")
                        self.cap.release()
                        self.cap = None
                
            # If all methods fail
            self.get_logger().error("❌ All camera initialization methods failed")
            self.cap = None
                
        except Exception as e:
            self.get_logger().error(f"Camera setup error: {e}")
            self.cap = None

    def process_camera_frame(self):
        """Process camera frames directly (DeepStream style)"""
        if self.cap is None or not self.cap.isOpened():
            self.get_logger().warn("📷 Camera not available, attempting reconnection...")
            self.setup_direct_camera()
            return
            
        try:
            # Capture frame
            ret, frame = self.cap.read()
            if not ret or frame is None:
                self.get_logger().warn("📷 Failed to capture frame")
                return
            
            self.frame_count += 1
            
            # Process frame with object detection
            if self.detection_enabled:
                processed_frame = self.detect_objects(frame)
            else:
                processed_frame = frame.copy()
                
            # Add frame info overlay (like DeepStream OSD)
            self.add_overlay_info(processed_frame)
            
            # Display frame
            cv2.imshow("Jetson Object Detection", processed_frame)
            key = cv2.waitKey(1) & 0xFF
            
            if key == ord('q'):
                self.get_logger().info("🛑 Quit key pressed. Shutting down...")
                rclpy.shutdown()
            elif key == ord('s'):
                self.detection_enabled = not self.detection_enabled
                status = "ENABLED" if self.detection_enabled else "DISABLED"
                self.get_logger().info(f"🎯 Detection {status}")
            
            # Publish processed image to ROS2 topic
            try:
                processed_msg = self.bridge.cv2_to_imgmsg(processed_frame, "bgr8")
                self.image_publisher.publish(processed_msg)
            except Exception as e:
                self.get_logger().debug(f"ROS image publish error: {e}")
                
        except Exception as e:
            self.get_logger().error(f"Frame processing error: {e}")

    def detect_objects(self, frame):
        """Run YOLO detection on frame"""
        try:
            # Auto-detect best device (GPU if available, otherwise CPU)
            import torch
            if torch.cuda.is_available():
                device = '0'  # Use GPU
                self.get_logger().info("🚀 Using GPU for inference")
            else:
                device = 'cpu'  # Use CPU
                if not hasattr(self, '_cpu_warning_shown'):
                    self.get_logger().info("💻 Using CPU for inference (GPU not available)")
                    self._cpu_warning_shown = True
            
            # Run object detection with optimized settings
            results = self.model(frame, stream=True, verbose=False, device=device)
            
            detected_objects = set()
            detection_count = 0
            
            for result in results:
                boxes = result.boxes
                if boxes is not None:
                    for box in boxes:
                        cls = int(box.cls[0])
                        conf = float(box.conf[0])
                        
                        # Confidence threshold
                        if conf > 0.2:
                            detection_count += 1
                            label = f"{self.model.names[cls]} {conf:.2f}"
                            
                            # Bounding box coordinates
                            x1, y1, x2, y2 = map(int, box.xyxy[0])
                            
                            # Draw bounding box (DeepStream style)
                            color = self.get_class_color(cls)
                            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 3)  # Thicker border like DeepStream
                            
                            # Text background
                            text_size = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2)[0]
                            cv2.rectangle(frame, (x1, y1-text_size[1]-10), (x1+text_size[0]+10, y1), color, -1)
                            cv2.putText(frame, label, (x1+5, y1-5), 
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                            
                            detected_objects.add(self.model.names[cls])

            # Publish detected objects
            if detected_objects:
                obj_msg = String()
                obj_msg.data = ",".join(detected_objects)
                self.objects_publisher.publish(obj_msg)
                
            return frame
            
        except Exception as e:
            self.get_logger().error(f"Detection error: {e}")
            return frame

    def add_overlay_info(self, frame):
        """Add overlay information like DeepStream OSD"""
        # Background for text
        overlay = frame.copy()
        cv2.rectangle(overlay, (10, 10), (400, 80), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.7, frame, 0.3, 0, frame)
        
        # Frame info
        info_lines = [
            f"Frame: {self.frame_count}",
            f"Resolution: {frame.shape[1]}x{frame.shape[0]}",
            f"Detection: {'ON' if self.detection_enabled else 'OFF'}",
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
        node = JetsonObjectDetection()
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