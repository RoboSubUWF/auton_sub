# front camera (solid black usb cable.)
#works
#paths to best.pt files should be updated
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_srvs.srv import SetBool
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO
import os
import warnings
import threading

from interfaces.msg import Objcoords

# Suppress matplotlib warnings
warnings.filterwarnings("ignore", category=UserWarning, module="matplotlib")

class MultiModelObjectDetection(Node):
    def __init__(self):
        super().__init__('multi_model_object_detection')
        
        # Define your available models
        self.available_models = {
            "gate_detection": {
                "paths": [
                    "src/auton_sub/auton_sub/cv/model/prequal.pt",
                    "./prequal.pt",
                    "prequal.pt"
                ],
                "description": "Gate detection model for navigation missions"
            },
            "buoy_detection": {
                "paths": [
                    "src/auton_sub/auton_sub/cv/model/buoy_best.pt", 
                    "./buoy_best.pt",
                    "buoy_best.pt"
                ],
                "description": "Buoy detection model for buoy missions"
            },
            "coin_detection": {
                "paths": [
                    "src/auton_sub/auton_sub/cv/model/coin_best.pt",
                    "./coin_best.pt", 
                    "coin_best.pt"
                ],
                "description": "Coin detection model for coin flip missions"
            },
            "general_objects": {
                "paths": [
                    "src/auton_sub/auton_sub/cv/model/best.pt",
                    "./best.pt",
                    "best.pt",
                    os.path.expanduser("~/best.pt")
                ],
                "description": "General object detection model"
            }
        }
        
        # Current model state
        self.current_model = None
        self.current_model_name = None
        self.detection_enabled = False
        self.model_lock = threading.Lock()
        
        # ROS2 Publishers
        self.image_publisher = self.create_publisher(Image, '/processed_camera_feed', 10)
        self.objects_publisher = self.create_publisher(String, '/detected_objects', 10)
        self.model_status_publisher = self.create_publisher(String, '/current_model_status', 10)
        self.coords_publisher = self.create_publisher(Objcoords, '/detected_object_coords', 10)

        # ROS2 Subscribers for topic-based model control
        self.model_command_sub = self.create_subscription(
            String,
            '/model_command',
            self.model_command_callback,
            10
        )
        
        # ROS2 Services (keep the toggle detection service)
        self.toggle_detection_service = self.create_service(
            SetBool,
            '/toggle_detection',
            self.toggle_detection_callback
        )
        
        self.bridge = CvBridge()
        
        # Camera configuration
        self.camera_width = 640
        self.camera_height = 480
        self.camera_fps = 60
        
        # Initialize camera
        self.cap = None
        self.setup_direct_camera()
        
        # Frame processing variables
        self.frame_count = 0
        
        # Create processing timer
        self.timer = self.create_timer(1.0/self.camera_fps, self.process_camera_frame)
        
        # Initialize display window
        cv2.namedWindow("Multi-Model Object Detection", cv2.WINDOW_AUTOSIZE)

        # Auto-load gate detection model for coin toss mission
        self.get_logger().info("🤖 Multi-Model Object Detection Node Initialized!")
        self.get_logger().info("🎯 Auto-loading gate detection model for mission...")
        if self.load_model("coin_detection"):
            self.detection_enabled = True
            self.get_logger().info("✅ coin detection model loaded and enabled automatically")
        else:
            self.get_logger().warn("⚠️ Failed to auto-load gate detection model")
        
        self.get_logger().info(f"📋 Available models: {', '.join(self.available_models.keys())}")
        self.get_logger().info("🎮 Use topic /model_command and service /toggle_detection for control")

    def load_model(self, model_name):
        """Load a specific model by name"""
        if model_name not in self.available_models:
            self.get_logger().error(f"❌ Model '{model_name}' not found in available models")
            return False
            
        model_config = self.available_models[model_name]
        
        # Try to load model from available paths
        for path in model_config["paths"]:
            if os.path.exists(path):
                try:
                    # Load model with CPU optimization
                    new_model = YOLO(path)
                    # Optimize for CPU if no CUDA
                    import torch
                    if not torch.cuda.is_available():
                        new_model.model.eval()
                    
                    with self.model_lock:
                        self.current_model = new_model
                        self.current_model_name = model_name
                        
                    self.get_logger().info(f"✅ Model '{model_name}' loaded from: {path}")
                    self.get_logger().info(f"📝 Description: {model_config['description']}")
                    
                    # Publish model status
                    status_msg = String()
                    status_msg.data = f"LOADED:{model_name}"
                    self.model_status_publisher.publish(status_msg)
                    
                    return True
                    
                except Exception as e:
                    self.get_logger().warn(f"Failed to load model from {path}: {e}")
                    continue
        
        self.get_logger().error(f"❌ Could not load model '{model_name}' from any path!")
        return False

    def model_command_callback(self, msg):
        """Topic callback to set the current detection model"""
        model_name = msg.data.strip()
        
        self.get_logger().info(f"🔄 Received model command: {model_name}")
        
        if model_name == "":
            # Unload current model
            with self.model_lock:
                self.current_model = None
                self.current_model_name = None
            self.detection_enabled = False
            
            self.get_logger().info("📤 Model unloaded")
            
            status_msg = String()
            status_msg.data = "UNLOADED"
            self.model_status_publisher.publish(status_msg)
            
        else:
            # Load requested model
            success = self.load_model(model_name)
            
            if success:
                self.detection_enabled = True
                self.get_logger().info(f"✅ Model '{model_name}' loaded successfully")
            else:
                self.get_logger().error(f"❌ Failed to load model '{model_name}'")

    def toggle_detection_callback(self, request, response):
        """Service callback to enable/disable detection"""
        self.detection_enabled = request.data
        
        status = "ENABLED" if self.detection_enabled else "DISABLED"
        response.success = True
        response.message = f"Detection {status}"
        
        self.get_logger().info(f"🎯 Detection {status}")
        
        return response

    def setup_direct_camera(self):
        """Setup direct camera access"""
        try:
            camera_configs = [
                {
                    'source': 0,
                    'backend': cv2.CAP_V4L2,
                    'name': 'V4L2 Direct'
                },
                {
                    'source': f"v4l2src device=/dev/video0 ! video/x-raw,width={self.camera_width},height={self.camera_height},framerate={self.camera_fps}/1 ! videoconvert ! appsink",
                    'backend': cv2.CAP_GSTREAMER,
                    'name': 'GStreamer V4L2'
                },
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
                    self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.camera_width)
                    self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.camera_height)
                    self.cap.set(cv2.CAP_PROP_FPS, self.camera_fps)
                    
                    ret, test_frame = self.cap.read()
                    if ret and test_frame is not None:
                        self.get_logger().info(f"✅ {config['name']} camera initialized!")
                        return
                    else:
                        self.cap.release()
                        self.cap = None
                        
            self.get_logger().error("❌ All camera initialization methods failed")
            self.cap = None
                
        except Exception as e:
            self.get_logger().error(f"Camera setup error: {e}")
            self.cap = None

    def process_camera_frame(self):
        """Process camera frames"""
        if self.cap is None or not self.cap.isOpened():
            return
            
        try:
            ret, frame = self.cap.read()
            if not ret or frame is None:
                return
            
            self.frame_count += 1
            
            # Process frame with object detection if enabled and model loaded
            if self.detection_enabled and self.current_model is not None:
                with self.model_lock:
                    processed_frame = self.detect_objects(frame)
            else:
                processed_frame = frame.copy()
                
            # Add overlay info
            self.add_overlay_info(processed_frame)
            
            # Display frame
            cv2.imshow("Multi-Model Object Detection", processed_frame)
            key = cv2.waitKey(1) & 0xFF
            
            if key == ord('q'):
                rclpy.shutdown()
            elif key == ord('s'):
                self.detection_enabled = not self.detection_enabled
                status = "ENABLED" if self.detection_enabled else "DISABLED"
                self.get_logger().info(f"🎯 Detection {status}")
            elif key == ord('1'):
                # Switch to gate detection
                msg = String()
                msg.data = "gate_detection"
                self.model_command_callback(msg)
            elif key == ord('2'):
                # Switch to buoy detection
                msg = String()
                msg.data = "buoy_detection"
                self.model_command_callback(msg)
            elif key == ord('3'):
                # Switch to coin detection
                msg = String()
                msg.data = "coin_detection"
                self.model_command_callback(msg)
            
            # Publish processed image
            try:
                processed_msg = self.bridge.cv2_to_imgmsg(processed_frame, "bgr8")
                self.image_publisher.publish(processed_msg)
            except Exception as e:
                self.get_logger().debug(f"ROS image publish error: {e}")
                
        except Exception as e:
            self.get_logger().error(f"Frame processing error: {e}")

    def detect_objects(self, frame):
        """Run detection on frame using current model"""
        try:
            import torch
            device = '0' if torch.cuda.is_available() else 'cpu'
            
            # Run detection
            results = self.current_model(frame, stream=True, verbose=False, device=device)
            
            detected_objects = set()
            
            for result in results:
                boxes = result.boxes
                if boxes is not None:
                    for box in boxes:
                        cls = int(box.cls[0])
                        conf = float(box.conf[0])
                        
                        if conf > 0.5:
                            label = f"{self.current_model.names[cls]} {conf:.2f}"
                            
                            # Bounding box
                            x1, y1, x2, y2 = map(int, box.xyxy[0])
                            
                            # Draw detection
                            color = self.get_class_color(cls)
                            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 3)
                            
                            # Text background
                            text_size = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2)[0]
                            cv2.rectangle(frame, (x1, y1-text_size[1]-10), (x1+text_size[0]+10, y1), color, -1)
                            cv2.putText(frame, label, (x1+5, y1-5), 
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                            
                            detected_objects.add(self.current_model.names[cls])

                            Objcoords_msg = Objcoords()
                            Objcoords_msg.name = self.current_model.names[cls]
                            Objcoords_msg.x1 = x1
                            Objcoords_msg.x2 = x2
                            self.coords_publisher.publish(Objcoords_msg)

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
        """Add overlay information"""
        overlay = frame.copy()
        cv2.rectangle(overlay, (10, 10), (500, 120), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.7, frame, 0.3, 0, frame)
        
        info_lines = [
            f"Frame: {self.frame_count}",
            f"Model: {self.current_model_name or 'None'}",
            f"Detection: {'ON' if self.detection_enabled else 'OFF'}",
            f"Available: {len(self.available_models)} models",
            f"Controls: s=toggle, 1=gate, 2=buoy, 3=coin, q=quit"
        ]
        
        for i, line in enumerate(info_lines):
            cv2.putText(frame, line, (15, 30 + i*15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

    def get_class_color(self, class_id):
        """Generate colors for different classes"""
        colors = [
            (0, 255, 0), (255, 0, 0), (0, 0, 255), (255, 255, 0),
            (255, 0, 255), (0, 255, 255), (128, 0, 128), (255, 165, 0)
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
        node = MultiModelObjectDetection()
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
