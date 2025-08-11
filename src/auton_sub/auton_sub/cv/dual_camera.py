# Enhanced Dual camera object detection node for different camera specs
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
import time

from interfaces.msg import Objcoords

# Suppress matplotlib warnings
warnings.filterwarnings("ignore", category=UserWarning, module="matplotlib")

class DualCameraObjectDetection(Node):
    def __init__(self):
        super().__init__('dual_camera_object_detection')
        
        # Camera configurations - customize these for your specific cameras
        self.front_camera_config = {
            'width': 640,
            'height': 480,
            'fps': 60,
            'device_id': 0,  # Usually /dev/video0
            'name': 'Front Camera'
        }
        
        self.bottom_camera_config = {
            'width': 600,
            'height': 800,
            'fps': 30,
            'device_id': 2,  # Usually /dev/video1
            'name': 'Bottom Camera'
        }
        
        # Define your available models including claw_detection
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
            "claw_detection": {
                "paths": [
                    "src/auton_sub/auton_sub/cv/model/claw_detection.pt",
                    "./claw_detection.pt",
                    "claw_detection.pt",
                    os.path.expanduser("~/claw_detection.pt")
                ],
                "description": "Custom claw detection model"
            },
            "general_objects": {
                "paths": [
                    "src/auton_sub/auton_sub/cv/model/prequal.pt",
                    "./best.pt",
                    "best.pt",
                    os.path.expanduser("~/best.pt")
                ],
                "description": "General object detection model (includes table, items, baskets)"
            }
        }
        
        # Current model state
        self.current_model = None
        self.current_model_name = None
        self.detection_enabled = False
        self.model_lock = threading.Lock()
        
        # ROS2 Publishers
        self.front_image_publisher = self.create_publisher(Image, '/processed_front_camera_feed', 10)
        self.bottom_image_publisher = self.create_publisher(Image, '/processed_bottom_camera_feed', 10)
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
        
        # ROS2 Services
        self.toggle_detection_service = self.create_service(
            SetBool,
            '/toggle_detection',
            self.toggle_detection_callback
        )
        
        self.bridge = CvBridge()
        
        # Initialize both cameras with different specs
        self.front_cap = None
        self.bottom_cap = None
        self.setup_dual_cameras()
        
        # Frame processing variables
        self.frame_count = 0
        self.front_image = None
        self.bottom_image = None
        self.last_front_time = time.time()
        self.last_bottom_time = time.time()
        
        # Create processing timers for different FPS rates
        # Use the higher FPS as base timer and skip frames for lower FPS camera
        self.base_fps = max(self.front_camera_config['fps'], self.bottom_camera_config['fps'])
        self.timer = self.create_timer(1.0/self.base_fps, self.process_camera_frames)
        
        # Frame skip calculations for synchronized processing
        self.front_frame_interval = self.base_fps / self.front_camera_config['fps']
        self.bottom_frame_interval = self.base_fps / self.bottom_camera_config['fps']
        self.front_frame_counter = 0
        self.bottom_frame_counter = 0
        
        # Initialize display windows
        cv2.namedWindow("Front Camera - Object Detection", cv2.WINDOW_AUTOSIZE)
        cv2.namedWindow("Bottom Camera - Object Detection", cv2.WINDOW_AUTOSIZE)

        # Auto-load claw detection model
        self.get_logger().info("🤖 Enhanced Dual Camera Object Detection Node Initialized!")
        self.get_logger().info(f"🎯 Front Camera: {self.front_camera_config['width']}x{self.front_camera_config['height']} @ {self.front_camera_config['fps']}fps")
        self.get_logger().info(f"🎯 Bottom Camera: {self.bottom_camera_config['width']}x{self.bottom_camera_config['height']} @ {self.bottom_camera_config['fps']}fps")
        self.get_logger().info("🦀 Auto-loading claw detection model...")
        
        if self.load_model("claw_detection"):
            self.detection_enabled = True
            self.get_logger().info("✅ Claw detection model loaded and enabled automatically")
        else:
            self.get_logger().warn("⚠️ Failed to auto-load claw detection model, trying general objects...")
            if self.load_model("general_objects"):
                self.detection_enabled = True
                self.get_logger().info("✅ General objects model loaded as fallback")
            else:
                self.get_logger().error("❌ Failed to load any model")
        
        self.get_logger().info(f"📋 Available models: {', '.join(self.available_models.keys())}")
        self.get_logger().info("🎮 Use topic /model_command and service /toggle_detection for control")

    def load_model(self, model_name):
        """Load a specific model by name with Jetson Orin optimization"""
        if model_name not in self.available_models:
            self.get_logger().error(f"❌ Model '{model_name}' not found in available models")
            return False
            
        model_config = self.available_models[model_name]
        
        # Try to load model from available paths
        for path in model_config["paths"]:
            if os.path.exists(path):
                try:
                    # Load model with Jetson Orin optimization
                    new_model = YOLO(path)
                    
                    # Optimize for Jetson Orin Nano
                    import torch
                    if torch.cuda.is_available():
                        self.get_logger().info("🚀 CUDA available - using GPU acceleration")
                        device = 'cuda:0'
                    else:
                        self.get_logger().info("💻 Using CPU inference")
                        device = 'cpu'
                        new_model.model.eval()
                    
                    # Set model to evaluation mode for better performance
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

    def setup_dual_cameras(self):
        """Setup both cameras with different configurations"""
        try:
            # Setup front camera
            self.get_logger().info(f"🔍 Setting up front camera ({self.front_camera_config['width']}x{self.front_camera_config['height']} @ {self.front_camera_config['fps']}fps)...")
            
            front_configs = [
                {
                    'source': self.front_camera_config['device_id'],
                    'backend': cv2.CAP_V4L2,
                    'name': 'Front V4L2 Direct'
                },
                {
                    'source': f"v4l2src device=/dev/video{self.front_camera_config['device_id']} ! video/x-raw,width={self.front_camera_config['width']},height={self.front_camera_config['height']},framerate={self.front_camera_config['fps']}/1 ! videoconvert ! appsink",
                    'backend': cv2.CAP_GSTREAMER,
                    'name': 'Front GStreamer V4L2'
                },
                {
                    'source': self.front_camera_config['device_id'],
                    'backend': cv2.CAP_ANY,
                    'name': 'Front Default'
                }
            ]
            
            for config in front_configs:
                self.get_logger().info(f"🔍 Trying {config['name']} camera access...")
                self.front_cap = cv2.VideoCapture(config['source'], config['backend'])
                
                if self.front_cap.isOpened():
                    self.front_cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.front_camera_config['width'])
                    self.front_cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.front_camera_config['height'])
                    self.front_cap.set(cv2.CAP_PROP_FPS, self.front_camera_config['fps'])
                    
                    # Additional Jetson optimizations
                    self.front_cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # Reduce buffer for lower latency
                    
                    ret, test_frame = self.front_cap.read()
                    if ret and test_frame is not None:
                        actual_width = self.front_cap.get(cv2.CAP_PROP_FRAME_WIDTH)
                        actual_height = self.front_cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
                        actual_fps = self.front_cap.get(cv2.CAP_PROP_FPS)
                        self.get_logger().info(f"✅ Front camera initialized! Actual: {actual_width}x{actual_height} @ {actual_fps}fps")
                        break
                    else:
                        self.front_cap.release()
                        self.front_cap = None
            
            # Setup bottom camera
            self.get_logger().info(f"🔍 Setting up bottom camera ({self.bottom_camera_config['width']}x{self.bottom_camera_config['height']} @ {self.bottom_camera_config['fps']}fps)...")
            
            bottom_configs = [
                {
                    'source': self.bottom_camera_config['device_id'],
                    'backend': cv2.CAP_V4L2,
                    'name': 'Bottom V4L2 Direct'
                },
                {
                    'source': f"v4l2src device=/dev/video{self.bottom_camera_config['device_id']} ! video/x-raw,width={self.bottom_camera_config['width']},height={self.bottom_camera_config['height']},framerate={self.bottom_camera_config['fps']}/1 ! videoconvert ! appsink",
                    'backend': cv2.CAP_GSTREAMER,
                    'name': 'Bottom GStreamer V4L2'
                },
                {
                    'source': self.bottom_camera_config['device_id'],
                    'backend': cv2.CAP_ANY,
                    'name': 'Bottom Default'
                }
            ]
            
            for config in bottom_configs:
                self.get_logger().info(f"🔍 Trying {config['name']} camera access...")
                self.bottom_cap = cv2.VideoCapture(config['source'], config['backend'])
                
                if self.bottom_cap.isOpened():
                    self.bottom_cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.bottom_camera_config['width'])
                    self.bottom_cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.bottom_camera_config['height'])
                    self.bottom_cap.set(cv2.CAP_PROP_FPS, self.bottom_camera_config['fps'])
                    
                    # Additional Jetson optimizations
                    self.bottom_cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # Reduce buffer for lower latency
                    
                    ret, test_frame = self.bottom_cap.read()
                    if ret and test_frame is not None:
                        actual_width = self.bottom_cap.get(cv2.CAP_PROP_FRAME_WIDTH)
                        actual_height = self.bottom_cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
                        actual_fps = self.bottom_cap.get(cv2.CAP_PROP_FPS)
                        self.get_logger().info(f"✅ Bottom camera initialized! Actual: {actual_width}x{actual_height} @ {actual_fps}fps")
                        break
                    else:
                        self.bottom_cap.release()
                        self.bottom_cap = None
            
            if self.front_cap is None and self.bottom_cap is None:
                self.get_logger().error("❌ Both camera initialization methods failed")
            elif self.front_cap is None:
                self.get_logger().warn("⚠️ Front camera failed, using bottom camera only")
            elif self.bottom_cap is None:
                self.get_logger().warn("⚠️ Bottom camera failed, using front camera only")
            else:
                self.get_logger().info("✅ Both cameras initialized successfully!")
                
        except Exception as e:
            self.get_logger().error(f"Dual camera setup error: {e}")
            self.front_cap = None
            self.bottom_cap = None

    def process_camera_frames(self):
        """Process frames from both cameras with different FPS rates"""
        self.frame_count += 1
        
        # Determine which cameras to process this cycle
        process_front = False
        process_bottom = False
        
        # Front camera frame timing (60fps target)
        if self.front_frame_counter >= self.front_frame_interval:
            process_front = True
            self.front_frame_counter = 0
        self.front_frame_counter += 1
        
        # Bottom camera frame timing (30fps target)
        if self.bottom_frame_counter >= self.bottom_frame_interval:
            process_bottom = True
            self.bottom_frame_counter = 0
        self.bottom_frame_counter += 1
        
        # Read front camera frame
        if process_front and self.front_cap is not None and self.front_cap.isOpened():
            ret_front, front_frame = self.front_cap.read()
            if ret_front and front_frame is not None:
                self.front_image = front_frame.copy()
                self.last_front_time = time.time()
        
        # Read bottom camera frame
        if process_bottom and self.bottom_cap is not None and self.bottom_cap.isOpened():
            ret_bottom, bottom_frame = self.bottom_cap.read()
            if ret_bottom and bottom_frame is not None:
                self.bottom_image = bottom_frame.copy()
                self.last_bottom_time = time.time()
        
        # Process frames with object detection if enabled and model loaded
        processed_front = None
        processed_bottom = None
        
        if self.detection_enabled and self.current_model is not None:
            with self.model_lock:
                if self.front_image is not None and process_front:
                    processed_front = self.detect_objects(self.front_image.copy(), "front")
                elif self.front_image is not None:
                    processed_front = self.front_image.copy()
                    
                if self.bottom_image is not None and process_bottom:
                    processed_bottom = self.detect_objects(self.bottom_image.copy(), "bottom")
                elif self.bottom_image is not None:
                    processed_bottom = self.bottom_image.copy()
        else:
            processed_front = self.front_image.copy() if self.front_image is not None else None
            processed_bottom = self.bottom_image.copy() if self.bottom_image is not None else None
        
        # Add overlay info to both frames
        if processed_front is not None:
            self.add_overlay_info(processed_front, "Front Camera", self.front_camera_config)
            cv2.imshow("Front Camera - Object Detection", processed_front)
        
        if processed_bottom is not None:
            self.add_overlay_info(processed_bottom, "Bottom Camera", self.bottom_camera_config)
            cv2.imshow("Bottom Camera - Object Detection", processed_bottom)
        
        # Handle key presses
        key = cv2.waitKey(1) & 0xFF
        
        if key == ord('q'):
            rclpy.shutdown()
        elif key == ord('s'):
            self.detection_enabled = not self.detection_enabled
            status = "ENABLED" if self.detection_enabled else "DISABLED"
            self.get_logger().info(f"🎯 Detection {status}")
        elif key == ord('1'):
            msg = String()
            msg.data = "gate_detection"
            self.model_command_callback(msg)
        elif key == ord('2'):
            msg = String()
            msg.data = "buoy_detection"
            self.model_command_callback(msg)
        elif key == ord('3'):
            msg = String()
            msg.data = "coin_detection"
            self.model_command_callback(msg)
        elif key == ord('4'):
            msg = String()
            msg.data = "general_objects"
            self.model_command_callback(msg)
        elif key == ord('5'):
            msg = String()
            msg.data = "claw_detection"
            self.model_command_callback(msg)
        
        # Publish processed images
        try:
            if processed_front is not None:
                front_msg = self.bridge.cv2_to_imgmsg(processed_front, "bgr8")
                self.front_image_publisher.publish(front_msg)
            if processed_bottom is not None:
                bottom_msg = self.bridge.cv2_to_imgmsg(processed_bottom, "bgr8")
                self.bottom_image_publisher.publish(bottom_msg)
        except Exception as e:
            self.get_logger().debug(f"ROS image publish error: {e}")

    def detect_objects(self, frame, camera_name):
        """Run detection on frame using current model with Jetson optimization"""
        try:
            import torch
            device = 'cuda:0' if torch.cuda.is_available() else 'cpu'
            
            # Optimized inference settings for Jetson
            with torch.no_grad():  # Disable gradient computation for inference
                results = self.current_model(
                    frame, 
                    stream=True, 
                    verbose=False, 
                    device=device,
                    imgsz=640,  # Optimize input size
                    conf=0.5,   # Confidence threshold
                    iou=0.4     # NMS IoU threshold
                )
            
            detected_objects = set()
            
            for result in results:
                boxes = result.boxes
                if boxes is not None:
                    for box in boxes:
                        cls = int(box.cls[0])
                        conf = float(box.conf[0])
                        
                        if conf > 0.5:
                            label = f"{self.current_model.names[cls]} {conf:.2f}"
                            object_name = self.current_model.names[cls]
                            
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
                            
                            detected_objects.add(object_name)
                            
                            # Publish coordinates with camera context
                            coords_msg = Objcoords()
                            coords_msg.name = f"{object_name}_{camera_name}"
                            coords_msg.x1 = x1
                            coords_msg.x2 = x2
                            self.coords_publisher.publish(coords_msg)

            # Publish detected objects
            if detected_objects:
                obj_msg = String()
                obj_msg.data = f"{camera_name}:" + ",".join(detected_objects)
                self.objects_publisher.publish(obj_msg)
                
            return frame
            
        except Exception as e:
            self.get_logger().error(f"Detection error on {camera_name} camera: {e}")
            return frame

    def add_overlay_info(self, frame, camera_title, camera_config):
        """Add overlay information with camera-specific details"""
        overlay = frame.copy()
        cv2.rectangle(overlay, (10, 10), (550, 160), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.7, frame, 0.3, 0, frame)
        
        info_lines = [
            f"Camera: {camera_title}",
            f"Resolution: {camera_config['width']}x{camera_config['height']}",
            f"Target FPS: {camera_config['fps']}",
            f"Frame: {self.frame_count}",
            f"Model: {self.current_model_name or 'None'}",
            f"Detection: {'ON' if self.detection_enabled else 'OFF'}",
            f"Available: {len(self.available_models)} models",
            f"Controls: s=toggle, 1=gate, 2=buoy, 3=coin, 4=general, 5=claw, q=quit"
        ]
        
        for i, line in enumerate(info_lines):
            cv2.putText(frame, line, (15, 30 + i*15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

    def get_class_color(self, class_id):
        """Generate colors for different classes"""
        colors = [
            (0, 255, 0), (255, 0, 0), (0, 0, 255), (255, 255, 0),
            (255, 0, 255), (0, 255, 255), (128, 0, 128), (255, 165, 0),
            (255, 192, 203), (0, 128, 0), (128, 128, 0), (128, 0, 0)
        ]
        return colors[class_id % len(colors)]

    def destroy_node(self):
        """Clean up resources"""
        if self.front_cap is not None:
            self.front_cap.release()
        if self.bottom_cap is not None:
            self.bottom_cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = DualCameraObjectDetection()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Enhanced dual camera node stopped cleanly.")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()