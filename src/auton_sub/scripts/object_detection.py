import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO

class JetsonObjectDetection(Node):
    def __init__(self):
        super().__init__('jetson_object_detection')

        # Camera parameters based on deepstream configuration
        self.camera_width = 800
        self.camera_height = 600
        self.camera_fps = 15

        # Model configuration
        # Option 1: Built-in YOLO model
        # self.model = YOLO("yolov8n.pt")  # Smallest pre-trained model
        
        # Option 2: Custom trained model (replace with your model path)
        self.model = YOLO("/path/to/your/custom/best.pt")

        # ROS2 Publishers and Subscribers
        self.image_publisher = self.create_publisher(Image, '/processed_camera_feed', 10)
        self.objects_publisher = self.create_publisher(String, '/detected_objects', 10)
        
        # Camera subscription (adjust topic if needed)
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',  # Typical ROS2 camera topic
            self.process_image,
            10)
        
        self.bridge = CvBridge()
        
        # Jetson-specific video capture (if direct camera access is needed)
        self.setup_camera()

        self.get_logger().info("🤖 Jetson Object Detection Node Initialized!")

    def setup_camera(self):
        """Configure camera for Jetson Orin Nano with Blue Robotics Low Light Camera"""
        # GStreamer pipeline for Blue Robotics Low Light Camera on Jetson
        gst_pipeline = (
            f"nvarguscamerasrc sensor_id=0 ! "
            f"video/x-raw(memory:NVMM), width={self.camera_width}, height={self.camera_height}, "
            f"format=(string)NV12, framerate=(fraction){self.camera_fps}/1 ! "
            "nvvidconv ! video/x-raw, format=(string)BGRx ! "
            "videoconvert ! video/x-raw, format=(string)BGR ! "
            "appsink"
        )
        
        try:
            self.cap = cv2.VideoCapture(gst_pipeline, cv2.CAP_GSTREAMER)
            if not self.cap.isOpened():
                self.get_logger().error("❌ Failed to open camera!")
        except Exception as e:
            self.get_logger().error(f"Camera setup error: {e}")

    def process_image(self, msg):
        """Process incoming camera frame with object detection"""
        try:
            # Convert ROS Image to OpenCV format
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            # Run object detection
            results = self.model(frame, stream=True)

            detected_objects = set()
            for result in results:
                boxes = result.boxes
                for box in boxes:
                    # Extract class information
                    cls = int(box.cls[0])
                    conf = float(box.conf[0])
                    label = f"{self.model.names[cls]} {conf:.2f}"
                    
                    # Filtering (optional): Only detect if confidence > threshold
                    if conf > 0.5:
                        # Bounding box coordinates
                        x1, y1, x2, y2 = map(int, box.xyxy[0])
                        
                        # Draw bounding box
                        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                        cv2.putText(frame, label, (x1, y1-10), 
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
                        
                        # Track unique objects
                        detected_objects.add(self.model.names[cls])

            # Publish detected objects
            for obj in detected_objects:
                obj_msg = String()
                obj_msg.data = obj
                self.objects_publisher.publish(obj_msg)

            # Publish processed image
            processed_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
            self.image_publisher.publish(processed_msg)

            # Optional: Display frame (for local debugging)
            cv2.imshow("Jetson Object Detection", frame)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"Processing error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = JetsonObjectDetection()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node stopped cleanly.")
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()