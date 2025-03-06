import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO

class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')

        # Initialize YOLO model (change model path if needed)
        self.model = YOLO("yolov8n.pt")  # Using YOLOv8 Nano for speed on Jetson Orin Nano

        # Create a publisher for detected objects
        self.publisher_ = self.create_publisher(String, '/detected_object', 10)

        # Create a subscriber to receive camera frames
        self.subscription = self.create_subscription(
            Image,
            '/camera_feed',
            self.process_image,
            10)
        
        self.bridge = CvBridge()  # Convert ROS Image messages to OpenCV format

        self.get_logger().info("✅ Vision node started with YOLOv8!")

    def process_image(self, msg):
        """Receives an image, runs YOLO detection, and publishes results."""
        # Convert ROS Image message to OpenCV format
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        # Run YOLO object detection
        results = self.model(frame)

        detected_objects = set()  # Store unique detected objects
        for result in results:
            for box in result.boxes:
                obj_class = self.model.names[int(box.cls[0])]
                detected_objects.add(obj_class)

                # Draw bounding box and label
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(frame, obj_class, (x1, y1 - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # Publish detected objects
        for obj in detected_objects:
            msg = String()
            msg.data = obj
            self.publisher_.publish(msg)
            self.get_logger().info(f"🎯 Detected: {obj}")

        # Show the image (optional for debugging)
        cv2.imshow("Object Detection", frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()  # Close OpenCV windows

if __name__ == '__main__':
    main()
