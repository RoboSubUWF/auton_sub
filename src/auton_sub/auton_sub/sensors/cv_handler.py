import importlib
import json
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import threading


class CVHandler(Node):
    def __init__(self):
        super().__init__('cv_handler')
        self.active_cv_scripts = {}
        self.bridge = CvBridge()

    def start_cv(self, mission_name, module_path, camera_topic):
        if mission_name in self.active_cv_scripts:
            self.get_logger().warn(f"Mission '{mission_name}' already running.")
            return

        try:
            module = importlib.import_module(module_path)
            cv_class = getattr(module, "CV", None)
            if not cv_class:
                self.get_logger().error(f"No class 'CV' found in {module_path}")
                return

            cv_instance = cv_class()
            handler = _ScriptHandler(self, mission_name, cv_instance, camera_topic)
            self.active_cv_scripts[mission_name] = handler

        except Exception as e:
            self.get_logger().error(f"Error starting mission {mission_name}: {e}")

    def stop_cv(self, mission_name):
        if mission_name in self.active_cv_scripts:
            self.active_cv_scripts[mission_name].stop()
            del self.active_cv_scripts[mission_name]
        else:
            self.get_logger().warn(f"Mission '{mission_name}' not found.")


class _ScriptHandler:
    def __init__(self, node: Node, mission_name, cv_instance, camera_topic):
        self.node = node
        self.cv_instance = cv_instance
        self.camera_topic = camera_topic
        self.mission_name = mission_name
        self.bridge = CvBridge()
        self.running = True
        self.sub = node.create_subscription(Image, camera_topic, self.callback, 10)
        self.pub_result = node.create_publisher(String, f"cv/{mission_name}/result", 10)
        self.pub_viz = node.create_publisher(Image, f"cv/{mission_name}/viz", 10)

    def callback(self, msg: Image):
        if not self.running:
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            result = self.cv_instance.run(frame)

            # If result is a tuple
            if isinstance(result, tuple) and len(result) == 2:
                motion_cmds, viz_frame = result
                if viz_frame is not None:
                    viz_msg = self.bridge.cv2_to_imgmsg(viz_frame, encoding='bgr8')
                    self.pub_viz.publish(viz_msg)
            elif isinstance(result, dict):
                motion_cmds = result
            else:
                self.node.get_logger().warn("CV script returned invalid format.")
                return

            self.pub_result.publish(String(data=json.dumps(motion_cmds)))

        except Exception as e:
            self.node.get_logger().error(f"Error processing frame in mission '{self.mission_name}': {e}")

    def stop(self):
        self.running = False
        self.node.destroy_subscription(self.sub)
        self.node.get_logger().info(f"Stopped mission '{self.mission_name}'")


def main(args=None):
    rclpy.init(args=args)
    node = CVHandler()

    # Start a mission manually for testing
    node.start_cv(
        mission_name="buoy",
        module_path="auton_sub.cv.cointoss_cv",  # Replace with actual import path
        camera_topic="/camera/front/image_raw"
    )

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
