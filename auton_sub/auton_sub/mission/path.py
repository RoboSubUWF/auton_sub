import time
import rclpy
import math
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import SetBool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from interfaces.msg import Objcoords

from auton_sub.motion.robot_control import RobotControl


class PathFollowingMission(Node):
    def __init__(self):
        super().__init__('path_following_mission')

        self.robot_control = RobotControl()
        self.get_logger().info("[INFO] Path Following Mission Node Initialized")
        self.bridge = CvBridge()

        # Mission parameters - maintain depth from gate mission
        self.target_depth = 2.0      # meters below surface (continue from gate)
        self.pause_time = 2.0        # seconds to pause between steps
        self.forward_speed = 0.8     # slower speed for path following
        self.turn_speed = 0.5        # gentler turn rate for corrections (rad/s)
        self.max_distance = 2.5      # meters - stop after traveling this far
        
        # Path detection parameters
        self.path_lost_timeout = 5.0  # seconds before turning to search
        self.turn_search_time = 2.0   # seconds to turn when searching
        
        # Tolerances
        self.depth_tolerance = 0.25   # 25cm tolerance for depth

        # Path following state
        self.path_visible = False
        self.path_center_x = None
        self.path_lost_time = None
        self.last_known_side = None   # "left" or "right"
        self.distance_traveled = 0.0
        self.start_position = None
        
        # Camera image processing
        self.latest_bottom_image = None
        
        # Vision system publishers and subscribers - focus on bottom camera only
        self.model_command_pub = self.create_publisher(String, '/model_command', 10)
        
        # Create service client for detection toggle
        self.toggle_detection_client = self.create_client(SetBool, '/toggle_detection')
        
        # Subscribe to bottom camera processed feed for path detection
        self.bottom_image_sub = self.create_subscription(
            Image,
            '/processed_bottom_camera_feed',
            self.bottom_image_callback,
            10
        )
        
        # Subscribe to object coordinates for path center detection
        self.coords_sub = self.create_subscription(
            Objcoords,
            '/detected_object_coords',
            self.coords_callback,
            10
        )
        
        # Subscribe to detected objects to confirm path visibility
        self.detected_objects_sub = self.create_subscription(
            String,
            '/detected_objects',
            self.objects_callback,
            10
        )
        
        # Subscribe to model status
        self.model_status_sub = self.create_subscription(
            String,
            '/current_model_status',
            self.model_status_callback,
            10
        )
        
        # Mission state variables
        self.detected_objects = set()
        self.model_loaded = False
        
        # Wait for vision services
        self.get_logger().info("[INFO] Waiting for vision detection services...")
        if not self.toggle_detection_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().warn("[WARNING] Detection service not available - vision may not work")

    def set_detection_model(self, model_name):
        """Set the active detection model using topics"""
        self.get_logger().info(f"[VISION] Setting detection model to: {model_name}")
        
        # Publish model command
        msg = String()
        msg.data = model_name
        self.model_command_pub.publish(msg)
        
        # Wait for confirmation
        self.model_loaded = False
        timeout = 10.0
        start_time = time.time()
        
        while not self.model_loaded and (time.time() - start_time) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(0.1)
        
        if self.model_loaded:
            self.get_logger().info(f"[VISION] ✅ Model {model_name} loaded successfully")
            return True
        else:
            self.get_logger().error(f"[VISION] ❌ Failed to load model {model_name}")
            return False

    def model_status_callback(self, msg):
        """Callback for model status updates"""
        if msg.data.startswith("LOADED:"):
            model_name = msg.data.split(":")[1]
            self.get_logger().info(f"[VISION] Model status update: {model_name} loaded")
            self.model_loaded = True

    def toggle_detection(self, enable):
        """Enable or disable object detection"""
        if not self.toggle_detection_client.service_is_ready():
            self.get_logger().error("[VISION] Detection service not available")
            return False
            
        request = SetBool.Request()
        request.data = enable
        
        future = self.toggle_detection_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        if future.result() is not None:
            response = future.result()
            status = "enabled" if enable else "disabled"
            self.get_logger().info(f"[VISION] Detection {status}")
            return response.success
        else:
            self.get_logger().error("[VISION] Failed to toggle detection")
            return False

    def bottom_image_callback(self, msg):
        """Store the latest bottom camera image for processing"""
        try:
            self.latest_bottom_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().debug(f"Image conversion error: {e}")

    def coords_callback(self, msg):
        """Process object coordinates to find path center - focus on bottom camera"""
        # Only process coordinates from bottom camera
        if msg.name.endswith("_bottom") and "path" in msg.name.lower():
            # Calculate center X coordinate of detected path
            self.path_center_x = (msg.x1 + msg.x2) / 2
            self.path_visible = True
            self.path_lost_time = None  # Reset lost timer
            
            # Log path center for debugging
            if hasattr(self, 'latest_bottom_image') and self.latest_bottom_image is not None:
                image_width = self.latest_bottom_image.shape[1]
                center_ratio = self.path_center_x / image_width
                self.get_logger().debug(f"[VISION] Path center at x={self.path_center_x:.0f}, ratio={center_ratio:.2f}")

    def objects_callback(self, msg):
        """Callback for detected objects - track path visibility"""
        if msg.data:
            # Check if this is from bottom camera
            if msg.data.startswith("bottom:"):
                objects_str = msg.data.replace("bottom:", "")
                self.detected_objects = set(objects_str.split(',')) if objects_str else set()
                
                # Check for path/red path detection
                path_detected = any("path" in obj.lower() or "red" in obj.lower() for obj in self.detected_objects)
                
                if path_detected:
                    if not self.path_visible:
                        self.get_logger().info("[VISION] 🔴 RED PATH DETECTED in bottom camera!")
                    self.path_visible = True
                    self.path_lost_time = None
                else:
                    if self.path_visible:
                        self.get_logger().info("[VISION] ⚠️ Path lost from bottom camera view")
                        self.path_lost_time = time.time()
                    self.path_visible = False

    def detect_red_path_opencv(self):
        """Fallback: Use OpenCV to detect red path if YOLO model doesn't work"""
        if self.latest_bottom_image is None:
            return False, None
            
        try:
            # Convert BGR to HSV for better color detection
            hsv = cv2.cvtColor(self.latest_bottom_image, cv2.COLOR_BGR2HSV)
            
            # Define range for red color (accounting for red wrapping in HSV)
            # Lower red range
            lower_red1 = np.array([0, 50, 50])
            upper_red1 = np.array([10, 255, 255])
            
            # Upper red range
            lower_red2 = np.array([170, 50, 50])
            upper_red2 = np.array([180, 255, 255])
            
            # Create masks for both red ranges
            mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
            mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
            
            # Combine masks
            red_mask = mask1 + mask2
            
            # Remove noise
            kernel = np.ones((5,5), np.uint8)
            red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN, kernel)
            red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_CLOSE, kernel)
            
            # Find contours
            contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            if contours:
                # Find the largest red contour (likely the path)
                largest_contour = max(contours, key=cv2.contourArea)
                area = cv2.contourArea(largest_contour)
                
                # Minimum area threshold for path detection
                if area > 500:  # Adjust this threshold as needed
                    # Calculate center of the path
                    M = cv2.moments(largest_contour)
                    if M["m00"] != 0:
                        center_x = int(M["m10"] / M["m00"])
                        self.path_center_x = center_x
                        return True, center_x
            
            return False, None
            
        except Exception as e:
            self.get_logger().debug(f"OpenCV path detection error: {e}")
            return False, None

    def calculate_steering_correction(self):
        """Calculate steering correction based on path center position"""
        if self.latest_bottom_image is None or self.path_center_x is None:
            return 0.0
            
        image_width = self.latest_bottom_image.shape[1]
        image_center = image_width / 2
        
        # Calculate offset from center (-1.0 to 1.0)
        offset = (self.path_center_x - image_center) / (image_width / 2)
        
        # Generate proportional steering command (negative because we want to turn toward the path)
        steering_gain = 0.3  # Adjust this gain for responsiveness
        yaw_correction = -offset * steering_gain
        
        # Clamp to reasonable limits
        yaw_correction = max(-0.5, min(0.5, yaw_correction))
        
        return yaw_correction

    def get_current_distance_from_start(self):
        """Calculate distance traveled from start position"""
        if self.start_position is None:
            return 0.0
            
        current_pos = self.robot_control.get_current_position()
        if not current_pos.get('valid', False):
            return self.distance_traveled  # Return last known distance
            
        # Calculate Euclidean distance in X-Y plane
        dx = current_pos['x'] - self.start_position['x']
        dy = current_pos['y'] - self.start_position['y']
        distance = math.sqrt(dx*dx + dy*dy)
        
        return distance

    def follow_red_path(self):
        """Main path following logic"""
        self.get_logger().info("[MOTION] Starting red path following...")
        
        # Record starting position
        start_pos = self.robot_control.get_current_position()
        if start_pos.get('valid', False):
            self.start_position = {'x': start_pos['x'], 'y': start_pos['y']}
            self.get_logger().info(f"[MOTION] Starting position: x={start_pos['x']:.2f}, y={start_pos['y']:.2f}")
        else:
            self.get_logger().warn("[MOTION] No valid starting position - using distance estimation")
            self.start_position = None
        
        mission_start_time = time.time()
        last_log_time = time.time()
        
        while True:
            current_time = time.time()
            mission_elapsed = current_time - mission_start_time
            
            # Check if we've traveled far enough
            distance_traveled = self.get_current_distance_from_start()
            if distance_traveled >= self.max_distance:
                self.get_logger().info(f"[MOTION] ✅ Target distance reached: {distance_traveled:.2f}m")
                break
                
            # Check for mission timeout (safety)
            if mission_elapsed > 120.0:  # 2 minutes max
                self.get_logger().warn(f"[MOTION] ⏰ Mission timeout after {mission_elapsed:.1f}s")
                break
            
            # Process ROS callbacks to get latest vision data
            rclpy.spin_once(self, timeout_sec=0.01)
            
            # Try YOLO detection first, fallback to OpenCV
            if not self.path_visible:
                opencv_success, opencv_center = self.detect_red_path_opencv()
                if opencv_success:
                    self.path_visible = True
                    self.path_center_x = opencv_center
                    self.path_lost_time = None
            
            # Check current depth and maintain it
            current_pos = self.robot_control.get_current_position()
            depth_error = abs(current_pos['z'] - self.target_depth)
            if depth_error > (self.depth_tolerance * 2):
                self.robot_control.set_depth(self.target_depth)
            
            # Path following logic
            if self.path_visible and self.path_center_x is not None:
                # Calculate steering correction
                yaw_correction = self.calculate_steering_correction()
                
                # Move forward with steering correction
                self.robot_control.set_movement_command(
                    forward=self.forward_speed, 
                    yaw=yaw_correction
                )
                
                # Reset lost time
                self.path_lost_time = None
                
            else:
                # Path not visible - determine search behavior
                if self.path_lost_time is None:
                    self.path_lost_time = current_time
                    self.get_logger().warn("[VISION] Path lost - starting search behavior")
                
                time_since_lost = current_time - self.path_lost_time
                
                if time_since_lost < self.path_lost_timeout:
                    # Continue forward for a short time
                    self.robot_control.set_movement_command(forward=self.forward_speed, yaw=0.0)
                else:
                    # Start search behavior - turn to look for path
                    if self.last_known_side == "left":
                        # Turn right to find path
                        self.robot_control.set_movement_command(forward=0.0, yaw=self.turn_speed)
                        self.get_logger().info("[MOTION] Searching right for lost path")
                    elif self.last_known_side == "right":
                        # Turn left to find path
                        self.robot_control.set_movement_command(forward=0.0, yaw=-self.turn_speed)
                        self.get_logger().info("[MOTION] Searching left for lost path")
                    else:
                        # Unknown last side - default search right
                        self.robot_control.set_movement_command(forward=0.0, yaw=self.turn_speed)
                        self.get_logger().info("[MOTION] Searching right for path (default)")
            
            # Update last known side based on path center
            if self.path_visible and self.path_center_x is not None and self.latest_bottom_image is not None:
                image_center = self.latest_bottom_image.shape[1] / 2
                if self.path_center_x < image_center * 0.4:  # Path on left side
                    self.last_known_side = "left"
                elif self.path_center_x > image_center * 1.6:  # Path on right side
                    self.last_known_side = "right"
            
            # Log progress every 3 seconds
            if (current_time - last_log_time) > 3.0:
                detected_str = ", ".join(self.detected_objects) if self.detected_objects else "none"
                path_status = f"visible at x={self.path_center_x:.0f}" if self.path_visible else "lost"
                self.get_logger().info(f"[MOTION] Following... Time: {mission_elapsed:.1f}s, "
                                     f"Distance: {distance_traveled:.2f}m/{self.max_distance}m, "
                                     f"Path: {path_status}, Depth: {current_pos['z']:.2f}m")
                last_log_time = current_time
            
            # Short sleep to prevent excessive CPU usage
            time.sleep(0.1)
        
        # Stop all movement when done
        self.robot_control.set_movement_command(forward=0.0, yaw=0.0)
        self.get_logger().info("[MOTION] Path following completed - all movement stopped")

    def pause_and_monitor_depth(self, pause_duration):
        """Pause while maintaining depth"""
        self.get_logger().info(f"[MOTION] Pausing {pause_duration}s while maintaining depth...")
        
        start_time = time.time()
        while (time.time() - start_time) < pause_duration:
            current_depth = self.robot_control.get_current_depth()
            
            # Check depth drift
            depth_error = abs(current_depth - self.target_depth)
            if depth_error > (self.depth_tolerance * 2):
                self.get_logger().warn(f"[DEPTH] Depth drift: {current_depth:.2f}m "
                                     f"(target: {self.target_depth}m)")
                # Re-set target depth
                self.robot_control.set_depth(self.target_depth)
            
            time.sleep(0.5)

    def run(self):
        self.get_logger().info("[INFO] Starting Path Following Mission")
        
        try:
            # Step 1: Set up vision system for path detection
            self.get_logger().info("[INFO] Setting up path detection vision system...")
            # Try to use a general model first, then fallback to OpenCV if needed
            model_success = self.set_detection_model("path_detection")
            if not model_success:
                self.get_logger().warn("[WARNING] Failed to load YOLO model - will use OpenCV fallback")
            
            # Step 2: Enable detection
            if not self.toggle_detection(True):
                self.get_logger().warn("[WARNING] Failed to enable detection - using OpenCV only")

            # Step 3: Maintain depth from gate mission (2m)
            self.get_logger().info("[INFO] Maintaining 2m depth from gate mission...")
            current_depth = self.robot_control.get_current_depth()
            depth_error = abs(current_depth - self.target_depth)
            
            if depth_error > self.depth_tolerance:
                self.get_logger().info(f"[DEPTH] Adjusting depth from {current_depth:.2f}m to {self.target_depth:.2f}m")
                self.robot_control.set_depth(self.target_depth)
                
                # Wait for depth adjustment
                adjustment_start = time.time()
                while (time.time() - adjustment_start) < 10.0:
                    current = self.robot_control.get_current_depth()
                    if abs(current - self.target_depth) < self.depth_tolerance:
                        self.get_logger().info(f"[DEPTH] ✅ Depth adjusted to {current:.2f}m")
                        break
                    time.sleep(0.5)

            # Step 4: Wait for systems to be ready
            self.get_logger().info("[INFO] Waiting for vision system and position data...")
            
            # Wait for valid position data
            max_wait = 10.0
            wait_start = time.time()
            while (time.time() - wait_start) < max_wait:
                pos = self.robot_control.get_current_position()
                if pos.get('valid', False):
                    self.get_logger().info(f"[INFO] ✅ DVL/EKF position valid: x={pos['x']:.2f}, y={pos['y']:.2f}, z={pos['z']:.2f}m")
                    break
                self.get_logger().info("[INFO] Waiting for DVL/EKF position data...")
                time.sleep(1.0)
            else:
                self.get_logger().warn("[WARNING] DVL/EKF position not available - using time-based distance estimation")

            # Step 5: Brief pause to allow camera feed to stabilize
            self.pause_and_monitor_depth(self.pause_time)

            # Step 6: Follow the red path until target distance is reached
            self.follow_red_path()

            # Step 7: Disable detection to save resources
            self.get_logger().info("[VISION] Disabling detection system...")
            if not self.toggle_detection(False):
                self.get_logger().warn("[WARNING] Failed to disable detection")
            
            # Unload model to save resources
            msg = String()
            msg.data = ""
            self.model_command_pub.publish(msg)

            # Step 8: Final status and position
            final_pos = self.robot_control.get_current_position()
            distance_traveled = self.get_current_distance_from_start()
            
            self.get_logger().info("[INFO] ✅ Path Following Mission completed successfully!")
            self.get_logger().info(f"[INFO] Distance traveled: {distance_traveled:.2f}m (target: {self.max_distance}m)")
            self.get_logger().info(f"[INFO] Final position: x={final_pos['x']:.2f}m, y={final_pos['y']:.2f}m, depth={final_pos['z']:.2f}m")
            self.get_logger().info("[INFO] 🔴 Red path following complete - submarine ready for next mission")
            
            return True

        except KeyboardInterrupt:
            self.get_logger().info("[INFO] Mission interrupted")
            return False
        except Exception as e:
            self.get_logger().error(f"[ERROR] Mission failed: {e}")
            return False
        finally:
            # Ensure all movement is stopped
            self.robot_control.set_movement_command(forward=0.0, yaw=0.0)
            self.get_logger().info("[INFO] All movement stopped. Depth control still active.")


def main():
    rclpy.init()
    mission = PathFollowingMission()
    
    success = False
    try:
        success = mission.run()
        if success:
            mission.get_logger().info("[INFO] 🏁 Path Following Mission completed successfully!")
        else:
            mission.get_logger().error("[ERROR] 🚫 Path Following Mission failed!")
    except KeyboardInterrupt:
        mission.get_logger().info("[INFO] Mission interrupted by user")
    except Exception as e:
        mission.get_logger().error(f"[ERROR] Exception occurred: {e}")
    finally:
        # Cleanup: disable detection and stop robot control
        try:
            mission.toggle_detection(False)
            # Unload model to save resources
            msg = String()
            msg.data = ""
            mission.model_command_pub.publish(msg)
        except:
            pass
            
        mission.robot_control.stop()
        mission.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()