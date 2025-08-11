# -*- coding: utf-8 -*-

import time
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import SetBool
from geometry_msgs.msg import Point
from auton_sub.motion.robot_control import RobotControl


class RedPipeMission(Node):
    def __init__(self):
        super().__init__('red_pipe_mission')

        self.robot_control = RobotControl()
        self.get_logger().info("[INFO] Red Pipe Mission Node Initialized")

        # Mission parameters
        self.target_depth_feet = 2.5      # feet below surface
        self.target_depth_meters = self.target_depth_feet * 0.3048  # convert to meters
        self.pause_time = 2.0             # seconds to pause between steps
        self.turn_speed = 1.0             # yaw rate for turning (rad/s)
        self.forward_speed = 0.5          # forward speed for movement
        self.search_timeout = 30.0        # 30 seconds max to find pipe
        self.distance_forward_feet = 3.0  # 3 feet forward movement
        self.distance_forward_meters = self.distance_forward_feet * 0.3048  # convert to meters
        
        # Tolerances
        self.depth_tolerance = 0.1        # 10cm tolerance for depth
        self.centering_tolerance = 50     # pixels from center to consider "centered"
        self.large_bbox_threshold = 150   # bounding box width threshold for "big enough"
        self.position_tolerance = 0.1     # 10cm tolerance for position moves

        # Vision system publishers and subscribers
        self.model_command_pub = self.create_publisher(String, '/model_command', 10)
        
        # Create service client for detection toggle
        self.toggle_detection_client = self.create_client(SetBool, '/toggle_detection')
        
        # Subscribe to object detection results - expecting bounding box info
        self.detected_objects_sub = self.create_subscription(
            String,
            '/detected_objects',
            self.objects_callback,
            10
        )
        
        # Subscribe to bounding box center for centering control
        self.bbox_center_sub = self.create_subscription(
            Point,
            '/red_pipe_center',
            self.bbox_center_callback,
            10
        )
        
        # Subscribe to bounding box size for distance estimation
        self.bbox_size_sub = self.create_subscription(
            Point,
            '/red_pipe_size',
            self.bbox_size_callback,
            10
        )
        
        # Subscribe to model status (to confirm model changes)
        self.model_status_sub = self.create_subscription(
            String,
            '/current_model_status',
            self.model_status_callback,
            10
        )
        
        # Mission state variables
        self.red_pipe_detected = False
        self.pipe_centered = False
        self.pipe_close_enough = False
        self.bbox_center_x = 320  # assume 640x480 camera, center at 320
        self.bbox_width = 0
        self.bbox_height = 0
        self.camera_width = 640   # adjust based on your camera resolution
        self.camera_height = 480  # adjust based on your camera resolution
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

    def objects_callback(self, msg):
        """Callback for detected objects"""
        if msg.data:
            detected_objects = set(msg.data.split(','))
            
            if 'red_pipe' in detected_objects:
                if not self.red_pipe_detected:
                    self.get_logger().info("[VISION] 🔴 RED PIPE DETECTED!")
                self.red_pipe_detected = True
            else:
                if self.red_pipe_detected:
                    self.get_logger().warn("[VISION] Red pipe lost from view")
                self.red_pipe_detected = False

    def bbox_center_callback(self, msg):
        """Callback for bounding box center position"""
        self.bbox_center_x = msg.x
        center_error = abs(self.bbox_center_x - (self.camera_width / 2))
        self.pipe_centered = center_error < self.centering_tolerance
        
        if self.red_pipe_detected and center_error > self.centering_tolerance:
            self.get_logger().info(f"[VISION] Pipe center: {self.bbox_center_x:.0f}, error: {center_error:.0f}px")

    def bbox_size_callback(self, msg):
        """Callback for bounding box size"""
        self.bbox_width = msg.x
        self.bbox_height = msg.y
        self.pipe_close_enough = self.bbox_width > self.large_bbox_threshold
        
        if self.red_pipe_detected:
            self.get_logger().info(f"[VISION] Pipe size: {self.bbox_width:.0f}x{self.bbox_height:.0f}, close enough: {self.pipe_close_enough}")

    def set_depth_feet(self, target_depth_feet):
        """Set the target depth in feet (converted to meters internally)"""
        target_depth_meters = target_depth_feet * 0.3048
        self.robot_control.set_depth(target_depth_meters)
        self.get_logger().info(f"[DEPTH] Target depth set to: {target_depth_feet}ft ({target_depth_meters:.2f}m)")

    def wait_for_depth(self, target_depth_meters, timeout=30.0):
        """Wait until target depth is reached"""
        self.get_logger().info(f"[DEPTH] Waiting to reach {target_depth_meters:.2f}m...")
        
        start_time = time.time()
        while (time.time() - start_time) < timeout:
            current_depth = self.robot_control.get_current_depth()
            error = abs(current_depth - target_depth_meters)
            
            if error < self.depth_tolerance:
                self.get_logger().info(f"[DEPTH] ✅ Target depth reached: {current_depth:.2f}m")
                return True
                
            time.sleep(0.5)
        
        self.get_logger().warn(f"[DEPTH] ⏰ Depth timeout - Current: {self.robot_control.get_current_depth():.2f}m")
        return False

    def center_red_pipe_in_camera(self, timeout=30.0):
        """Turn until red pipe is centered in camera"""
        self.get_logger().info("[MOTION] Centering red pipe in camera...")
        
        start_time = time.time()
        
        while (time.time() - start_time) < timeout:
            if not self.red_pipe_detected:
                self.get_logger().warn("[VISION] Red pipe not detected, stopping centering")
                return False
                
            if self.pipe_centered:
                self.get_logger().info("[VISION] ✅ Red pipe centered!")
                self.robot_control.set_movement_command(forward=0.0, yaw=0.0)
                return True
            
            # Calculate turning direction and speed
            center_error = self.bbox_center_x - (self.camera_width / 2)
            turn_direction = -1.0 if center_error > 0 else 1.0  # Turn left if pipe is on right
            
            self.robot_control.set_movement_command(forward=0.0, yaw=turn_direction * self.turn_speed)
            
            # Allow ROS to process callbacks
            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(0.1)
        
        self.get_logger().warn("[MOTION] ⏰ Centering timeout")
        self.robot_control.set_movement_command(forward=0.0, yaw=0.0)
        return False

    def drive_towards_pipe_until_close(self, timeout=60.0):
        """Drive forward towards red pipe until bounding box is large enough"""
        self.get_logger().info("[MOTION] Driving towards red pipe...")
        
        start_time = time.time()
        
        while (time.time() - start_time) < timeout:
            if not self.red_pipe_detected:
                self.get_logger().warn("[VISION] Red pipe lost, stopping approach")
                break
                
            if self.pipe_close_enough:
                self.get_logger().info(f"[VISION] ✅ Close enough to pipe! BBox width: {self.bbox_width:.0f}px")
                break
            
            # Continue driving forward while adjusting for centering
            yaw_correction = 0.0
            if not self.pipe_centered:
                center_error = self.bbox_center_x - (self.camera_width / 2)
                yaw_correction = -0.3 if center_error > 0 else 0.3  # Gentle correction
            
            self.robot_control.set_movement_command(forward=self.forward_speed, yaw=yaw_correction)
            
            # Log progress every 3 seconds
            elapsed = time.time() - start_time
            if int(elapsed) % 3 == 0 and elapsed > 2.0:
                self.get_logger().info(f"[MOTION] Approaching pipe... Time: {elapsed:.1f}s, BBox: {self.bbox_width:.0f}px")
            
            # Allow ROS to process callbacks
            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(0.1)
        
        # Stop movement
        self.robot_control.set_movement_command(forward=0.0, yaw=0.0)
        
        if self.pipe_close_enough:
            self.get_logger().info("[MOTION] ✅ Successfully approached red pipe")
            return True
        else:
            self.get_logger().warn("[MOTION] ❌ Failed to get close enough to pipe")
            return False

    def turn_degrees(self, degrees):
        """Turn by specified degrees (positive = right, negative = left)"""
        direction = "right" if degrees > 0 else "left"
        self.get_logger().info(f"[MOTION] Turning {abs(degrees)}° {direction}...")
        
        # Get current heading
        current_pos = self.robot_control.get_current_position()
        start_yaw = current_pos['yaw']
        target_yaw = start_yaw + math.radians(degrees)
        
        # Normalize target yaw to [-pi, pi]
        while target_yaw > math.pi:
            target_yaw -= 2 * math.pi
        while target_yaw < -math.pi:
            target_yaw += 2 * math.pi
        
        # Turn towards target
        turn_direction = 1.0 if degrees > 0 else -1.0
        self.robot_control.set_movement_command(forward=0.0, yaw=turn_direction * self.turn_speed)
        
        # Wait until turn is complete
        timeout = 30.0
        start_time = time.time()
        
        while (time.time() - start_time) < timeout:
            current_pos = self.robot_control.get_current_position()
            current_yaw = current_pos['yaw']
            
            # Calculate angular difference
            yaw_diff = target_yaw - current_yaw
            # Normalize difference to [-pi, pi]
            while yaw_diff > math.pi:
                yaw_diff -= 2 * math.pi
            while yaw_diff < -math.pi:
                yaw_diff += 2 * math.pi
            
            if abs(yaw_diff) < math.radians(5):  # 5 degree tolerance
                self.get_logger().info(f"[MOTION] ✅ Turn completed! Yaw error: {math.degrees(yaw_diff):.1f}°")
                break
                
            time.sleep(0.1)
        
        # Stop turning
        self.robot_control.set_movement_command(forward=0.0, yaw=0.0)
        time.sleep(1.0)  # Brief pause to stabilize

    def drive_forward_distance(self, distance_meters):
        """Drive forward for specified distance using DVL/EKF position feedback"""
        distance_feet = distance_meters / 0.3048
        self.get_logger().info(f"[MOTION] Driving forward {distance_feet:.1f}ft ({distance_meters:.2f}m)...")
        
        # Get starting position
        start_pos = self.robot_control.get_current_position()
        start_x = start_pos['x']
        start_y = start_pos['y']
        
        # Calculate target position
        current_yaw = start_pos['yaw']
        target_x = start_x + distance_meters * math.cos(current_yaw)
        target_y = start_y + distance_meters * math.sin(current_yaw)
        
        self.get_logger().info(f"[MOTION] Target position: ({target_x:.2f}, {target_y:.2f})")
        
        # Drive towards target
        self.robot_control.set_movement_command(forward=self.forward_speed, yaw=0.0)
        
        timeout = 30.0
        start_time = time.time()
        
        while (time.time() - start_time) < timeout:
            current_pos = self.robot_control.get_current_position()
            
            # Calculate distance to target
            dx = target_x - current_pos['x']
            dy = target_y - current_pos['y']
            distance_remaining = math.sqrt(dx*dx + dy*dy)
            
            if distance_remaining < self.position_tolerance:
                distance_traveled = math.sqrt((current_pos['x'] - start_x)**2 + (current_pos['y'] - start_y)**2)
                self.get_logger().info(f"[MOTION] ✅ Distance completed! Traveled: {distance_traveled:.2f}m")
                break
                
            # Log progress every 2 seconds
            elapsed = time.time() - start_time
            if int(elapsed) % 2 == 0 and elapsed > 1.0:
                distance_traveled = math.sqrt((current_pos['x'] - start_x)**2 + (current_pos['y'] - start_y)**2)
                self.get_logger().info(f"[MOTION] Forward progress: {distance_traveled:.2f}m / {distance_meters:.2f}m")
            
            time.sleep(0.1)
        
        # Stop movement
        self.robot_control.set_movement_command(forward=0.0, yaw=0.0)
        time.sleep(1.0)  # Brief pause to stabilize

    def rotate_until_pipe_centered(self, timeout=45.0):
        """Rotate right until red pipe is centered in camera"""
        self.get_logger().info("[MOTION] Rotating right to find and center red pipe...")
        
        start_time = time.time()
        
        # Start rotating right
        self.robot_control.set_movement_command(forward=0.0, yaw=1.0 * self.turn_speed)
        
        while (time.time() - start_time) < timeout:
            # Check if pipe is detected and centered
            if self.red_pipe_detected and self.pipe_centered:
                self.get_logger().info("[VISION] ✅ Red pipe found and centered!")
                self.robot_control.set_movement_command(forward=0.0, yaw=0.0)
                return True
                
            # Log progress every 5 seconds
            elapsed = time.time() - start_time
            if int(elapsed) % 5 == 0 and elapsed > 4.0:
                status = "detected" if self.red_pipe_detected else "not detected"
                self.get_logger().info(f"[MOTION] Searching... Time: {elapsed:.1f}s, Pipe: {status}")
            
            # Allow ROS to process callbacks
            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(0.1)
        
        # Stop rotation
        self.robot_control.set_movement_command(forward=0.0, yaw=0.0)
        self.get_logger().warn("[MOTION] ⏰ Pipe search timeout")
        return False

    def pause_and_monitor(self, duration):
        """Pause while maintaining depth and monitoring systems"""
        self.get_logger().info(f"[MOTION] Pausing for {duration}s while maintaining depth...")
        
        start_time = time.time()
        while (time.time() - start_time) < duration:
            # Monitor depth
            current_depth = self.robot_control.get_current_depth()
            depth_error = abs(current_depth - self.target_depth_meters)
            
            if depth_error > (self.depth_tolerance * 2):
                self.get_logger().warn(f"[DEPTH] Depth drift: {current_depth:.2f}m (target: {self.target_depth_meters:.2f}m)")
                self.robot_control.set_depth(self.target_depth_meters)
            
            time.sleep(0.5)

    def run(self):
        self.get_logger().info("[INFO] Starting Red Pipe Following Mission")
        
        try:
            # Step 1: Set up vision system for red pipe detection
            self.get_logger().info("[INFO] Setting up vision system for red pipe detection...")
            if not self.set_detection_model("slalom"):
                self.get_logger().error("[ERROR] Failed to load slalom detection model")
                return False
            
            # Enable detection
            if not self.toggle_detection(True):
                self.get_logger().error("[ERROR] Failed to enable detection")
                return False

            # Step 2: Set depth to 2.5 feet
            self.get_logger().info(f"[INFO] Setting depth to {self.target_depth_feet} feet...")
            self.set_depth_feet(self.target_depth_feet)
            if not self.wait_for_depth(self.target_depth_meters):
                self.get_logger().error("[ERROR] Failed to reach target depth")
                return False

            # Step 3: Center red pipe in camera
            self.get_logger().info("[INFO] Centering red pipe in camera...")
            if not self.center_red_pipe_in_camera():
                self.get_logger().error("[ERROR] Failed to center red pipe")
                return False

            # Step 4: Drive towards pipe until close
            self.get_logger().info("[INFO] Driving towards red pipe...")
            if not self.drive_towards_pipe_until_close():
                self.get_logger().error("[ERROR] Failed to approach red pipe")
                return False

            # Step 5: Turn 45 degrees left
            self.get_logger().info("[INFO] Turning 45° left...")
            self.turn_degrees(-45)
            self.pause_and_monitor(self.pause_time)

            # Step 6: Drive forward 3 feet
            self.get_logger().info("[INFO] Driving forward 3 feet...")
            self.drive_forward_distance(self.distance_forward_meters)
            self.pause_and_monitor(self.pause_time)

            # Step 7: Rotate right until red pipe is centered
            self.get_logger().info("[INFO] Rotating right to find red pipe...")
            if not self.rotate_until_pipe_centered():
                self.get_logger().error("[ERROR] Failed to find red pipe after rotation")
                return False

            # Step 8: Drive forward 3 feet
            self.get_logger().info("[INFO] Driving forward 3 feet...")
            self.drive_forward_distance(self.distance_forward_meters)
            self.pause_and_monitor(self.pause_time)

            # Step 9: Turn 45 degrees left
            self.get_logger().info("[INFO] Turning 45° left...")
            self.turn_degrees(-45)
            self.pause_and_monitor(self.pause_time)

            # Step 10: Drive forward 3 feet
            self.get_logger().info("[INFO] Driving forward 3 feet...")
            self.drive_forward_distance(self.distance_forward_meters)
            self.pause_and_monitor(self.pause_time)

            # Step 11: Turn 45 degrees right
            self.get_logger().info("[INFO] Turning 45° right...")
            self.turn_degrees(45)
            self.pause_and_monitor(self.pause_time)

            # Mission complete
            self.get_logger().info("[INFO] ✅ Red Pipe Mission completed successfully!")
            final_pos = self.robot_control.get_current_position()
            self.get_logger().info(f"[INFO] Final position: x={final_pos['x']:.2f}m, y={final_pos['y']:.2f}m, depth={final_pos['z']:.2f}m")
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
            self.get_logger().info("[INFO] All movement stopped. Maintaining current depth.")


def main():
    rclpy.init()
    mission = RedPipeMission()
    
    success = False
    try:
        success = mission.run()
        if success:
            mission.get_logger().info("[INFO] 🏁 Red Pipe Mission completed successfully!")
        else:
            mission.get_logger().error("[ERROR] 🚫 Red Pipe Mission failed!")
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