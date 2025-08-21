#occurs directly after path, the mission should start while depth is still being held and continuing after that
#Has not been tested in water
#needs to be updated to work with new robot control
# -*- coding: utf-8 -*-

import time
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import SetBool
from geometry_msgs.msg import Point
from auton_sub.motion.robot_control import RobotControl


class SlalomMission(Node):
    def __init__(self, side="left"):
        super().__init__('slalom_mission')

        self.robot_control = RobotControl()
        self.get_logger().info("[INFO] Slalom Mission Node Initialized")

        # Get side variable from gate mission (passed as parameter)
        self.side = side  # "left" or "right" from gate.py
        self.get_logger().info(f"[INFO] Slalom side set to: {self.side}")

        # Mission parameters
        self.target_depth_meters = 2.0    # 2 meters below surface
        self.pause_time = 2.0             # seconds to pause between steps
        self.turn_speed = 1.0             # yaw rate for turning (rad/s)
        self.forward_speed = 0.5          # forward speed for movement
        self.search_timeout = 60.0        # 30 seconds max to find pipes
        
        # Distance parameters
        self.distance_1_meter = 1.0       # 1 meter movement
        self.distance_2_meter = 2.0       # 2 meter movement
        
        # Tolerances
        self.depth_tolerance = 0.1        # 10cm tolerance for depth
        self.centering_tolerance = 50     # pixels from center to consider "centered"
        self.bbox_threshold_80_percent = 80  # 80% of camera width for stopping
        self.position_tolerance = 0.1     # 10cm tolerance for position moves

        # Camera parameters (front camera only)
        self.camera_width = 640   # front camera width from dual_camera.py
        self.camera_height = 480  # front camera height from dual_camera.py

        # Vision system publishers and subscribers
        self.model_command_pub = self.create_publisher(String, '/model_command', 10)
        
        # Create service client for detection toggle
        self.toggle_detection_client = self.create_client(SetBool, '/toggle_detection')
        
        # Subscribe to object detection results from dual camera (front camera)
        self.detected_objects_sub = self.create_subscription(
            String,
            '/detected_objects',
            self.objects_callback,
            10
        )
        
        # Subscribe to object coordinates from dual camera
        from interfaces.msg import Objcoords
        self.coords_sub = self.create_subscription(
            Objcoords,
            '/detected_object_coords',
            self.coords_callback,
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
        self.white_pipe_detected = False
        self.red_pipe_centered = False
        self.white_pipe_centered = False
        self.red_pipe_close_enough = False
        self.white_pipe_close_enough = False
        
        # Pipe tracking variables
        self.red_pipe_bbox = {'x1': 0, 'x2': 0, 'center_x': 320, 'width': 0}
        self.white_pipe_bbox = {'x1': 0, 'x2': 0, 'center_x': 320, 'width': 0}
        
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
        """Callback for detected objects from dual camera (front camera only)"""
        if msg.data:
            # Parse detection message - format: "front:object1,object2" or "bottom:object1,object2"
            if "front:" in msg.data:
                # Only process front camera detections
                detected_objects = msg.data.replace("front:", "").split(',')
                detected_objects = set(obj.strip() for obj in detected_objects if obj.strip())
                
                # Check for red pipe
                if 'red_pipe' in detected_objects:
                    if not self.red_pipe_detected:
                        self.get_logger().info("[VISION] 🔴 RED PIPE DETECTED! (Front Camera)")
                    self.red_pipe_detected = True
                else:
                    if self.red_pipe_detected:
                        self.get_logger().warn("[VISION] Red pipe lost from front camera view")
                    self.red_pipe_detected = False
                
                # Check for white pipe
                if 'white_pipe' in detected_objects:
                    if not self.white_pipe_detected:
                        self.get_logger().info("[VISION] ⚪ WHITE PIPE DETECTED! (Front Camera)")
                    self.white_pipe_detected = True
                else:
                    if self.white_pipe_detected:
                        self.get_logger().warn("[VISION] White pipe lost from front camera view")
                    self.white_pipe_detected = False

    def coords_callback(self, msg):
        """Callback for object coordinates from dual camera"""
        # Only process front camera coordinates
        if "_front" in msg.name:
            object_name = msg.name.replace("_front", "")
            
            if object_name == "red_pipe":
                self.red_pipe_bbox['x1'] = msg.x1
                self.red_pipe_bbox['x2'] = msg.x2
                self.red_pipe_bbox['center_x'] = (msg.x1 + msg.x2) / 2
                self.red_pipe_bbox['width'] = msg.x2 - msg.x1
                
                # Calculate if centered and close enough
                center_error = abs(self.red_pipe_bbox['center_x'] - (self.camera_width / 2))
                self.red_pipe_centered = center_error < self.centering_tolerance
                
                # Check if pipe takes up 80% of camera width
                width_percentage = (self.red_pipe_bbox['width'] / self.camera_width) * 100
                self.red_pipe_close_enough = width_percentage >= self.bbox_threshold_80_percent
                
                if self.red_pipe_detected:
                    self.get_logger().info(f"[VISION] Red pipe: center={self.red_pipe_bbox['center_x']:.0f}, "
                                         f"width={width_percentage:.0f}%, centered={self.red_pipe_centered}, "
                                         f"close_enough={self.red_pipe_close_enough}")
            
            elif object_name == "white_pipe":
                self.white_pipe_bbox['x1'] = msg.x1
                self.white_pipe_bbox['x2'] = msg.x2
                self.white_pipe_bbox['center_x'] = (msg.x1 + msg.x2) / 2
                self.white_pipe_bbox['width'] = msg.x2 - msg.x1
                
                # Calculate if centered and close enough
                center_error = abs(self.white_pipe_bbox['center_x'] - (self.camera_width / 2))
                self.white_pipe_centered = center_error < self.centering_tolerance
                
                # Check if pipe takes up 80% of camera width
                width_percentage = (self.white_pipe_bbox['width'] / self.camera_width) * 100
                self.white_pipe_close_enough = width_percentage >= self.bbox_threshold_80_percent
                
                if self.white_pipe_detected:
                    self.get_logger().info(f"[VISION] White pipe: center={self.white_pipe_bbox['center_x']:.0f}, "
                                         f"width={width_percentage:.0f}%, centered={self.white_pipe_centered}, "
                                         f"close_enough={self.white_pipe_close_enough}")

    def set_depth(self, target_depth):
        """Set the target depth"""
        self.robot_control.set_depth(target_depth)
        self.get_logger().info(f"[DEPTH] Target depth set to: {target_depth}m")

    def wait_for_depth(self, target_depth, timeout=30.0):
        """Wait until target depth is reached"""
        self.get_logger().info(f"[DEPTH] Waiting to reach {target_depth}m...")
        
        start_time = time.time()
        while (time.time() - start_time) < timeout:
            current_depth = self.robot_control.get_current_depth()
            error = abs(current_depth - target_depth)
            
            if error < self.depth_tolerance:
                self.get_logger().info(f"[DEPTH] ✅ Target depth reached: {current_depth:.2f}m")
                return True
                
            time.sleep(0.5)
        
        self.get_logger().warn(f"[DEPTH] ⏰ Depth timeout - Current: {self.robot_control.get_current_depth():.2f}m")
        return False

    def center_pipe_in_camera(self, pipe_type="red", timeout=30.0):
        """Turn until specified pipe is centered in camera"""
        self.get_logger().info(f"[MOTION] Centering {pipe_type} pipe in camera...")
        
        start_time = time.time()
        
        while (time.time() - start_time) < timeout:
            if pipe_type == "red":
                if not self.red_pipe_detected:
                    self.get_logger().warn("[VISION] Red pipe not detected, stopping centering")
                    return False
                if self.red_pipe_centered:
                    self.get_logger().info("[VISION] ✅ Red pipe centered!")
                    self.robot_control.set_movement_command(forward=0.0, yaw=0.0)
                    return True
                center_error = self.red_pipe_bbox['center_x'] - (self.camera_width / 2)
            else:  # white
                if not self.white_pipe_detected:
                    self.get_logger().warn("[VISION] White pipe not detected, stopping centering")
                    return False
                if self.white_pipe_centered:
                    self.get_logger().info("[VISION] ✅ White pipe centered!")
                    self.robot_control.set_movement_command(forward=0.0, yaw=0.0)
                    return True
                center_error = self.white_pipe_bbox['center_x'] - (self.camera_width / 2)
            
            # Calculate turning direction and speed
            turn_direction = -1.0 if center_error > 0 else 1.0  # Turn left if pipe is on right
            self.robot_control.set_movement_command(forward=0.0, yaw=turn_direction * self.turn_speed)
            
            # Allow ROS to process callbacks
            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(0.1)
        
        self.get_logger().warn(f"[MOTION] ⏰ {pipe_type} pipe centering timeout")
        self.robot_control.set_movement_command(forward=0.0, yaw=0.0)
        return False

    def drive_towards_pipe_until_80_percent(self, pipe_type="red", timeout=60.0):
        """Drive forward towards pipe until it fills 80% of camera"""
        self.get_logger().info(f"[MOTION] Driving towards {pipe_type} pipe until 80% fill...")
        
        start_time = time.time()
        
        while (time.time() - start_time) < timeout:
            if pipe_type == "red":
                if not self.red_pipe_detected:
                    self.get_logger().warn("[VISION] Red pipe lost, stopping approach")
                    break
                if self.red_pipe_close_enough:
                    self.get_logger().info(f"[VISION] ✅ Red pipe fills 80% of camera!")
                    break
                pipe_centered = self.red_pipe_centered
                center_error = self.red_pipe_bbox['center_x'] - (self.camera_width / 2)
            else:  # white
                if not self.white_pipe_detected:
                    self.get_logger().warn("[VISION] White pipe lost, stopping approach")
                    break
                if self.white_pipe_close_enough:
                    self.get_logger().info(f"[VISION] ✅ White pipe fills 80% of camera!")
                    break
                pipe_centered = self.white_pipe_centered
                center_error = self.white_pipe_bbox['center_x'] - (self.camera_width / 2)
            
            # Continue driving forward while adjusting for centering
            yaw_correction = 0.0
            if not pipe_centered:
                yaw_correction = -0.3 if center_error > 0 else 0.3  # Gentle correction
            
            self.robot_control.set_movement_command(forward=self.forward_speed, yaw=yaw_correction)
            
            # Allow ROS to process callbacks
            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(0.1)
        
        # Stop movement
        self.robot_control.set_movement_command(forward=0.0, yaw=0.0)
        
        if pipe_type == "red" and self.red_pipe_close_enough:
            return True
        elif pipe_type == "white" and self.white_pipe_close_enough:
            return True
        else:
            self.get_logger().warn(f"[MOTION] ❌ Failed to get {pipe_type} pipe to 80% fill")
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
        self.get_logger().info(f"[MOTION] Driving forward {distance_meters}m...")
        
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
                
            time.sleep(0.1)
        
        # Stop movement
        self.robot_control.set_movement_command(forward=0.0, yaw=0.0)
        time.sleep(1.0)  # Brief pause to stabilize

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

    def run_left_side_slalom(self):
        """Run slalom mission for left side configuration"""
        self.get_logger().info("[INFO] Executing LEFT SIDE slalom pattern...")
        
        # Step 1: Center red pipe and drive to 80% fill
        if not self.center_pipe_in_camera("red"):
            return False
        if not self.drive_towards_pipe_until_80_percent("red"):
            return False
        
        # Step 2: Turn 45 degrees left
        self.turn_degrees(-45)
        self.pause_and_monitor(self.pause_time)
        
        # Step 3: Drive forward until white pipe fills 80% of camera
        if not self.drive_towards_pipe_until_80_percent("white"):
            return False
        
        # Step 4: Rotate right 45 degrees
        self.turn_degrees(45)
        self.pause_and_monitor(self.pause_time)
        
        # Step 5: Drive forward 2 meters and stop
        self.drive_forward_distance(self.distance_2_meter)
        
        return True

    def run_right_side_slalom(self):
        """Run slalom mission for right side configuration"""
        self.get_logger().info("[INFO] Executing RIGHT SIDE slalom pattern...")
        
        # Step 1: Center red pipe and drive to 80% fill
        if not self.center_pipe_in_camera("red"):
            return False
        if not self.drive_towards_pipe_until_80_percent("red"):
            return False
        
        # Step 2: Turn 45 degrees right
        self.turn_degrees(45)
        self.pause_and_monitor(self.pause_time)
        
        # Step 3: Drive forward 1 meter
        self.drive_forward_distance(self.distance_1_meter)
        self.pause_and_monitor(self.pause_time)
        
        # Step 4: Turn 45 degrees left
        self.turn_degrees(-45)
        self.pause_and_monitor(self.pause_time)
        
        # Step 5: Drive forward until red pipe fills 80% of camera
        if not self.drive_towards_pipe_until_80_percent("red"):
            return False
        
        # Step 6: Turn right 45 degrees
        self.turn_degrees(45)
        self.pause_and_monitor(self.pause_time)
        
        # Step 7: Drive forward 1 meter and stop
        self.drive_forward_distance(self.distance_1_meter)
        
        return True

    def run(self):
        self.get_logger().info(f"[INFO] Starting Slalom Mission - Side: {self.side}")
        
        try:
            # Step 1: Set up vision system for slalom detection (red and white pipes)
            self.get_logger().info("[INFO] Setting up vision system for slalom detection...")
            if not self.set_detection_model("slalom_detection"):
                self.get_logger().error("[ERROR] Failed to load slalom detection model")
                return False
            
            # Enable detection
            if not self.toggle_detection(True):
                self.get_logger().error("[ERROR] Failed to enable detection")
                return False

            # Step 2: Set depth to 2 meters
            self.get_logger().info("[INFO] Setting depth to 2 meters...")
            self.set_depth(self.target_depth_meters)
            if not self.wait_for_depth(self.target_depth_meters):
                self.get_logger().error("[ERROR] Failed to reach target depth")
                return False

            # Step 3: Execute slalom pattern based on side from gate mission
            success = False
            if self.side == "left":
                success = self.run_left_side_slalom()
            elif self.side == "right":
                success = self.run_right_side_slalom()
            else:
                self.get_logger().error(f"[ERROR] Invalid side parameter: {self.side}")
                return False
            
            if not success:
                self.get_logger().error("[ERROR] Slalom pattern execution failed")
                return False

            # Mission complete
            self.get_logger().info(f"[INFO] ✅ Slalom Mission ({self.side} side) completed successfully!")
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
    
    # Get side parameter from gate mission (this would be passed in real implementation)
    # For testing, default to "left"
    side = "left"  # This should be passed from gate.py somehow
    
    mission = SlalomMission(side=side)
    
    success = False
    try:
        success = mission.run()
        if success:
            mission.get_logger().info("[INFO] 🏁 Slalom Mission completed successfully!")
        else:
            mission.get_logger().error("[ERROR] 🚫 Slalom Mission failed!")
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
