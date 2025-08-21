#for the claw and octogon combined challenge
#needs to be updated now that robot control has been
#Has not been tested on the sub
import time
import rclpy
import math
from rclpy.node import Node
from std_msgs.msg import String, Bool
from std_srvs.srv import SetBool
from auton_sub.utils import arm, disarm
from auton_sub.utils.guided import set_guided_mode
from interfaces.msg import Objcoords

from auton_sub.motion.robot_control import RobotControl
from auton_sub.claw.claw_control import open_claw, close_claw, get_claw_state


class OceanCleanupMission(Node):
    def __init__(self):
        super().__init__('ocean_cleanup_mission')

        self.robot_control = RobotControl()
        self.get_logger().info("[INFO] Ocean Cleanup Mission Node Initialized")

        # Mission parameters
        self.target_depth = 2.0      # meters below surface (positive = down)
        self.surface_depth = 0.5     # surface depth for identification
        self.pause_time = 2.0        # seconds to pause between steps
        self.forward_speed = 0.8     # forward movement speed (slower for precision)
        self.turn_speed = 0.8        # yaw rate for turning (rad/s)
        self.search_timeout = 60.0   # 60 seconds max to find objects
        
        # Tolerances
        self.depth_tolerance = 0.25    # 25cm tolerance for depth
        self.position_tolerance = 20   # pixels for centering
        
        # Vision system publishers and subscribers
        self.model_command_pub = self.create_publisher(String, '/model_command', 10)
        
        # Create service client for detection toggle
        self.toggle_detection_client = self.create_client(SetBool, '/toggle_detection')
        
        # Subscribe to object detection results (works for both cameras)
        self.detected_objects_sub = self.create_subscription(
            String,
            '/detected_objects',
            self.objects_callback,
            10
        )
        
        # Subscribe to object coordinates (for positioning)
        self.object_coords_sub = self.create_subscription(
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
        self.detected_objects = set()
        self.model_loaded = False
        self.object_coordinates = {}
        self.items_collected = 0
        self.total_items_detected = 0
        
        # Detection flags
        self.table_detected = False
        self.shark_detected = False
        self.item_detected = False
        self.basket_detected = False
        self.red_basket_detected = False
        self.blue_basket_detected = False
        
        # Position tracking
        self.table_centered = False
        self.item_position = None
        self.basket_position = None
        
        # Mission progress
        self.initial_position = None
        self.items_in_bins = {'red': 0, 'blue': 0}
        
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
        """Callback for detected objects from both cameras"""
        if msg.data:
            # Parse camera-specific detections
            if ":" in msg.data:
                camera_name, objects_str = msg.data.split(":", 1)
                self.detected_objects = set(objects_str.split(','))
                self.get_logger().debug(f"[VISION] {camera_name} detected: {self.detected_objects}")
            else:
                self.detected_objects = set(msg.data.split(','))
            
            # Update detection flags
            self.table_detected = 'table' in self.detected_objects
            self.shark_detected = 'shark' in self.detected_objects
            self.item_detected = any(item in self.detected_objects for item in ['bottle', 'can', 'debris', 'trash'])
            self.red_basket_detected = 'red_basket' in self.detected_objects
            self.blue_basket_detected = 'blue_basket' in self.detected_objects
            self.basket_detected = self.red_basket_detected or self.blue_basket_detected

    def coords_callback(self, msg):
        """Callback for object coordinates"""
        self.object_coordinates[msg.name] = {'x1': msg.x1, 'x2': msg.x2}
        
        # Update position variables based on object type
        object_name = msg.name.replace("_front", "").replace("_bottom", "")
        
        if object_name == 'table':
            center_x = (msg.x1 + msg.x2) / 2
            # Assuming image width of 640 pixels
            self.table_centered = abs(center_x - 320) < self.position_tolerance
            
        elif object_name in ['bottle', 'can', 'debris', 'trash']:
            center_x = (msg.x1 + msg.x2) / 2
            self.item_position = (center_x, 240)  # Assuming center y
            
        elif 'basket' in object_name:
            center_x = (msg.x1 + msg.x2) / 2
            self.basket_position = (center_x, 240)

    def descend_to_depth(self, target_depth):
        """Descend to the specified depth and maintain it"""
        self.get_logger().info(f"[DEPTH] Setting depth to {target_depth}m...")
        
        current_depth = self.robot_control.get_current_depth()
        self.get_logger().info(f"[DEPTH] Current depth: {current_depth:.2f}m")
        
        self.robot_control.set_depth(target_depth)
        self.robot_control.set_max_descent_rate(True)
        
        max_wait_time = 30.0
        start_time = time.time()
        
        while (time.time() - start_time) < max_wait_time:
            current = self.robot_control.get_current_depth()
            error = abs(current - target_depth)
            
            if error < self.depth_tolerance:
                self.get_logger().info(f"[DEPTH] ✅ Target depth achieved: {current:.2f}m")
                self.robot_control.set_max_descent_rate(False)
                return True
                
            if int(time.time() - start_time) % 3 == 0:
                self.get_logger().info(f"[DEPTH] Adjusting... Current: {current:.2f}m, Target: {target_depth}m")
            
            time.sleep(0.5)
        
        self.robot_control.set_max_descent_rate(False)
        return False

    def turn_right_90_degrees(self):
        """Turn right exactly 90 degrees"""
        self.get_logger().info("[MOTION] Turning right 90 degrees...")
        
        # Calculate turn duration: 90 degrees = π/2 radians
        turn_angle = math.pi / 2
        turn_duration = turn_angle / self.turn_speed
        
        # Start turning right (positive yaw)
        self.robot_control.set_movement_command(forward=0.0, yaw=self.turn_speed)
        
        start_time = time.time()
        while (time.time() - start_time) < turn_duration:
            # Maintain depth during turn
            current_depth = self.robot_control.get_current_depth()
            if abs(current_depth - self.target_depth) > self.depth_tolerance:
                self.robot_control.set_depth(self.target_depth)
            time.sleep(0.1)
        
        # Stop turning
        self.robot_control.set_movement_command(forward=0.0, yaw=0.0)
        self.get_logger().info("[MOTION] ✅ 90-degree right turn completed")

    def drive_forward_distance(self, target_distance_feet):
        """Drive forward a specific distance in feet"""
        target_distance_m = target_distance_feet * 0.3048  # Convert feet to meters
        self.get_logger().info(f"[MOTION] Driving forward {target_distance_feet} feet ({target_distance_m:.2f}m)...")
        
        # At forward_speed, calculate approximate time
        drive_time = target_distance_m / self.forward_speed
        
        self.robot_control.set_movement_command(forward=self.forward_speed, yaw=0.0)
        
        start_time = time.time()
        while (time.time() - start_time) < drive_time:
            current_depth = self.robot_control.get_current_depth()
            if abs(current_depth - self.target_depth) > self.depth_tolerance:
                self.robot_control.set_depth(self.target_depth)
            time.sleep(0.1)
        
        self.robot_control.set_movement_command(forward=0.0, yaw=0.0)
        self.get_logger().info(f"[MOTION] ✅ Forward movement of {target_distance_feet} feet completed")

    def rotate_until_object_found(self, target_object, direction="left"):
        """Rotate until specific object is detected"""
        self.get_logger().info(f"[MOTION] Rotating {direction} to find {target_object}...")
        
        # Reset detection flag
        if target_object == "table":
            self.table_detected = False
        elif target_object == "shark":
            self.shark_detected = False
        elif target_object == "basket":
            self.basket_detected = False
        
        yaw_speed = self.turn_speed if direction == "left" else -self.turn_speed
        start_time = time.time()
        
        self.robot_control.set_movement_command(forward=0.0, yaw=yaw_speed)
        
        while True:
            elapsed_time = time.time() - start_time
            
            # Check for timeout
            if elapsed_time > self.search_timeout:
                self.get_logger().warn(f"[MOTION] ⏰ Search timeout - {target_object} not found!")
                break
            
            # Check if object is detected
            object_found = False
            if target_object == "table" and self.table_detected:
                object_found = True
            elif target_object == "shark" and self.shark_detected:
                object_found = True
            elif target_object == "basket" and self.basket_detected:
                object_found = True
            
            if object_found:
                self.get_logger().info(f"[MOTION] ✅ {target_object} found! Stopping rotation...")
                break
            
            # Maintain depth and process callbacks
            current_depth = self.robot_control.get_current_depth()
            if abs(current_depth - self.target_depth) > self.depth_tolerance:
                self.robot_control.set_depth(self.target_depth)
            
            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(0.1)
        
        # Stop rotation
        self.robot_control.set_movement_command(forward=0.0, yaw=0.0)
        return self.table_detected or self.shark_detected or self.basket_detected

    def center_over_table(self):
        """Center over table using bottom camera"""
        self.get_logger().info("[POSITIONING] Centering over table using bottom camera...")
        
        max_attempts = 30
        attempts = 0
        
        while attempts < max_attempts and not self.table_centered:
            if 'table_bottom' in self.object_coordinates or 'table' in self.object_coordinates:
                # Try bottom camera first, then fallback to general detection
                coords_key = 'table_bottom' if 'table_bottom' in self.object_coordinates else 'table'
                coords = self.object_coordinates[coords_key]
                center_x = (coords['x1'] + coords['x2']) / 2
                error_x = center_x - 320  # Image center (assuming 640px width)
                
                # Simple proportional control for centering
                if abs(error_x) > self.position_tolerance:
                    # Convert pixel error to movement command
                    lateral_speed = max(-0.3, min(0.3, error_x * 0.001))  # Proportional lateral movement
                    self.robot_control.set_movement_command(forward=0.0, lateral=lateral_speed, yaw=0.0)
                    time.sleep(0.5)  # Brief lateral movement
                    self.robot_control.set_movement_command(forward=0.0, lateral=0.0, yaw=0.0)
                else:
                    self.table_centered = True
            
            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(0.2)
            attempts += 1
        
        if self.table_centered:
            self.get_logger().info("[POSITIONING] ✅ Successfully centered over table")
        else:
            self.get_logger().warn("[POSITIONING] ⚠️ Could not achieve perfect centering")

    def surface_and_identify(self):
        """Surface and wait for object identification"""
        self.get_logger().info("[DEPTH] Surfacing for identification...")
        
        if self.descend_to_depth(self.surface_depth):
            self.get_logger().info("[IDENTIFICATION] Waiting 3 seconds for identification...")
            time.sleep(3.0)
            # Process any pending callbacks during identification
            for i in range(10):
                rclpy.spin_once(self, timeout_sec=0.1)
                time.sleep(0.2)
            return True
        return False

    def collect_item_sequence(self):
        """Complete item collection sequence - SIMPLIFIED VERSION"""
        self.get_logger().info("[COLLECTION] Starting item collection sequence...")
        
        # 1. OPEN CLAW - Simple function call
        self.get_logger().info("[COLLECTION] Opening claw...")
        if not open_claw():
            self.get_logger().error("[COLLECTION] Failed to open claw")
            return False
        
        # 2. Submerge to working depth
        if not self.descend_to_depth(self.target_depth):
            self.get_logger().error("[COLLECTION] Failed to reach working depth")
            return False
        
        # 3. Check if item is still detected
        if not self.item_detected:
            self.get_logger().warn("[COLLECTION] No item detected for collection")
            return False
        
        # 4. Position for item pickup
        self.get_logger().info("[COLLECTION] Positioning for item pickup...")
        self.robot_control.set_movement_command(forward=-self.forward_speed * 0.5, yaw=0.0)
        time.sleep(2.0)
        self.robot_control.set_movement_command(forward=0.0, yaw=0.0)
        
        # 5. Approach item slowly
        self.get_logger().info("[COLLECTION] Approaching item...")
        approach_time = 3.0
        start_time = time.time()
        
        while (time.time() - start_time) < approach_time:
            self.robot_control.set_movement_command(forward=self.forward_speed * 0.2, yaw=0.0)
            rclpy.spin_once(self, timeout_sec=0.1)
            if not self.item_detected:
                self.get_logger().warn("[COLLECTION] Lost sight of item during approach")
                break
            time.sleep(0.1)
        
        self.robot_control.set_movement_command(forward=0.0, yaw=0.0)
        
        # 6. CLOSE CLAW - Simple function call
        self.get_logger().info("[COLLECTION] Closing claw to grab item...")
        if not close_claw():
            self.get_logger().error("[COLLECTION] Failed to close claw")
            return False
        
        # Wait for claw to close
        time.sleep(2.0)
        
        # 7. Lift item
        self.get_logger().info("[COLLECTION] Lifting item...")
        current_depth = self.robot_control.get_current_depth()
        lift_depth = current_depth - 0.6096  # 2 feet up
        if not self.descend_to_depth(max(0.1, lift_depth)):
            self.get_logger().warn("[COLLECTION] Could not lift item properly")
        
        # 8. Return to table center
        self.get_logger().info("[COLLECTION] Returning to table center...")
        self.center_over_table()
        
        self.robot_control.set_movement_command(forward=0.0, yaw=0.0)
        
        self.items_collected += 1
        self.get_logger().info(f"[COLLECTION] ✅ Item {self.items_collected} collected successfully")
        return True

    def deposit_item_sequence(self, basket_color):
        """Complete item deposit sequence - SIMPLIFIED VERSION"""
        self.get_logger().info(f"[DEPOSIT] Starting deposit sequence for {basket_color} basket...")
        
        # 1. Surface to find basket
        if not self.surface_and_identify():
            self.get_logger().error("[DEPOSIT] Failed to surface for basket identification")
            return False
        
        # 2. Find basket
        self.get_logger().info(f"[DEPOSIT] Searching for {basket_color} basket...")
        if not self.rotate_until_object_found("basket"):
            self.get_logger().error(f"[DEPOSIT] Could not find any basket")
            return False
        
        # 3. Approach basket
        self.get_logger().info("[DEPOSIT] Approaching basket...")
        approach_duration = 3.0
        start_time = time.time()
        
        while (time.time() - start_time) < approach_duration:
            self.robot_control.set_movement_command(forward=self.forward_speed * 0.4, yaw=0.0)
            rclpy.spin_once(self, timeout_sec=0.1)
            if not self.basket_detected:
                self.get_logger().warn("[DEPOSIT] Lost sight of basket during approach")
                break
            time.sleep(0.1)
        
        self.robot_control.set_movement_command(forward=0.0, yaw=0.0)
        
        # 4. OPEN CLAW - Simple function call to drop item
        self.get_logger().info("[DEPOSIT] Opening claw to drop item...")
        if not open_claw():
            self.get_logger().error("[DEPOSIT] Failed to open claw for drop")
            return False
        
        # 5. Move up to ensure item drops
        self.get_logger().info("[DEPOSIT] Moving up to ensure clean drop...")
        current_depth = self.robot_control.get_current_depth()
        drop_depth = current_depth - 0.3048  # 1 foot up
        self.descend_to_depth(max(0.1, drop_depth))
        
        # Wait for item to drop
        time.sleep(2.0)
        
        # 6. Count item in bin
        self.items_in_bins[basket_color] += 1
        
        self.robot_control.set_movement_command(forward=0.0, yaw=0.0)
        self.get_logger().info(f"[DEPOSIT] ✅ Item deposited in {basket_color} basket (Total: {self.items_in_bins[basket_color]})")
        return True

    def return_to_table(self):
        """Return to table position"""
        self.get_logger().info("[NAVIGATION] Returning to table...")
        
        # Move back from basket area
        self.robot_control.set_movement_command(forward=-self.forward_speed * 0.5, yaw=0.0)
        time.sleep(3.0)
        self.robot_control.set_movement_command(forward=0.0, yaw=0.0)
        
        # Descend back to working depth
        self.descend_to_depth(self.target_depth)
        
        # Search for table again if needed
        if not self.table_detected:
            self.get_logger().info("[NAVIGATION] Re-locating table...")
            self.rotate_until_object_found("table", "left")
        
        # Re-center over table
        self.center_over_table()

    def final_celebration(self):
        """Perform final celebration based on items collected"""
        total_items_in_bins = sum(self.items_in_bins.values())
        
        self.get_logger().info(f"[CELEBRATION] Items in bins - Red: {self.items_in_bins['red']}, Blue: {self.items_in_bins['blue']}")
        self.get_logger().info(f"[CELEBRATION] Total items between bins: {total_items_in_bins}")
        
        # OPEN CLAW for celebration - Simple function call
        open_claw()
        
        # Perform 360-degree yaw for each item collected
        for i in range(total_items_in_bins):
            self.get_logger().info(f"[CELEBRATION] Celebration spin {i+1}/{total_items_in_bins}")
            
            # 360-degree turn = 2π radians
            turn_duration = (2 * math.pi) / self.turn_speed
            
            self.robot_control.set_movement_command(forward=0.0, yaw=self.turn_speed)
            
            start_time = time.time()
            while (time.time() - start_time) < turn_duration:
                # Maintain depth during celebration
                current_depth = self.robot_control.get_current_depth()
                if abs(current_depth - self.target_depth) > self.depth_tolerance:
                    self.robot_control.set_depth(self.target_depth)
                time.sleep(0.1)
            
            self.robot_control.set_movement_command(forward=0.0, yaw=0.0)
            time.sleep(1.0)  # Pause between spins
        
        self.get_logger().info("[CELEBRATION] ✅ Ocean cleanup celebration completed!")

    def pause_and_monitor_depth(self, pause_duration):
        """Pause while maintaining depth"""
        self.get_logger().info(f"[MOTION] Pausing {pause_duration}s while maintaining depth...")
        
        start_time = time.time()
        while (time.time() - start_time) < pause_duration:
            current_depth = self.robot_control.get_current_depth()
            depth_error = abs(current_depth - self.target_depth)
            if depth_error > (self.depth_tolerance * 2):
                self.robot_control.set_depth(self.target_depth)
            
            # Process callbacks during pause
            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(0.5)

    def run(self):
        self.get_logger().info("[INFO] Starting Ocean Cleanup Mission")
        
        try:
            # Step 1: Set up vision
            self.get_logger().info("[INFO] Setting up object detection vision system...")
            if not self.set_detection_model("general_objects"):
                self.get_logger().error("[ERROR] Failed to load general objects detection model")
                return False
            
            # Step 2: Enable detection
            if not self.toggle_detection(True):
                self.get_logger().error("[ERROR] Failed to enable detection")
                return False

            # Step 3: Initialize claw - SIMPLE FUNCTION CALL
            self.get_logger().info("[INFO] Initializing claw to open position...")
            open_claw()  # Just call the function!

            # Step 4: Set working depth
            self.get_logger().info("[INFO] Setting working depth...")
            if not self.descend_to_depth(self.target_depth):
                self.get_logger().error("[ERROR] Failed to reach working depth")
                return False

            # Step 5: Save initial position
            self.initial_position = self.robot_control.get_current_position()
            self.get_logger().info(f"[INFO] Initial position saved: {self.initial_position}")

            # Step 6: Turn right 90 degrees
            self.turn_right_90_degrees()
            self.pause_and_monitor_depth(self.pause_time)

            # Step 7: Drive forward 4 feet
            self.drive_forward_distance(4.0)
            self.pause_and_monitor_depth(self.pause_time)

            # Step 8: Rotate left until table is seen
            if not self.rotate_until_object_found("table", "left"):
                self.get_logger().error("[ERROR] Could not find table")
                return False

            # Step 9: Drive forward toward table
            self.robot_control.set_movement_command(forward=self.forward_speed * 0.5, yaw=0.0)
            time.sleep(3.0)
            self.robot_control.set_movement_command(forward=0.0, yaw=0.0)

            # Step 10: Center over table
            self.center_over_table()

            # Step 11: Surface and look for shark
            if not self.surface_and_identify():
                self.get_logger().error("[ERROR] Failed to surface")
                return False

            # Step 12: Rotate until shark is found
            if not self.rotate_until_object_found("shark", "right"):
                self.get_logger().warn("[WARNING] Shark not found, continuing mission")

            # Step 13: Stop for 5 seconds when shark is found
            if self.shark_detected:
                self.get_logger().info("[MISSION] Shark detected! Stopping for 5 seconds...")
                self.robot_control.set_movement_command(forward=0.0, yaw=0.0)
                time.sleep(5.0)

            # Main cleanup loop
            max_cleanup_cycles = 10  # Prevent infinite loops
            cleanup_cycle = 0
            
            while cleanup_cycle < max_cleanup_cycles:
                cleanup_cycle += 1
                self.get_logger().info(f"[CLEANUP] Starting cleanup cycle {cleanup_cycle}")

                # Return to working depth for item identification
                if not self.descend_to_depth(self.target_depth):
                    break

                # Center over table
                self.center_over_table()

                # Surface to identify items
                if not self.surface_and_identify():
                    break
                
                # Check for items
                if not self.item_detected:
                    self.get_logger().info("[CLEANUP] No more items detected on table")
                    break

                # Collect item - uses simple open_claw() and close_claw() functions
                if not self.collect_item_sequence():
                    self.get_logger().error("[ERROR] Failed to collect item")
                    break

                # Determine basket color (alternate between red and blue)
                basket_color = "red" if (self.items_collected % 2) == 1 else "blue"
                
                # Deposit item - uses simple open_claw() function  
                if not self.deposit_item_sequence(basket_color):
                    self.get_logger().error("[ERROR] Failed to deposit item")
                    break

                # Return to table for next item
                self.return_to_table()
                
                # Brief pause before next cycle
                self.pause_and_monitor_depth(self.pause_time)

            # Final celebration
            self.final_celebration()

            # Mission complete
            self.get_logger().info("[INFO] ✅ Ocean Cleanup Mission completed successfully!")
            final_pos = self.robot_control.get_current_position()
            self.get_logger().info(f"[INFO] Final position: x={final_pos['x']:.2f}m, y={final_pos['y']:.2f}m, depth={final_pos['z']:.2f}m")
            self.get_logger().info(f"[INFO] 🧹 Total items collected: {self.items_collected}")
            self.get_logger().info(f"[INFO] 🗂️ Items in bins - Red: {self.items_in_bins['red']}, Blue: {self.items_in_bins['blue']}")
            return True

        except KeyboardInterrupt:
            self.get_logger().info("[INFO] Mission interrupted")
            return False
        except Exception as e:
            self.get_logger().error(f"[ERROR] Mission failed: {e}")
            return False
        finally:
            # Ensure all movement is stopped and claw is safe
            self.robot_control.set_movement_command(forward=0.0, yaw=0.0)
            # EMERGENCY OPEN CLAW for safety - Simple function call
            self.get_logger().info("[SAFETY] Opening claw for safety...")
            open_claw()
            self.get_logger().info("[INFO] All movement stopped. Claw opened for safety. Mission cleanup complete.")


def main():
    rclpy.init()
    mission = OceanCleanupMission()
    
    success = False
    try:
        success = mission.run()
        if success:
            mission.get_logger().info("[INFO] 🏁 Ocean Cleanup Mission completed successfully!")
        else:
            mission.get_logger().error("[ERROR] 🚫 Ocean Cleanup Mission failed!")
    except KeyboardInterrupt:
        mission.get_logger().info("[INFO] Mission interrupted by user")
    except Exception as e:
        mission.get_logger().error(f"[ERROR] Exception occurred: {e}")
    finally:
        # Final cleanup
        try:
            mission.toggle_detection(False)
            # Unload model to save resources
            msg = String()
            msg.data = ""
            mission.model_command_pub.publish(msg)
            # Final safety claw open
            from auton_sub.claw.claw_control import open_claw
            open_claw()
        except:
            pass
            
        mission.robot_control.stop()
        mission.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':

    main()
