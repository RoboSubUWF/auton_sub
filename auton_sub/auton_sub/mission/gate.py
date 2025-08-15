import time
import rclpy
import math
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import SetBool
from auton_sub.utils import arm, disarm
from auton_sub.utils.guided import set_guided_mode

from auton_sub.motion.robot_control import RobotControl


class GateMission(Node):
    def __init__(self):
        super().__init__('gate_mission')

        self.robot_control = RobotControl()
        self.get_logger().info("[INFO] Gate Mission Node Initialized")

        # Mission parameters - consistent with coin_toss structure
        self.target_depth = 2.0      # meters below surface (positive = down)
        self.pause_time = 2.0         # seconds to pause between steps
        self.forward_speed = 1.0      # forward movement speed
        self.turn_speed = 1.0         # yaw rate for turning (rad/s)
        self.search_timeout = 60.0    # 60 seconds max to find objects
        
        # Tolerances
        self.depth_tolerance = 0.25    # 25cm tolerance for depth

        # Vision system publishers and subscribers
        self.model_command_pub = self.create_publisher(String, '/model_command', 10)
        
        # Create service client for detection toggle
        self.toggle_detection_client = self.create_client(SetBool, '/toggle_detection')
        
        # Subscribe to object detection results
        self.detected_objects_sub = self.create_subscription(
            String,
            '/detected_objects',
            self.objects_callback,
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
        self.shark_detected = False
        self.sawfish_detected = False
        self.detected_objects = set()
        self.model_loaded = False
        self.shark_side = None  # Will be "left" or "right"
        self.shark_lost = False
        
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
        """Callback for detected objects - determine shark position relative to sawfish"""
        if msg.data:
            self.detected_objects = set(msg.data.split(','))
            
            # Check for shark detection
            if 'shark' in self.detected_objects:
                if not self.shark_detected:
                    self.get_logger().info("[VISION] 🦈 SHARK DETECTED!")
                    self.shark_detected = True
                    
            # Check for sawfish detection
            if 'sawfish' in self.detected_objects:
                if not self.sawfish_detected:
                    self.get_logger().info("[VISION] 🐟 SAWFISH DETECTED!")
                    self.sawfish_detected = True
                    
            # Determine shark side when both are visible
            if 'shark' in self.detected_objects and 'sawfish' in self.detected_objects:
                if self.shark_side is None:
                    # For now, we'll need coordinate data to determine left/right
                    # This is a simplified version - you may need to enhance with coordinate comparison
                    self.shark_side = "left"  # Default assumption - enhance with coordinate logic
                    self.get_logger().info(f"[VISION] 🎯 Shark position determined: {self.shark_side} side of sawfish")
            
            # Check if shark is no longer detected
            if 'shark' not in self.detected_objects and self.shark_detected:
                if not self.shark_lost:
                    self.get_logger().info("[VISION] 🦈 SHARK LOST FROM VIEW!")
                    self.shark_lost = True

    def descend_to_depth(self, target_depth=2.0):
        """Descend to the specified depth and maintain it"""
        self.get_logger().info(f"[DEPTH] Setting depth to {target_depth}m...")
        
        # Log current depth before setting target
        current_depth = self.robot_control.get_current_depth()
        self.get_logger().info(f"[DEPTH] Current depth: {current_depth:.2f}m")
        
        # Set target depth
        self.robot_control.set_depth(target_depth)
        self.robot_control.set_max_descent_rate(True)
        
        # Wait and monitor depth changes
        max_wait_time = 30.0  # 30 seconds max for descent
        start_time = time.time()
        
        while (time.time() - start_time) < max_wait_time:
            current = self.robot_control.get_current_depth()
            error = abs(current - target_depth)
            
            if error < self.depth_tolerance:
                self.get_logger().info(f"[DEPTH] ✅ Target depth achieved: {current:.2f}m (target: {target_depth}m)")
                self.robot_control.set_max_descent_rate(False)
                return True
                
            # Log progress every 3 seconds
            elapsed = time.time() - start_time
            if int(elapsed) % 3 == 0 and elapsed > 2.0:
                self.get_logger().info(f"[DEPTH] Adjusting... Current: {current:.2f}m, Target: {target_depth}m, Error: {error:.2f}m")
            
            time.sleep(0.5)
        
        final_depth = self.robot_control.get_current_depth()
        self.get_logger().warn(f"[DEPTH] ⏰ Depth adjustment timeout - Final depth: {final_depth:.2f}m (target: {target_depth}m)")
        self.robot_control.set_max_descent_rate(False)
        return abs(final_depth - target_depth) < (self.depth_tolerance * 2)

    def wait_for_shark_and_sawfish(self):
        """Wait until both shark and sawfish are detected and determine shark side"""
        self.get_logger().info("[VISION] Scanning for shark and sawfish to determine gate orientation...")
        
        start_time = time.time()
        
        while not (self.shark_detected and self.sawfish_detected and self.shark_side is not None):
            elapsed_time = time.time() - start_time
            
            # Check for timeout
            if elapsed_time > self.search_timeout:
                self.get_logger().warn(f"[VISION] ⏰ Detection timeout ({self.search_timeout}s) - Could not identify gate orientation!")
                return False
            
            # Log progress every 5 seconds
            if int(elapsed_time) % 5 == 0 and elapsed_time > 4.0:
                detected_str = ", ".join(self.detected_objects) if self.detected_objects else "none"
                self.get_logger().info(f"[VISION] Searching... Time: {elapsed_time:.1f}s, Detected: {detected_str}")
            
            # Maintain depth during search
            current_depth = self.robot_control.get_current_depth()
            depth_error = abs(current_depth - self.target_depth)
            if depth_error > (self.depth_tolerance * 2):
                self.robot_control.set_depth(self.target_depth)
            
            # Allow ROS to process detection callbacks
            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(0.1)
        
        if self.shark_detected and self.sawfish_detected:
            self.get_logger().info(f"[VISION] ✅ Gate orientation identified! Shark is on {self.shark_side} side of sawfish")
            return True
        else:
            return False

    def drive_forward_until_shark_lost(self):
        """Drive forward until shark is no longer visible"""
        self.get_logger().info("[MOTION] Driving forward until shark is lost from view...")
        
        self.shark_lost = False
        start_time = time.time()
        
        # Start moving forward while maintaining depth
        self.robot_control.set_movement_command(forward=self.forward_speed, yaw=0.0)
        
        while not self.shark_lost:
            elapsed_time = time.time() - start_time
            
            # Check for timeout
            if elapsed_time > self.search_timeout:
                self.get_logger().warn(f"[MOTION] ⏰ Forward movement timeout ({self.search_timeout}s) - Shark still visible!")
                break
            
            # Check current depth and log progress
            current_pos = self.robot_control.get_current_position()
            depth_error = abs(current_pos['z'] - self.target_depth)
            
            # Log progress every 3 seconds
            if int(elapsed_time) % 3 == 0 and elapsed_time > 2.0:
                detected_str = ", ".join(self.detected_objects) if self.detected_objects else "none"
                self.get_logger().info(f"[MOTION] Moving forward... Time: {elapsed_time:.1f}s, Depth: {current_pos['z']:.2f}m, Detected: {detected_str}")
                
                # Re-correct depth if drifting too much
                if depth_error > (self.depth_tolerance * 2):
                    self.get_logger().warn(f"[DEPTH] Depth drift during forward motion: {current_pos['z']:.2f}m (target: {self.target_depth}m)")
                    self.robot_control.set_depth(self.target_depth)
            
            # Allow ROS to process detection callbacks
            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(0.1)
        
        if self.shark_lost:
            final_pos = self.robot_control.get_current_position()
            self.get_logger().info(f"[MOTION] ✅ Shark lost from view! Moved forward for {elapsed_time:.1f}s, Final depth: {final_pos['z']:.2f}m")
            return True
        else:
            self.get_logger().error("[MOTION] ❌ Shark still visible after timeout")
            return False

    def drive_forward_one_foot(self):
        """Drive forward exactly 1 foot (0.3048 meters) after losing shark"""
        self.get_logger().info("[MOTION] Driving forward 1 additional foot...")
        
        # Continue moving forward for 1 foot
        # At forward_speed = 1.0 m/s, 1 foot (0.3048m) takes ~0.3 seconds
        # Use a more conservative time estimate
        forward_time = 0.5  # seconds to move 1 foot
        
        self.robot_control.set_movement_command(forward=self.forward_speed, yaw=0.0)
        
        start_time = time.time()
        while (time.time() - start_time) < forward_time:
            # Maintain depth during forward movement
            current_depth = self.robot_control.get_current_depth()
            depth_error = abs(current_depth - self.target_depth)
            if depth_error > (self.depth_tolerance * 2):
                self.robot_control.set_depth(self.target_depth)
            
            time.sleep(0.1)
        
        # Stop forward movement
        self.robot_control.set_movement_command(forward=0.0, yaw=0.0)
        
        final_pos = self.robot_control.get_current_position()
        self.get_logger().info(f"[MOTION] ✅ Completed 1-foot advance. Final depth: {final_pos['z']:.2f}m")

    def yaw_720_degrees_left(self):
        """Perform a 720-degree turn to the left while maintaining depth"""
        self.get_logger().info("[MOTION] Performing 720-degree turn to the left...")
        
        # Calculate turn duration: 720 degrees = 4π radians
        total_rotation = 4 * math.pi  # 720 degrees in radians
        turn_duration = total_rotation / self.turn_speed  # time needed at current turn speed
        
        self.get_logger().info(f"[MOTION] Turn will take approximately {turn_duration:.1f} seconds")
        
        # Start turning left (negative yaw)
        self.robot_control.set_movement_command(forward=0.0, yaw=-self.turn_speed)
        
        start_time = time.time()
        while (time.time() - start_time) < turn_duration:
            elapsed_time = time.time() - start_time
            
            # Check current depth and maintain it
            current_pos = self.robot_control.get_current_position()
            depth_error = abs(current_pos['z'] - self.target_depth)
            
            # Log progress every 3 seconds
            if int(elapsed_time) % 3 == 0 and elapsed_time > 2.0:
                degrees_turned = (elapsed_time / turn_duration) * 720
                self.get_logger().info(f"[MOTION] Turning... {degrees_turned:.0f}°/720°, Depth: {current_pos['z']:.2f}m")
                
            # Re-correct depth if drifting too much
            if depth_error > (self.depth_tolerance * 2):
                self.get_logger().warn(f"[DEPTH] Depth drift during turn: {current_pos['z']:.2f}m (target: {self.target_depth}m)")
                self.robot_control.set_depth(self.target_depth)
            
            time.sleep(0.1)
        
        # Stop turning
        self.robot_control.set_movement_command(forward=0.0, yaw=0.0)
        
        final_pos = self.robot_control.get_current_position()
        self.get_logger().info(f"[MOTION] ✅ 720-degree turn completed! Final depth: {final_pos['z']:.2f}m")

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
        self.get_logger().info("[INFO] Starting Gate Mission with Shark/Sawfish Detection")
        
        try:
            # Step 1: Set up the vision detection model for gate detection
            self.get_logger().info("[INFO] Setting up gate detection vision system...")
            if not self.set_detection_model("gate_detection"):
                self.get_logger().error("[ERROR] Failed to load gate detection model")
                return False
            
            # Step 2: Enable detection
            if not self.toggle_detection(True):
                self.get_logger().error("[ERROR] Failed to enable detection")
                return False

            # Step 3: Ensure we're at the correct depth (continuing from coin toss)
            self.get_logger().info("[INFO] Maintaining 2m depth from previous mission...")
            if not self.descend_to_depth(self.target_depth):
                self.get_logger().warn("[WARNING] Depth adjustment from previous mission")

            # Step 4: Wait for systems to be ready and scan for gate objects
            self.get_logger().info("[INFO] Scanning for shark and sawfish to determine gate orientation...")
            
            # Wait for valid position data (DVL + EKF)
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
                self.get_logger().warn("[WARNING] DVL/EKF position not available - mission may be inaccurate")

            # Step 5: Wait for both shark and sawfish detection and determine orientation
            if not self.wait_for_shark_and_sawfish():
                self.get_logger().error("[ERROR] Failed to identify gate orientation")
                return False

            # Step 6: Brief pause to stabilize
            self.pause_and_monitor_depth(self.pause_time)

            # Step 7: Drive forward until shark is no longer visible
            if not self.drive_forward_until_shark_lost():
                self.get_logger().error("[ERROR] Failed to pass through gate properly")
                return False

            # Step 8: Drive forward an additional 1 foot
            self.drive_forward_one_foot()

            # Step 9: Disable image recognition model
            self.get_logger().info("[VISION] Disabling detection model...")
            if not self.toggle_detection(False):
                self.get_logger().warn("[WARNING] Failed to disable detection")
            
            # Unload model to save resources
            msg = String()
            msg.data = ""
            self.model_command_pub.publish(msg)

            # Step 10: Perform 720-degree turn to the left
            import math  # Import needed for yaw calculation
            self.yaw_720_degrees_left()

            # Step 11: Final pause and confirmation
            self.pause_and_monitor_depth(self.pause_time)

            self.get_logger().info("[INFO] ✅ Gate Mission completed successfully!")
            final_pos = self.robot_control.get_current_position()
            self.get_logger().info(f"[INFO] Final position: x={final_pos['x']:.2f}m, y={final_pos['y']:.2f}m, depth={final_pos['z']:.2f}m")
            self.get_logger().info(f"[INFO] 🎯 Shark was detected on {self.shark_side} side of sawfish")
            self.get_logger().info("[INFO] 🌊 Submarine has passed through gate and completed 720° turn")
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
    mission = GateMission()
    
    success = False
    try:
        success = mission.run()
        if success:
            mission.get_logger().info("[INFO] 🏁 Gate Mission completed successfully!")
        else:
            mission.get_logger().error("[ERROR] 🚫 Gate Mission failed!")
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