import time
import rclpy
import math
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import SetBool
from auton_sub.utils import arm, disarm
from auton_sub.utils.guided import set_guided_mode

from auton_sub.motion.robot_control import RobotControl


class HomeMission(Node):
    def __init__(self):
        super().__init__('home_mission')

        self.robot_control = RobotControl()
        self.get_logger().info("[INFO] Home Mission Node Initialized")

        # Mission parameters - consistent with gate mission structure
        self.target_depth = 2.0       # meters below surface (positive = down)
        self.surface_depth = 0.5      # target depth for surfacing (slightly below surface)
        self.pause_time = 2.0         # seconds to pause between steps
        self.forward_speed = 1.0      # forward movement speed
        self.turn_speed = 1.0         # yaw rate for turning (rad/s)
        self.search_timeout = 60.0    # 60 seconds max to find gate
        self.drive_timeout = 30.0     # 30 seconds max to drive to gate
        
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
        self.gate_detected = False
        self.detected_objects = set()
        self.model_loaded = False
        self.gate_lost = False
        
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
        """Callback for detected objects - looking for gate structure from behind"""
        if msg.data:
            # Parse camera-specific detections
            self.detected_objects = set()
            
            # Handle camera-specific format: "front:object1,object2" or "bottom:object3,object4"
            for camera_data in msg.data.split(';'):
                if ':' in camera_data:
                    camera_name, objects_str = camera_data.split(':', 1)
                    objects = objects_str.split(',') if objects_str else []
                else:
                    # Fallback for non-camera-specific format
                    objects = msg.data.split(',')
                
                for obj in objects:
                    self.detected_objects.add(obj.strip())
            
            # Check for gate detection - looking for the gate structure itself from behind
            gate_objects = ['gate']  # Looking for the physical gate structure
            current_gate_detected = any(obj in self.detected_objects for obj in gate_objects)
            
            if current_gate_detected and not self.gate_detected:
                self.get_logger().info("[VISION] 🚪 GATE STRUCTURE DETECTED (from behind)!")
                self.gate_detected = True
                
            # Check if gate is no longer detected
            if not current_gate_detected and self.gate_detected:
                if not self.gate_lost:
                    self.get_logger().info("[VISION] 🚪 GATE STRUCTURE LOST FROM VIEW!")
                    self.gate_lost = True

    def maintain_depth(self, target_depth):
        """Maintain the specified depth"""
        current_depth = self.robot_control.get_current_depth()
        depth_error = abs(current_depth - target_depth)
        
        if depth_error > (self.depth_tolerance * 2):
            self.get_logger().warn(f"[DEPTH] Depth drift: {current_depth:.2f}m (target: {target_depth}m)")
            self.robot_control.set_depth(target_depth)

    def rotate_left_until_gate_found(self):
        """Rotate left until gate structure is detected from behind"""
        self.get_logger().info("[MOTION] Rotating left to search for gate structure...")
        
        self.gate_detected = False
        start_time = time.time()
        
        # Start rotating left (negative yaw)
        self.robot_control.set_movement_command(forward=0.0, yaw=-self.turn_speed)
        
        while not self.gate_detected:
            elapsed_time = time.time() - start_time
            
            # Check for timeout
            if elapsed_time > self.search_timeout:
                self.get_logger().warn(f"[MOTION] ⏰ Gate search timeout ({self.search_timeout}s) - No gate found!")
                break
            
            # Maintain depth during rotation
            self.maintain_depth(self.target_depth)
            
            # Log progress every 5 seconds
            if int(elapsed_time) % 5 == 0 and elapsed_time > 4.0:
                degrees_turned = (elapsed_time * self.turn_speed) * (180 / math.pi)
                detected_str = ", ".join(self.detected_objects) if self.detected_objects else "none"
                self.get_logger().info(f"[MOTION] Searching for gate structure... {degrees_turned:.0f}° turned, Time: {elapsed_time:.1f}s, Detected: {detected_str}")
            
            # Allow ROS to process detection callbacks
            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(0.1)
        
        # Stop rotation
        self.robot_control.set_movement_command(forward=0.0, yaw=0.0)
        
        if self.gate_detected:
            elapsed_time = time.time() - start_time
            degrees_turned = (elapsed_time * self.turn_speed) * (180 / math.pi)
            self.get_logger().info(f"[MOTION] ✅ Gate structure found after {degrees_turned:.0f}° rotation ({elapsed_time:.1f}s)!")
            return True
        else:
            self.get_logger().error("[MOTION] ❌ Gate structure not found after full search")
            return False

    def drive_toward_gate_until_lost(self):
        """Drive forward toward the gate structure until it's lost from view (passed through from behind)"""
        self.get_logger().info("[MOTION] Driving toward gate structure until passed through...")
        
        self.gate_lost = False
        start_time = time.time()
        
        # Start moving forward while maintaining depth
        self.robot_control.set_movement_command(forward=self.forward_speed, yaw=0.0)
        
        while not self.gate_lost:
            elapsed_time = time.time() - start_time
            
            # Check for timeout
            if elapsed_time > self.drive_timeout:
                self.get_logger().warn(f"[MOTION] ⏰ Drive timeout ({self.drive_timeout}s) - Gate still visible!")
                break
            
            # Maintain depth during forward movement
            self.maintain_depth(self.target_depth)
            
            # Check current position and log progress
            current_pos = self.robot_control.get_current_position()
            
            # Log progress every 3 seconds
            if int(elapsed_time) % 3 == 0 and elapsed_time > 2.0:
                detected_str = ", ".join(self.detected_objects) if self.detected_objects else "none"
                self.get_logger().info(f"[MOTION] Approaching gate structure... Time: {elapsed_time:.1f}s, Depth: {current_pos['z']:.2f}m, Detected: {detected_str}")
            
            # Allow ROS to process detection callbacks
            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(0.1)
        
        # Stop forward movement
        self.robot_control.set_movement_command(forward=0.0, yaw=0.0)
        
        if self.gate_lost:
            elapsed_time = time.time() - start_time
            final_pos = self.robot_control.get_current_position()
            self.get_logger().info(f"[MOTION] ✅ Gate structure passed through from behind! Drove for {elapsed_time:.1f}s, Final depth: {final_pos['z']:.2f}m")
            return True
        else:
            self.get_logger().error("[MOTION] ❌ Gate structure still visible after timeout")
            return False

    def drive_forward_one_meter(self):
        """Drive forward exactly 1 meter after passing through gate"""
        self.get_logger().info("[MOTION] Driving forward 1 additional meter...")
        
        # At forward_speed = 1.0 m/s, 1 meter takes 1 second
        # Use a conservative time estimate with some buffer
        forward_time = 1.2  # seconds to move 1 meter with buffer
        
        self.robot_control.set_movement_command(forward=self.forward_speed, yaw=0.0)
        
        start_time = time.time()
        while (time.time() - start_time) < forward_time:
            # Maintain depth during forward movement
            self.maintain_depth(self.target_depth)
            time.sleep(0.1)
        
        # Stop forward movement
        self.robot_control.set_movement_command(forward=0.0, yaw=0.0)
        
        final_pos = self.robot_control.get_current_position()
        self.get_logger().info(f"[MOTION] ✅ Completed 1-meter advance. Final depth: {final_pos['z']:.2f}m")

    def surface_submarine(self):
        """Surface the submarine to near-surface depth"""
        self.get_logger().info(f"[DEPTH] Surfacing to {self.surface_depth}m depth...")
        
        # Log current depth before surfacing
        current_depth = self.robot_control.get_current_depth()
        self.get_logger().info(f"[DEPTH] Current depth: {current_depth:.2f}m")
        
        # Set target depth to surface level
        self.robot_control.set_depth(self.surface_depth)
        
        # Wait and monitor surfacing
        max_wait_time = 30.0  # 30 seconds max for surfacing
        start_time = time.time()
        
        while (time.time() - start_time) < max_wait_time:
            current = self.robot_control.get_current_depth()
            error = abs(current - self.surface_depth)
            
            if error < self.depth_tolerance:
                self.get_logger().info(f"[DEPTH] ✅ Surfaced! Current depth: {current:.2f}m (target: {self.surface_depth}m)")
                return True
                
            # Log progress every 3 seconds
            elapsed = time.time() - start_time
            if int(elapsed) % 3 == 0 and elapsed > 2.0:
                self.get_logger().info(f"[DEPTH] Surfacing... Current: {current:.2f}m, Target: {self.surface_depth}m, Error: {error:.2f}m")
            
            time.sleep(0.5)
        
        final_depth = self.robot_control.get_current_depth()
        self.get_logger().warn(f"[DEPTH] ⏰ Surfacing timeout - Final depth: {final_depth:.2f}m (target: {self.surface_depth}m)")
        return abs(final_depth - self.surface_depth) < (self.depth_tolerance * 2)

    def stop_all_motors(self):
        """Stop all motors and movement"""
        self.get_logger().info("[MOTION] Stopping all motors...")
        
        # Stop all movement commands
        self.robot_control.set_movement_command(forward=0.0, yaw=0.0)
        
        # Clear all target positions and depths
        with self.robot_control.lock:
            self.robot_control.desired_point = {'x': None, 'y': None, 'z': None, 'yaw': None}
            self.robot_control.movement_command = {'forward': 0.0, 'yaw': 0.0}
        
        # Send explicit stop commands
        self.robot_control.stop()
        
        self.get_logger().info("[MOTION] ✅ All motors stopped")

    def pause_and_monitor_depth(self, pause_duration):
        """Pause while maintaining depth"""
        self.get_logger().info(f"[MOTION] Pausing {pause_duration}s while maintaining depth...")
        
        start_time = time.time()
        while (time.time() - start_time) < pause_duration:
            self.maintain_depth(self.target_depth)
            time.sleep(0.5)

    def run(self):
        self.get_logger().info("[INFO] Starting Home Mission - Return Through Gate and Surface")
        
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

            # Step 3: Maintain current depth (2m from previous mission)
            self.get_logger().info("[INFO] Maintaining 2m depth from previous mission...")
            current_depth = self.robot_control.get_current_depth()
            if abs(current_depth - self.target_depth) > self.depth_tolerance:
                self.robot_control.set_depth(self.target_depth)
                self.pause_and_monitor_depth(3.0)  # Allow time for depth adjustment

            # Step 4: Wait for systems to be ready
            self.get_logger().info("[INFO] Preparing for gate structure search (from behind)...")
            
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

            # Step 5: Rotate left until gate structure is found
            if not self.rotate_left_until_gate_found():
                self.get_logger().error("[ERROR] Failed to locate gate structure")
                return False

            # Step 6: Brief pause to stabilize after finding gate
            self.pause_and_monitor_depth(self.pause_time)

            # Step 7: Drive toward gate structure until it's lost from view (passed through from behind)
            if not self.drive_toward_gate_until_lost():
                self.get_logger().error("[ERROR] Failed to pass through gate structure")
                return False

            # Step 8: Drive forward an additional 1 meter
            self.drive_forward_one_meter()

            # Step 9: Disable image recognition model to save resources
            self.get_logger().info("[VISION] Disabling detection model...")
            if not self.toggle_detection(False):
                self.get_logger().warn("[WARNING] Failed to disable detection")
            
            # Unload model to save resources
            msg = String()
            msg.data = ""
            self.model_command_pub.publish(msg)

            # Step 10: Surface the submarine
            if not self.surface_submarine():
                self.get_logger().warn("[WARNING] Surfacing may not have completed fully")

            # Step 11: Stop all motors
            self.stop_all_motors()

            # Step 12: Final status report
            final_pos = self.robot_control.get_current_position()
            self.get_logger().info("[INFO] ✅ Home Mission completed successfully!")
            self.get_logger().info(f"[INFO] Final position: x={final_pos['x']:.2f}m, y={final_pos['y']:.2f}m, depth={final_pos['z']:.2f}m")
            self.get_logger().info("[INFO] 🏠 Submarine has returned through gate from behind and surfaced")
            self.get_logger().info("[INFO] 🛑 All motors stopped - Mission complete!")
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
            self.get_logger().info("[INFO] All movement stopped.")


def main():
    rclpy.init()
    mission = HomeMission()
    
    success = False
    try:
        success = mission.run()
        if success:
            mission.get_logger().info("[INFO] 🏁 Home Mission completed successfully!")
        else:
            mission.get_logger().error("[ERROR] 🚫 Home Mission failed!")
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
            mission.stop_all_motors()
        except:
            pass
            
        mission.robot_control.stop()
        mission.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()