import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import SetBool
from auton_sub.utils import arm, disarm
from auton_sub.utils.guided import set_guided_mode

from auton_sub.motion.robot_control import RobotControl


class CoinTossMission(Node):
    def __init__(self):
        super().__init__('coin_toss_mission')

        self.robot_control = RobotControl()
        self.get_logger().info("[INFO] Coin Toss Mission Node Initialized")

        # Mission parameters - consistent with prequal structure
        self.target_depth = 2.0      # meters below surface (positive = down)
        self.pause_time = 2.0         # seconds to pause between steps
        self.turn_speed = 1.0         # yaw rate for turning (rad/s)
        self.search_timeout = 60.0    # 60 seconds max to find gate
        
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
            self.detected_objects = set(msg.data.split(','))
            
            if 'gate' in self.detected_objects:
                if not self.gate_detected:
                    self.get_logger().info("[VISION] 🎯 GATE DETECTED! Stopping rotation...")
                    self.gate_detected = True
                    # Stop movement but maintain depth
                    self.robot_control.set_movement_command(forward=0.0, yaw=0.0)

    def descend_to_depth(self, target_depth=2.0):
        """Descend to the specified depth and maintain it"""
        self.get_logger().info(f"[DEPTH] Descending to {target_depth}m depth...")
        
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
                self.get_logger().info(f"[DEPTH] Descending... Current: {current:.2f}m, Target: {target_depth}m, Error: {error:.2f}m")
            
            time.sleep(0.5)
        
        final_depth = self.robot_control.get_current_depth()
        self.get_logger().warn(f"[DEPTH] ⏰ Descent timeout - Final depth: {final_depth:.2f}m (target: {target_depth}m)")
        self.robot_control.set_max_descent_rate(False)
        return abs(final_depth - target_depth) < (self.depth_tolerance * 2)

    def turn_right_until_gate_found(self):
        """Turn right until gate is detected using vision system"""
        self.get_logger().info("[MOTION] Starting right turn to search for gate...")
        
        self.gate_detected = False
        start_time = time.time()
        
        # Start turning right while maintaining depth
        self.robot_control.set_movement_command(forward=0.0, yaw=self.turn_speed)
        
        while not self.gate_detected:
            elapsed_time = time.time() - start_time
            
            # Check for timeout
            if elapsed_time > self.search_timeout:
                self.get_logger().warn(f"[MOTION] ⏰ Search timeout ({self.search_timeout}s) - Gate not found!")
                break
            
            # Check current depth and log progress
            current_pos = self.robot_control.get_current_position()
            depth_error = abs(current_pos['z'] - self.target_depth)
            
            # Log progress every 5 seconds
            if int(elapsed_time) % 5 == 0 and elapsed_time > 4.0:
                self.get_logger().info(f"[MOTION] Searching... Time: {elapsed_time:.1f}s, Depth: {current_pos['z']:.2f}m (±{depth_error:.2f}m)")
                
                # Re-correct depth if drifting too much
                if depth_error > (self.depth_tolerance * 2):
                    self.get_logger().warn(f"[DEPTH] Depth drift during turn: {current_pos['z']:.2f}m (target: {self.target_depth}m)")
                    self.robot_control.set_depth(self.target_depth)
            
            # Allow ROS to process detection callbacks
            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(0.1)
        
        # Stop turning but maintain depth
        self.robot_control.set_movement_command(forward=0.0, yaw=0.0)
        
        if self.gate_detected:
            final_pos = self.robot_control.get_current_position()
            self.get_logger().info(f"[MOTION] ✅ Gate found! Turned for {elapsed_time:.1f}s, Final depth: {final_pos['z']:.2f}m")
            return True
        else:
            self.get_logger().error("[MOTION] ❌ Gate not found within timeout period")
            return False

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
        self.get_logger().info("[INFO] Starting Coin Toss Mission with Gate Detection")
        
        try:
            # Step 1: Set up the vision detection model
            self.get_logger().info("[INFO] Setting up vision system...")
            if not self.set_detection_model("coin_detection"):
                self.get_logger().error("[ERROR] Failed to load coin detection model")
                return False
            
            # Step 2: Enable detection
            if not self.toggle_detection(True):
                self.get_logger().error("[ERROR] Failed to enable detection")
                return False
            
            # Step 3: Arm the vehicle
            self.get_logger().info("[INFO] Arming vehicle...")
            arm_node = arm.ArmerNode()
            time.sleep(2.0)
            
            # Step 4: Set GUIDED mode
            self.get_logger().info("[INFO] Setting GUIDED mode...")
            if not set_guided_mode():
                self.get_logger().error("[ERROR] Failed to set GUIDED mode")
                return False
            
            # Step 5: Wait for systems to be ready
            self.get_logger().info("[INFO] Waiting for control systems and vision data...")
            
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
            
            time.sleep(2.0)

            # Step 6: Descend to target depth
            if not self.descend_to_depth(self.target_depth):
                self.get_logger().error("[ERROR] Failed to reach target depth")
                return False

            # Step 7: Brief pause to stabilize at depth
            self.pause_and_monitor_depth(self.pause_time)
            
            # Step 8: Turn right until gate is detected
            if not self.turn_right_until_gate_found():
                self.get_logger().error("[ERROR] Failed to find gate")
                return False

            # Step 9: Final pause and confirmation
            self.pause_and_monitor_depth(self.pause_time)

            self.get_logger().info("[INFO] ✅ Mission completed successfully!")
            final_pos = self.robot_control.get_current_position()
            self.get_logger().info(f"[INFO] Final position: x={final_pos['x']:.2f}m, y={final_pos['y']:.2f}m, depth={final_pos['z']:.2f}m")
            self.get_logger().info("[INFO] 🎯 Submarine is now facing the gate and ready for next mission")
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
    mission = CoinTossMission()
    
    success = False
    try:
        success = mission.run()
        if success:
            mission.get_logger().info("[INFO] 🏁 Mission completed successfully!")
        else:
            mission.get_logger().error("[ERROR] 🚫 Mission failed!")
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