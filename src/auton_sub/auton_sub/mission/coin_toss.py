import time 
import rclpy # uses ros2 functions
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import SetBool
from auton_sub.utils import arm #from arm file to arm sub
from auton_sub.utils.guided import set_guided_mode #from guided mode file  to change mode

from auton_sub.motion.robot_control import RobotControl #controls thrusters 


class CoinTossMission(Node): #creates bundle of functions
    def __init__(self): #initiates function
        super().__init__('coin_toss_mission') 

        self.robot_control = RobotControl() #calls robot control for initialization
        self.get_logger().info("[INFO] Coin Toss Mission Node Initialized") #prints

        # Mission parameters - consistent with prequal structure
        self.target_depth = 2.0      # meters below surface (positive = down)
        self.pause_time = 2.0         # seconds to pause between steps
        self.turn_speed = 1.0         # yaw rate for turning (rad/s)
        self.search_timeout = 60.0    # 60 seconds max to find gate
        
        # Tolerances
        self.depth_tolerance = 0.25    # 25cm tolerance for depth

        # Vision system publishers and subscribers
        self.model_command_pub = self.create_publisher(String, '/model_command', 10) #publisher for ros2 topic saves last 10 messages
        
        # Create service client for detection toggle
        self.toggle_detection_client = self.create_client(SetBool, '/toggle_detection') #to change detection model based on mission
        
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
        self.gate_detected = False #sets initial status to be changed later
        self.detected_objects = set() #creates empy set to store detected objects later
        self.model_loaded = False #initializes until changes to true
        
        # Wait for vision services
        self.get_logger().info("[INFO] Waiting for vision detection services...")
        if not self.toggle_detection_client.wait_for_service(timeout_sec=10.0): #if cannot open the camera and image detection in 10 seconds times out
            self.get_logger().warn("[WARNING] Detection service not available - vision may not work")

    def set_detection_model(self, model_name): #sets which modelto use from dual_cam using model name in run
        """Set the active detection model using topics"""
        self.get_logger().info(f"[VISION] Setting detection model to: {model_name}") 
        
        # Publish model command
        msg = String() #turns the message into a string
        msg.data = model_name #turns the model name into a ros2 message to work with the system
        self.model_command_pub.publish(msg) #publish the new message
        
        # Wait for confirmation
        self.model_loaded = False #initializes
        timeout = 10.0 ##times out if no data in 10 seconds
        start_time = time.time() #starts timer
        
        while not self.model_loaded and (time.time() - start_time) < timeout: #if the model isnt loaded and less than the timeout period load the model keep trying
            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(0.1) #tries again every 0.1 seconds
        
        if self.model_loaded: #once loaded
            self.get_logger().info(f"[VISION] ✅ Model {model_name} loaded successfully")
            return True #changes model loading to true to mode one
        else:
            self.get_logger().error(f"[VISION] ❌ Failed to load model {model_name}")
            return False #either time out or dont load

    def model_status_callback(self, msg): #listens for a status message from the vision system.
        """Callback for model status updates"""
        if msg.data.startswith("LOADED:"):
            model_name = msg.data.split(":")[1]
            self.get_logger().info(f"[VISION] Model status update: {model_name} loaded") # If the message says "LOADED:...", it extracts the model’s name, logs it, and sets a flag 
            self.model_loaded = True

    def toggle_detection(self, enable):
        """Enable or disable object detection"""
        if not self.toggle_detection_client.service_is_ready():
            self.get_logger().error("[VISION] Detection service not available")
            return False
            
        request = SetBool.Request() #setBool is a service type that ROS 2 provides for simple on/off or yes/no commands.
        request.data = enable #enables data request
        
        future = self.toggle_detection_client.call_async(request) # just starts the call and immediately returns a Future object. ROS 2 service client for the /toggle_detection service, which uses the SetBool service type.
        #That Future will later hold the result once the service replies.
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0) #“spinning” means letting the node process incoming/outgoing messages and service responses
        
        if future.result() is not None:
            response = future.result() #sets empty list
            status = "enabled" if enable else "disabled"#sets info
            self.get_logger().info(f"[VISION] Detection {status}")
            return response.success #returns enabled or disabled
        else:
            self.get_logger().error("[VISION] Failed to toggle detection")
            return False

    def objects_callback(self, msg):
        """Callback for detected objects"""
        if msg.data:
            self.detected_objects = set(msg.data.split(','))
            
            if 'shark' in self.detected_objects: #chooses shark to identify to stop movement
                if not self.gate_detected:
                    self.get_logger().info("[VISION] 🎯 GATE DETECTED! Stopping rotation...")
                    self.gate_detected = True #sets detection to yes
                    # Stop movement but maintain depth
                    self.robot_control.set_movement_command(forward=0.0, yaw=0.0) #stops movement

    def descend_to_depth(self, target_depth=2.0):#makes sre depth is set correctly
        """Descend to the specified depth and maintain it"""
        self.get_logger().info(f"[DEPTH] Descending to {target_depth}m depth...")
        
        # Log current depth before setting target
        current_depth = self.robot_control.get_current_depth() #gets depth from default setting in qground which i baro
        self.get_logger().info(f"[DEPTH] Current depth: {current_depth:.2f}m")
        
        # Set target depth
        self.robot_control.set_depth(target_depth) #sets depth
        self.robot_control.set_max_descent_rate(True) #sets dull thruster speed
        
        # Wait and monitor depth changes
        max_wait_time = 30.0  # 30 seconds max for descent before tieout
        start_time = time.time() #starts tmer
        
        while (time.time() - start_time) < max_wait_time:
            current = self.robot_control.get_current_depth() #saves depth
            error = abs(current - target_depth) #calcuates error
            
            if error < self.depth_tolerance:
                self.get_logger().info(f"[DEPTH] ✅ Target depth achieved: {current:.2f}m (target: {target_depth}m)")
                self.robot_control.set_max_descent_rate(False) # keeps depth if good 
                return True 
                
            # Log progress every 3 seconds
            elapsed = time.time() - start_time #keeps time
            if int(elapsed) % 3 == 0 and elapsed > 2.0: #if remainder is zero time is up and has saftey of >2 s to ensure doesnt stop too soon
                self.get_logger().info(f"[DEPTH] Descending... Current: {current:.2f}m, Target: {target_depth}m, Error: {error:.2f}m")
            
            time.sleep(0.5) #pause
        
        final_depth = self.robot_control.get_current_depth() #gets current depth to check
        self.get_logger().warn(f"[DEPTH] ⏰ Descent timeout - Final depth: {final_depth:.2f}m (target: {target_depth}m)")
        self.robot_control.set_max_descent_rate(False) #doesnt change deoth
        return abs(final_depth - target_depth) < (self.depth_tolerance * 2) #returns tabsolute value of whether or not it is less than retured

    def turn_right_until_gate_found(self):
        """Turn right until gate is detected using vision system"""
        self.get_logger().info("[MOTION] Starting right turn to search for gate...")
        
        self.gate_detected = False #initialized
        start_time = time.time() #timer
        
        # Start turning right while maintaining depth
        self.robot_control.set_movement_command(forward=0.0, yaw=self.turn_speed) #sets turn speed from above
        
        while not self.gate_detected:
            elapsed_time = time.time() - start_time #timer
            
            # Check for timeout
            if elapsed_time > self.search_timeout: #if outside timelimits set above
                self.get_logger().warn(f"[MOTION] ⏰ Search timeout ({self.search_timeout}s) - Gate not found!")
                break #stops
            
            # Check current depth and log progress
            current_pos = self.robot_control.get_current_position() #gets position again
            depth_error = abs(current_pos['z'] - self.target_depth) #error again
            
            # Log progress every 5 seconds
            if int(elapsed_time) % 5 == 0 and elapsed_time > 4.0: #
                self.get_logger().info(f"[MOTION] Searching... Time: {elapsed_time:.1f}s, Depth: {current_pos['z']:.2f}m (±{depth_error:.2f}m)")
                
                # Re-correct depth if drifting too much
                if depth_error > (self.depth_tolerance * 2):
                    self.get_logger().warn(f"[DEPTH] Depth drift during turn: {current_pos['z']:.2f}m (target: {self.target_depth}m)")
                    self.robot_control.set_depth(self.target_depth) #sets depth if off
            
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
                self.robot_control.set_depth(self.target_depth) #calls set depth from robot_control
            
            time.sleep(0.5)

    def run(self):
        self.get_logger().info("[INFO] Starting Coin Toss Mission with Gate Detection")
        
        try:
            # Step 1: Set up the vision detection model
            self.get_logger().info("[INFO] Setting up vision system...")
            if not self.set_detection_model("coin_detection"): #change to gate_detection later
                self.get_logger().error("[ERROR] Failed to load coin detection model")
                return False
            
            # Step 2: Enable detection
            if not self.toggle_detection(True):
                self.get_logger().error("[ERROR] Failed to enable detection")
                return False
            
            # Step 3: Arm the vehicle
            self.get_logger().info("[INFO] Arming vehicle...")
            arm_node = arm.ArmerNode() #uses arming from utils script
            time.sleep(2.0) #pause to wait
            
            # Step 4: Set GUIDED mode
            self.get_logger().info("[INFO] Setting GUIDED mode...")
            if not set_guided_mode(): #sets mode from other script.
                self.get_logger().error("[ERROR] Failed to set GUIDED mode")
                return False
            
            # Step 5: Wait for systems to be ready
            self.get_logger().info("[INFO] Waiting for control systems and vision data...")
            
            # Wait for valid position data (DVL + EKF)
            max_wait = 10.0 #times out after this time
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
            if not self.descend_to_depth(self.target_depth): #if cant descend
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

        except KeyboardInterrupt: #if ^c is pressed
            self.get_logger().info("[INFO] Mission interrupted")
            return False
        except Exception as e:
            self.get_logger().error(f"[ERROR] Mission failed: {e}")
            return False
        finally:
            # Ensure all movement is stopped but keeps depth hold
            self.robot_control.set_movement_command(forward=0.0, yaw=0.0)
            self.get_logger().info("[INFO] All movement stopped. Depth control still active.")


def main(): #main callable function
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