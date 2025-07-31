import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import SetBool
import random

from auton_sub.motion import robot_control
from auton_sub.utils.guided import set_guided_mode


class CoinTossMission(Node):
    """
    Class to run the CoinToss mission using topic-based model control.
    """

    def __init__(self):
        super().__init__('coin_toss_mission')
        self.robot_control = robot_control.RobotControl()
        
        # Create publishers for model control (topic-based approach)
        self.model_command_pub = self.create_publisher(String, '/model_command', 10)
        
        # Create service client for detection toggle
        self.toggle_detection_client = self.create_client(SetBool, '/toggle_detection')
        
        # Wait for services
        self.get_logger().info("🔗 Waiting for detection services...")
        self.toggle_detection_client.wait_for_service(timeout_sec=10.0)
        
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
        self.mission_complete = False
        self.turn_direction = None
        self.model_loaded = False
        
        # Turning parameters
        self.turn_speed = 0.3
        self.search_timeout = 30.0
        
        self.get_logger().info("🪙 Coin Toss mission initialized with topic-based model control")

    def set_detection_model(self, model_name):
        """Set the active detection model using topics"""
        self.get_logger().info(f"🔄 Setting detection model to: {model_name}")
        
        # Publish model command
        msg = String()
        msg.data = model_name
        self.model_command_pub.publish(msg)
        
        # Wait for confirmation (simple timeout approach)
        self.model_loaded = False
        timeout = 10.0
        start_time = time.time()
        
        while not self.model_loaded and (time.time() - start_time) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(0.1)
        
        if self.model_loaded:
            self.get_logger().info(f"✅ Model {model_name} loaded successfully")
            return True
        else:
            self.get_logger().error(f"❌ Failed to load model {model_name}")
            return False

    def model_status_callback(self, msg):
        """Callback for model status updates"""
        if msg.data.startswith("LOADED:"):
            model_name = msg.data.split(":")[1]
            self.get_logger().info(f"📝 Model status update: {model_name} loaded")
            self.model_loaded = True

    def toggle_detection(self, enable):
        """Enable or disable object detection"""
        request = SetBool.Request()
        request.data = enable
        
        future = self.toggle_detection_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        if future.result() is not None:
            response = future.result()
            status = "enabled" if enable else "disabled"
            self.get_logger().info(f"🎯 Detection {status}")
            return response.success
        else:
            self.get_logger().error("❌ Failed to toggle detection")
            return False

    def objects_callback(self, msg):
        """Callback for detected objects"""
        if msg.data:
            self.detected_objects = set(msg.data.split(','))
            
            if 'gate' in self.detected_objects:
                if not self.gate_detected:
                    self.get_logger().info("🎯 GATE DETECTED! Stopping rotation...")
                    self.gate_detected = True
                    self.stop_movement()

    def flip_coin(self):
        """Simulate coin flip to determine turn direction"""
        result = random.choice(['heads', 'tails'])
        
        if result == 'heads':
            self.turn_direction = 'left'
            direction_text = "LEFT (counterclockwise)"
        else:
            self.turn_direction = 'right' 
            direction_text = "RIGHT (clockwise)"
            
        self.get_logger().info(f"🪙 Coin flip result: {result.upper()} - Turning {direction_text}")
        return result

    def descend_to_depth(self, target_depth=0.65):
        """Descend to the specified depth"""
        self.get_logger().info(f"⬇️ Descending to {target_depth}m depth...")
        self.robot_control.set_depth(target_depth)
        time.sleep(3.0)
        self.get_logger().info("✅ Depth achieved")

    def turn_until_gate_found(self):
        """Turn in the determined direction until gate is detected"""
        self.get_logger().info(f"🔄 Starting rotation {self.turn_direction} to search for gate...")
        
        self.gate_detected = False
        start_time = time.time()
        
        self.robot_control.mode = "direct"
        turn_multiplier = -1 if self.turn_direction == 'left' else 1
        
        while not self.gate_detected and not self.mission_complete:
            elapsed_time = time.time() - start_time
            if elapsed_time > self.search_timeout:
                self.get_logger().warn(f"⏰ Search timeout ({self.search_timeout}s) - Gate not found!")
                break
            
            with self.robot_control.lock:
                self.robot_control.direct_input[0] = 0.0
                self.robot_control.direct_input[1] = 0.0
                self.robot_control.direct_input[2] = self.turn_speed * turn_multiplier
            
            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(0.05)
        
        self.stop_movement()
        
        if self.gate_detected:
            self.get_logger().info("🎯 Gate found! Mission phase complete.")
            return True
        else:
            self.get_logger().error("❌ Gate not found within timeout period")
            return False

    def stop_movement(self):
        """Stop all submarine movement"""
        with self.robot_control.lock:
            self.robot_control.direct_input = [0.0] * 6
        self.get_logger().info("🛑 All movement stopped")

    def run(self):
        """Run the complete CoinToss mission"""
        try:
            self.get_logger().info("🚀 Starting Coin Toss mission with gate detection model")
            
            # Step 1: Set up the gate detection model
            if not self.set_detection_model("coin_detection"):
                self.get_logger().error("❌ Failed to load coin detection model")
                return False
            
            # Step 2: Enable detection
            if not self.toggle_detection(True):
                self.get_logger().error("❌ Failed to enable detection")
                return False
            
            # Step 3: Set guided mode
            self.get_logger().info("🎮 Setting GUIDED mode...")
            if not set_guided_mode():
                self.get_logger().error("❌ Failed to set GUIDED mode")
                return False
            
            # Step 4: Flip coin to determine direction
            coin_result = self.flip_coin()
            
            # Step 5: Descend to operating depth
            self.descend_to_depth()
            
            # Step 6: Turn until gate is detected
            success = self.turn_until_gate_found()
            
            if success:
                self.get_logger().info("✅ Coin Toss mission completed successfully!")
                self.get_logger().info("🎯 Submarine is now facing the gate and ready for next mission")
            else:
                self.get_logger().error("❌ Coin Toss mission failed")
            
            return success
            
        except Exception as e:
            self.get_logger().error(f"💥 Mission error: {e}")
            return False

    def cleanup(self):
        """Clean up the mission"""
        self.mission_complete = True
        self.stop_movement()
        
        # Disable detection to save resources
        self.toggle_detection(False)
        
        # Unload model
        msg = String()
        msg.data = ""
        self.model_command_pub.publish(msg)
        
        self.robot_control.mode = "pid"
        self.get_logger().info("🧹 Coin Toss mission cleanup complete")


def main(args=None):
    rclpy.init(args=args)
    mission = CoinTossMission()

    try:
        success = mission.run()
        time.sleep(1.0)
        mission.cleanup()
        
        if success:
            mission.get_logger().info("🏁 Mission completed - ready for next phase")
        else:
            mission.get_logger().error("🚫 Mission failed")
            
    except KeyboardInterrupt:
        mission.get_logger().info("⌨️ Keyboard interrupt - cleaning up...")
        mission.cleanup()
    finally:
        mission.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()