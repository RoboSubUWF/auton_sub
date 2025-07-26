# auton_sub/utils/guided.py

import rclpy
from rclpy.node import Node
from mavros_msgs.srv import SetMode
from mavros_msgs.msg import State
import time

class SetGuidedMode(Node):
    def __init__(self):
        super().__init__('set_guided_mode')
        self.cli = self.create_client(SetMode, '/mavros/set_mode')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /mavros/set_mode...')
        self.req = SetMode.Request()
        self.req.custom_mode = "GUIDED"
        
        # Subscribe to state to verify mode change
        self.current_mode = None
        self.state_subscription = self.create_subscription(
            State,
            '/mavros/state',
            self.state_callback,
            10
        )

    def state_callback(self, msg):
        self.current_mode = msg.mode

    def send_request(self):
        # Get current mode first
        self.get_logger().info("🔍 Checking current flight mode...")
        
        # Wait for current state
        timeout = 3.0
        start_time = time.time()
        while self.current_mode is None and (time.time() - start_time) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        
        if self.current_mode is None:
            self.get_logger().error("❌ Could not get current flight mode")
            return False
            
        self.get_logger().info(f"📊 Current mode: {self.current_mode}")
        
        if self.current_mode == "GUIDED":
            self.get_logger().info("✅ Already in GUIDED mode")
            return True
        
        # Send mode change request
        self.get_logger().info("🔄 Requesting GUIDED mode...")
        future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, future)
        
        if not (future.result() and future.result().mode_sent):
            self.get_logger().error("❌ Failed to send mode change request")
            return False
        
        self.get_logger().info("📤 Mode change request sent, waiting for confirmation...")
        
        # Wait for mode to actually change
        timeout = 5.0
        start_time = time.time()
        
        while (time.time() - start_time) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            
            if self.current_mode == "GUIDED":
                self.get_logger().info("✅ Mode successfully changed to GUIDED")
                return True
            elif self.current_mode != self.req.custom_mode:
                self.get_logger().info(f"⏳ Still waiting... Current mode: {self.current_mode}")
        
        # Timeout reached
        self.get_logger().error(f"❌ Mode change failed - still in {self.current_mode}")
        self.get_logger().error("   Possible reasons:")
        self.get_logger().error("   - Flight controller rejected GUIDED mode")
        self.get_logger().error("   - No position reference (DVL/GPS)")
        self.get_logger().error("   - EKF not ready")
        self.get_logger().error("   - Safety checks failed")
        return False

# ✅ Modified function to work within an existing ROS 2 context
def set_guided_mode():
    """
    Set GUIDED mode. Should be called from within an initialized ROS 2 context.
    """
    node = SetGuidedMode()
    success = node.send_request()
    node.destroy_node()
    return success

# ✅ Standalone function for CLI use (initializes its own context)
def set_guided_mode_standalone():
    """
    Set GUIDED mode with its own ROS 2 context initialization.
    Use this for CLI or standalone scripts.
    """
    rclpy.init()
    try:
        success = set_guided_mode()
    finally:
        rclpy.shutdown()
    return success

# Keep this for CLI use
if __name__ == '__main__':
    set_guided_mode_standalone()