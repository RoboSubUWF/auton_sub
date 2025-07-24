# auton_sub/utils/guided.py

import rclpy
from rclpy.node import Node
from mavros_msgs.srv import SetMode

class SetGuidedMode(Node):
    def __init__(self):
        super().__init__('set_guided_mode')
        self.cli = self.create_client(SetMode, '/mavros/set_mode')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /mavros/set_mode...')
        self.req = SetMode.Request()
        self.req.custom_mode = "GUIDED"

    def send_request(self):
        future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() and future.result().mode_sent:
            self.get_logger().info("✅ Mode successfully changed to GUIDED")
            return True
        else:
            self.get_logger().error("❌ Failed to change mode")
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