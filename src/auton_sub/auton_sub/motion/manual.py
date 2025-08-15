# sets manual mode
#!/usr/bin/env python3

import sys
import rclpy
from rclpy.node import Node
from mavros_msgs.srv import SetMode


class SetModeClient(Node):
    def __init__(self):
        super().__init__('set_mode_client')
        self.cli = self.create_client(SetMode, '/mavros/set_mode')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /mavros/set_mode service...')
        self.req = SetMode.Request()

    def send_request(self, mode):
        self.req.custom_mode = mode
        future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            if future.result().mode_sent:
                self.get_logger().info(f"[SET_MODE] Mode set to '{mode}'")
                return True
            else:
                self.get_logger().warn(f"[SET_MODE] Failed to set mode '{mode}'")
                return False
        else:
            self.get_logger().error(f"[SET_MODE] Service call failed: {future.exception()}")
            return False

def main(args=None):
    rclpy.init(args=args)

    # Get mode from command line or use default
    mode = sys.argv[1] if len(sys.argv) > 1 else "MANUAL"

    set_mode_client = SetModeClient()
    set_mode_client.send_request(mode)

    set_mode_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
