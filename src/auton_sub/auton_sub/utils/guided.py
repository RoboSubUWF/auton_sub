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
        self.req.custom_mode = "GUIDED_NOGPS"

    def send_request(self):
        future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, future)
        if future.result().mode_sent:
            self.get_logger().info("Mode successfully changed to GUIDED_NOGPS")
        else:
            self.get_logger().error("Failed to change mode")

def main(args=None):
    rclpy.init(args=args)
    node = SetGuidedMode()
    node.send_request()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
