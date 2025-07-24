import rclpy
from rclpy.node import Node
from mavros_msgs.srv import CommandBool

class ArmerNode(Node):
    def __init__(self):
        super().__init__('armer_node')
        self.cli = self.create_client(CommandBool, '/mavros/cmd/arming')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for arming service...')
        self.send_arm_request()

    def send_arm_request(self):
        req = CommandBool.Request()
        req.value = True
        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None and future.result().success:
            self.get_logger().info('✅ Vehicle armed successfully.')
        else:
            self.get_logger().error('❌ Failed to arm vehicle.')

def main(args=None):
    rclpy.init(args=args)
    node = ArmerNode()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()