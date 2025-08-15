#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from mavros_msgs.srv import CommandBool


class DisarmerNode(Node):
    def __init__(self):
        super().__init__('disarmer_node')
        self.cli = self.create_client(CommandBool, '/mavros/cmd/arming')

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /mavros/cmd/arming service...')

        self.disarm_vehicle()

    def disarm_vehicle(self):
        req = CommandBool.Request()
        req.value = False  # False = disarm

        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            if future.result().success:
                self.get_logger().info('✅ Vehicle disarmed successfully.')
            else:
                self.get_logger().warn('❌ Failed to disarm vehicle.')
        else:
            self.get_logger().error(f'⚠️ Service call failed: {future.exception()}')


def main(args=None):
    rclpy.init(args=args)
    node = DisarmerNode()
    node.destroy_node()
    


if __name__ == '__main__':
    main()
