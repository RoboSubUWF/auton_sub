import time
import rclpy
from rclpy.node import Node

from auton_sub.motion import robot_control  # For controlling the thrusters of the sub


class CoinTossMission(Node):
    """
    Class to run the CoinToss mission.
    """

    def __init__(self):
        super().__init__('coin_toss_mission')
        self.robot_control = robot_control.RobotControl()
        self.get_logger().info("Coin Toss mission initialized")

    def run(self, heading):
        """
        Run the CoinToss mission.

        Args:
            heading (float): The desired heading (in degrees).
        """
        self.get_logger().info("Starting Coin Toss mission")

        # Descend to 0.65m
        self.robot_control.set_depth(0.65)
        time.sleep(1)

        # Turn to desired heading
        self.robot_control.setHeadingOld(heading)
        time.sleep(1)

        self.get_logger().info(f"Turned to heading {heading}")

    def cleanup(self):
        """
        Clean up the mission by stopping movement.
        """
        self.robot_control.movement()
        self.get_logger().info("Coin Toss mission terminated")


def main(args=None):
    rclpy.init(args=args)
    mission = CoinTossMission()

    try:
        mission.run(heading=218)
        time.sleep(2)
        mission.cleanup()
    finally:
        mission.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
