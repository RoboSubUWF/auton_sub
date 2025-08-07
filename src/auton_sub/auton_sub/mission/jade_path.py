import json
import time

import rclpy
from rclpy.node import Node

from auton_sub.sensors.cv_handler import CVHandler
from auton_sub.motion.robot_control import RobotControl
from auton_sub.utils import arm, disarm


"""
Pseudocode for path functionality

While path is detected:

    align sub to center of path

    move forward slightly

    if path is not detected

        break
"""