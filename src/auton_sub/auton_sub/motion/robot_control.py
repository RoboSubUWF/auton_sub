# ROS 2 version of RobotControl with hardcoded configuration for a single submarine

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import OverrideRCIn, ManualControl
from std_srvs.srv import Trigger

from simple_pid import PID
import numpy as np
import math
import time
import threading


def clip(pwm_min: int, pwm_max: int, value: int):
    value = min(pwm_max, value)
    value = max(pwm_min, value)
    return value


class RobotControl(Node):
    def __init__(self):
        super().__init__('robot_control')

        self.rate = self.create_rate(20)  # 20 Hz loop rate
        self.lock = threading.Lock()
        self.debug = False

        # Hardcoded config (since no deviceHelper file)
        self.pwm_scale = 80
        self.pwm_neutral = 1500
        self.pwm_min = 1100
        self.pwm_max = 1900

        # Robot state
        self.position = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        self.orientation = {'yaw': 0.0, 'pitch': 0.0, 'roll': 0.0}
        self.desired_point = {'x': None, 'y': None, 'z': None, 'yaw': None, 'pitch': None, 'roll': None}
        self.direct_input = [0] * 6
        self.mode = "pid"

        # Publishers and subscribers
        self.pose_sub = self.create_subscription(PoseStamped, '/pose', self.pose_callback, 10)
        self.thruster_pub = self.create_publisher(OverrideRCIn, '/mavros/rc/override', 10)

        # PID controllers
        self.PIDs = {
            "yaw": PID(12.0, 0.01, 0.0, setpoint=0, output_limits=(-1, 1)),
            "pitch": PID(0.5, 0.1, 0.1, setpoint=0, output_limits=(-5, 5)),
            "roll": PID(0.5, 0.1, 0.1, setpoint=0, output_limits=(-5, 5)),
            "surge": PID(2.0, 0.05, 0.01, setpoint=0, output_limits=(-2, 2)),
            "lateral": PID(2.0, 0.05, 0.01, setpoint=0, output_limits=(-2, 2)),
            "depth": PID(100.0, 10.0, 0.75, setpoint=0),
        }

        # Start the main control thread
        self.running = True
        self.control_thread = threading.Thread(target=self.control_loop)
        self.control_thread.start()

    def pose_callback(self, msg):
        self.position['x'] = msg.pose.position.x
        self.position['y'] = msg.pose.position.y
        self.position['z'] = msg.pose.position.z
        # Assuming quaternion is [0, 0, sin(yaw/2), cos(yaw/2)] for yaw only (simplified)
        self.orientation['yaw'] = msg.pose.orientation.z  # You should convert properly if using full 3D orientation

    def control_loop(self):
        while rclpy.ok() and self.running:
            if self.mode == "pid":
                self.update_pid_control()
            elif self.mode == "direct":
                self.send_pwm_from_direct_input()
            time.sleep(0.05)

    def update_pid_control(self):
        desired = {
            'x': self.desired_point["x"] if self.desired_point["x"] is not None else self.position['x'],
            'y': self.desired_point["y"] if self.desired_point["y"] is not None else self.position['y'],
            'z': self.desired_point["z"] if self.desired_point["z"] is not None else self.position['z'],
            'yaw': self.desired_point["yaw"] if self.desired_point["yaw"] is not None else self.orientation['yaw'],
        }

        errors = {
            'x': desired['x'] - self.position['x'],
            'y': desired['y'] - self.position['y'],
            'z': desired['z'] - self.position['z'],
            'yaw': desired['yaw'] - self.orientation['yaw'],
        }

        pwm = OverrideRCIn()
        pwm.channels = [self.pwm_neutral] * 18

        pwm.channels[0] = clip(self.pwm_min, self.pwm_max, int(self.PIDs["pitch"](0) * self.pwm_scale + self.pwm_neutral))
        pwm.channels  * self.pwm_scale + self.pwm_neutral
        pwm.channels[2] = clip(self.pwm_min, self.pwm_max, int(self.PIDs["depth"](errors['z']) * self.pwm_scale + self.pwm_neutral))
        pwm.channels[3] = clip(self.pwm_min, self.pwm_max, int(self.PIDs["yaw"](errors['yaw']) * self.pwm_scale + self.pwm_neutral))
        pwm.channels[4] = clip(self.pwm_min, self.pwm_max, int(self.PIDs["surge"](errors['y']) * self.pwm_scale + self.pwm_neutral))
        pwm.channels[5] = clip(self.pwm_min, self.pwm_max, int(self.PIDs["lateral"](errors['x']) * self.pwm_scale + self.pwm_neutral))

        self.thruster_pub.publish(pwm)

    def send_pwm_from_direct_input(self):
        pwm = OverrideRCIn()
        pwm.channels = [self.pwm_neutral] * 18
        with self.lock:
            for i in range(6):
                value = self.direct_input[i]
                pwm.channels[i] = clip(self.pwm_min, self.pwm_max, int(value * self.pwm_scale + self.pwm_neutral))
        self.thruster_pub.publish(pwm)

    def stop(self):
        self.running = False
        self.control_thread.join()
        self.get_logger().info("RobotControl stopped.")


def main(args=None):
    rclpy.init(args=args)
    rc = RobotControl()
    try:
        rclpy.spin(rc)
    except KeyboardInterrupt:
        rc.get_logger().info("Keyboard interrupt - shutting down")
    finally:
        rc.stop()
        rc.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
