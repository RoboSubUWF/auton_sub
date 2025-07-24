#!/usr/bin/env python3
# pip install keyboard

import rclpy
from rclpy.node import Node
import keyboard
import threading
import time

from auton_sub.motion.robot_control import RobotControl
from auton_sub.motion.servo import Servo
from auton_sub.utils import arm, disarm  # Ensure these are ROS 2 compatible


class KeyboardControlNode(Node):
    def __init__(self):
        super().__init__('keyboard_control_node')
        
        self.rc = RobotControl()
        self.servo = Servo()  # Assume always present

        self.forward = 0
        self.lateral = 0
        self.yaw = 0

        self.manual_control = False
        self.function_control = False
        self.flag = True
        self.data_lock = threading.Lock()

        arm.arm()  # Assume this sends the MAVLink arm command properly in ROS 2

        self.movement_thread = threading.Thread(target=self.send_data_loop)
        self.movement_thread.daemon = True

    def send_data_loop(self):
        while self.flag:
            with self.data_lock:
                self.rc.movement(
                    forward=self.forward,
                    lateral=self.lateral,
                    yaw=self.yaw,
                    pitch=0,
                    roll=0
                )
            time.sleep(0.05)

    def start(self):
        while not self.manual_control and not self.function_control:
            setting = input("Type 'm' for manual control, 'f' for function control: ").strip()
            if setting == 'm':
                self.manual_control = True
            elif setting == 'f':
                self.function_control = True
            else:
                print("[ERROR] Invalid input.")

        self.movement_thread.start()

        try:
            if self.manual_control:
                self.manual_control_loop()
            elif self.function_control:
                self.function_control_loop()
        except KeyboardInterrupt:
            pass
        finally:
            self.flag = False
            self.movement_thread.join()
            with self.data_lock:
                self.rc.movement(forward=0, lateral=0, yaw=0, pitch=0, roll=0)
            disarm.disarm()
            print("[INFO] Node shutdown complete.")

    def manual_control_loop(self):
        print("[INFO] Entering manual control mode (press 'b' to stop)...")
        print("[INFO] Use WASD for motion, QE for yaw.")

        while self.manual_control:
            event = keyboard.read_event(suppress=True)
            if event.event_type == keyboard.KEY_DOWN:
                key = event.name
                with self.data_lock:
                    if key == 'w':
                        self.forward = 1
                    elif key == 's':
                        self.forward = -1
                    elif key == 'a':
                        self.lateral = -1
                    elif key == 'd':
                        self.lateral = 1
                    elif key == 'q':
                        self.yaw = -1
                    elif key == 'e':
                        self.yaw = 1
                    elif key == 'b':
                        self.forward = 0
                        self.lateral = 0
                        self.yaw = 0
                        print("[INFO] Stopping manual control.")
                        self.manual_control = False
                        break
                    else:
                        print(f"[WARN] Unmapped key: {key}")
            elif event.event_type == keyboard.KEY_UP:
                with self.data_lock:
                    self.forward = 0
                    self.lateral = 0
                    self.yaw = 0
                    print("[INFO] Stopped motion.")

    def function_control_loop(self):
        print("[INFO] Entering function control mode. Type commands like: fire 1, drop 2, open, close")
        while self.function_control:
            try:
                cmd = input("> ").strip().lower()
                if cmd.startswith("fire"):
                    _, num = cmd.split()
                    self.servo.torpedo(int(num))
                elif cmd.startswith("drop"):
                    _, num = cmd.split()
                    self.servo.dropper(int(num))
                elif cmd == "open":
                    self.servo.gripper(True)
                elif cmd == "close":
                    self.servo.gripper(False)
                elif cmd == "exit":
                    self.function_control = False
                else:
                    print("[WARN] Unknown command.")
            except KeyboardInterrupt:
                break
        self.function_control = False


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardControlNode()
    node.start()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
