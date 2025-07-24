# need to update once get each component
import time
import serial
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

# Change this to the correct device path for your Pololu serial controller
POLOLU_SERIAL_PORT = "/dev/ttyACM0"

class PololuServo:
    def __init__(self, port=POLOLU_SERIAL_PORT):
        self.USB = serial.Serial(port=port, baudrate=9600, timeout=1)
        if not self.USB.is_open:
            self.USB.open()
        print("[INFO] Pololu serial opened")

    def set_pwm(self, channel: int, target: int):
        target *= 4
        lsb = target & 0x7F
        msb = (target >> 7) & 0x7F
        cmd = bytes([0x84, channel, lsb, msb])
        self.USB.write(cmd)


class GripperService(Node):
    def __init__(self, pololu):
        super().__init__('gripper_service')
        self.pololu = pololu
        self.srv = self.create_service(Trigger, 'gripper_toggle', self.callback)
        self.state = False  # False = closed, True = open

    def callback(self, request, response):
        pwm = 1550 if not self.state else 1450
        print(f"[GRIPPER] Setting to {'open' if not self.state else 'closed'}")
        start_time = time.time()
        while time.time() - start_time < 0.5:
            self.pololu.set_pwm(0, pwm)
            time.sleep(0.05)
        self.pololu.set_pwm(0, 1500)  # Stop gripper
        self.state = not self.state
        response.success = True
        response.message = f"Gripper {'opened' if self.state else 'closed'}"
        return response


class DropperService(Node):
    def __init__(self, pololu):
        super().__init__('dropper_service')
        self.pololu = pololu
        self.srv = self.create_service(Trigger, 'drop_ball', self.callback)
        self.state = 0
        self.positions = [1600, 1200, 700]

    def callback(self, request, response):
        if self.state >= len(self.positions):
            self.state = 0
            msg = "Dropper reset"
        else:
            pwm = self.positions[self.state]
            self.pololu.set_pwm(1, pwm)
            time.sleep(0.5)
            msg = f"Dropped ball #{self.state}"
            self.state += 1
        response.success = True
        response.message = msg
        return response


class TorpedoService(Node):
    def __init__(self, pololu):
        super().__init__('torpedo_service')
        self.pololu = pololu
        self.srv = self.create_service(Trigger, 'fire_torpedo', self.callback)
        self.state = 0
        self.positions = [2400, 1700, 1300]

    def callback(self, request, response):
        if self.state >= len(self.positions):
            self.state = 0
            msg = "Torpedo system reset"
        else:
            pwm = self.positions[self.state]
            self.pololu.set_pwm(2, pwm)
            time.sleep(0.5)
            msg = f"Fired torpedo #{self.state}"
            self.state += 1
        response.success = True
        response.message = msg
        return response


def main(args=None):
    rclpy.init(args=args)

    pololu = PololuServo()

    gripper_node = GripperService(pololu)
    dropper_node = DropperService(pololu)
    torpedo_node = TorpedoService(pololu)

    try:
        rclpy.spin(gripper_node)
        rclpy.spin(dropper_node)
        rclpy.spin(torpedo_node)
    except KeyboardInterrupt:
        print("Shutting down...")

    gripper_node.destroy_node()
    dropper_node.destroy_node()
    torpedo_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
