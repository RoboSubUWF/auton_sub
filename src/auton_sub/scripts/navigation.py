import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String

class Navigation(Node):
    def __init__(self):
        super().__init__('navigation')

        # Publisher: Sends thruster commands
        self.thruster_pub = self.create_publisher(Float32MultiArray, '/thruster_cmds', 10)

        # Subscriber: Listens for detected objects
        self.create_subscription(String, '/detected_object', self.object_callback, 10)

        self.get_logger().info("✅ Navigation system started!")

        # Default movement: Move forward
        self.target_thruster_values = [0.5, 0.0, 0.0, 0.0, 0.0, 0.0]  # Forward motion
        self.timer = self.create_timer(2.0, self.move_to_target)

        # Last seen wall side (Left or Right)
        self.last_wall_side = None

    def move_to_target(self):
        """Sends thruster commands to move forward (default behavior)."""
        msg = Float32MultiArray()
        msg.data = self.target_thruster_values
        self.thruster_pub.publish(msg)
        self.get_logger().info(f"🚀 Moving: {self.target_thruster_values}")

    def object_callback(self, msg):
        """Adjusts movement based on detected objects."""
        detected_object = msg.data
        self.get_logger().info(f"🎯 Detected object: {detected_object}")

        # 🚫 Avoid a person
        if detected_object == "person":
            self.get_logger().warn("⚠️ Person detected! Moving away.")
            self.target_thruster_values = [-0.5, 0.0, 0.0, 0.0, 0.0, 0.0]  # Move backward
        
        # 🚧 Avoid a wall
        elif "wall" in detected_object:
            direction = detected_object.split("_")[-1]  # Extract "left" or "right"
            self.last_wall_side = direction
            if direction == "left":
                self.get_logger().warn("⬅️ Wall detected on left! Turning right.")
                self.target_thruster_values = [0.0, 0.0, 0.0, 0.5, 0.0, 0.0]  # Turn right
            elif direction == "right":
                self.get_logger().warn("➡️ Wall detected on right! Turning left.")
                self.target_thruster_values = [0.0, 0.0, 0.0, -0.5, 0.0, 0.0]  # Turn left

        # 🔴 Red PVC Pipe → Turn left, move forward 1 foot, spin 180°
        elif detected_object == "red_pvc_pipe":
            self.get_logger().info("🔴 Red PVC Pipe detected! Performing maneuver.")
            self.execute_maneuver(left_turn=True)

        # ⚪ White PVC Pipe → Turn right, move forward 1 foot, spin 180°
        elif detected_object == "white_pvc_pipe":
            self.get_logger().info("⚪ White PVC Pipe detected! Performing maneuver.")
            self.execute_maneuver(left_turn=False)

        # ➡️ Resume forward movement if no obstacles are detected
        else:
            self.get_logger().info("🔄 No special object detected. Continuing forward.")
            self.target_thruster_values = [0.5, 0.0, 0.0, 0.0, 0.0, 0.0]

    def execute_maneuver(self, left_turn=True):
        """Handles PVC Pipe maneuvers: Turn, move forward, spin 180°."""
        turn_direction = -0.5 if left_turn else 0.5  # Left (-0.5) or Right (0.5)

        # 1️⃣ Turn
        self.target_thruster_values = [0.0, 0.0, 0.0, turn_direction, 0.0, 0.0]
        self.move_to_target()
        self.get_logger().info("↩️ Turning for 2 seconds...")
        self.timer_callback(2)

        # 2️⃣ Move Forward 1 Foot
        self.target_thruster_values = [0.5, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.move_to_target()
        self.get_logger().info("🚀 Moving forward 1 foot...")
        self.timer_callback(2)

        # 3️⃣ Spin 180 Degrees
        self.target_thruster_values = [0.0, 0.0, 0.0, 1.0, 0.0, 0.0]  # Fast yaw rotation
        self.move_to_target()
        self.get_logger().info("🔄 Spinning 180°...")
        self.timer_callback(3)

        # 4️⃣ Resume Forward Movement
        self.target_thruster_values = [0.5, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.get_logger().info("✅ Maneuver complete. Resuming forward movement.")

    def timer_callback(self, duration):
        """Helper function to pause execution for `duration` seconds while maintaining ROS execution."""
        self.get_logger().info(f"⏳ Waiting for {duration} seconds...")
        rclpy.spin_once(self, timeout_sec=duration)

def main(args=None):
    rclpy.init(args=args)
    node = Navigation()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
