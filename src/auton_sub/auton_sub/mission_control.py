import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String, Bool
from sensor_msgs.msg import Image, NavSatFix, FluidPressure
from geometry_msgs.msg import Pose, PoseStamped
import time
import math
import numpy as np
from pymavlink import mavutil

class AutonomousMissionControl(Node):
    def __init__(self):
        super().__init__('autonomous_mission_control')
        
        # Connect to Pixhawk via MAVLink (direct connection for autonomous commands)
        self.master = mavutil.mavlink_connection('/dev/ttyTHS1', baud=57600)
        
        # Wait for a heartbeat to confirm connection
        self.get_logger().info("🔄 Waiting for heartbeat from Pixhawk...")
        self.master.wait_heartbeat()
        self.get_logger().info("✅ Heartbeat received! Connected to Pixhawk.")
        
        # Define mission parameters
        self.mission_active = False
        self.current_mission_step = 0
        self.mission_steps = [
            {"type": "dive", "depth": 1.0},
            {"type": "move", "direction": "forward", "distance": 5.0},
            {"type": "turn", "heading": 90},
            {"type": "move", "direction": "forward", "distance": 3.0},
            {"type": "search", "target": "gate", "timeout": 60},
            {"type": "surface"}
            # Add more mission steps as needed
        ]
        
        # State tracking
        self.detected_objects = []
        self.current_depth = 0.0
        self.current_heading = 0.0
        self.leak_detected = False
        
        # Thruster control publisher (as backup for direct commands)
        self.thruster_pub = self.create_publisher(
            Float32MultiArray,
            '/thruster_cmds',
            10
        )
        
        # Subscribers
        self.leak_sub = self.create_subscription(
            Bool,
            '/leak_detected',
            self.leak_callback,
            10
        )
        
        self.object_sub = self.create_subscription(
            String,
            '/detected_object',
            self.object_detection_callback,
            10
        )
        
        self.pressure_sub = self.create_subscription(
            FluidPressure,
            '/pressure',
            self.pressure_callback,
            10
        )
        
        # Timer for mission execution
        self.mission_timer = self.create_timer(1.0, self.mission_step)
        
        self.get_logger().info("✅ Autonomous Mission Control started!")
        self.get_logger().info("Type 'start' to begin mission, 'stop' to abort")
        
        # Create command line interface for starting/stopping
        self.cmd_timer = self.create_timer(0.1, self.check_cmd_input)
    
    def check_cmd_input(self):
        """Check for command line input to start/stop mission"""
        try:
            # Non-blocking input check
            import sys, select
            if select.select([sys.stdin], [], [], 0)[0]:
                cmd = sys.stdin.readline().strip()
                if cmd.lower() == "start":
                    self.start_mission()
                elif cmd.lower() == "stop":
                    self.stop_mission()
        except:
            pass
    
    def start_mission(self):
        """Start the autonomous mission"""
        # Ensure Pixhawk is in GUIDED mode
        self.set_guided_mode()
        
        # Start the mission
        self.mission_active = True
        self.current_mission_step = 0
        self.get_logger().info("🚀 Mission started!")
    
    def stop_mission(self):
        """Stop the autonomous mission"""
        self.mission_active = False
        # Send stop command to all thrusters
        self.send_thruster_command([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.get_logger().info("🛑 Mission stopped!")
    
    def set_guided_mode(self):
        """Set the vehicle to GUIDED mode for autonomous operation"""
        self.get_logger().info("🛠️ Setting GUIDED mode...")
        
        # ArduSub GUIDED mode is 15
        guided_mode = 15
        
        self.master.mav.set_mode_send(
            self.master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            guided_mode
        )
        
        # Wait for mode change confirmation
        start_time = time.time()
        timeout = 10.0  # seconds
        
        while True:
            msg = self.master.recv_match(type='HEARTBEAT', blocking=True, timeout=1.0)
            if msg:
                if msg.custom_mode == guided_mode:
                    self.get_logger().info("✅ GUIDED mode activated!")
                    break
            
            if time.time() - start_time > timeout:
                self.get_logger().warn("⚠️ Mode change timeout, but continuing operation...")
                break
        
        # Arm the vehicle if not already armed
        self.arm_vehicle()
    
    def arm_vehicle(self):
        """Arm the vehicle for operation"""
        self.get_logger().info("🛠️ Arming vehicle...")
        
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1, 0, 0, 0, 0, 0, 0  # 1 = Arm
        )
        
        # Wait for arming confirmation
        start_time = time.time()
        timeout = 10.0  # seconds
        
        while True:
            msg = self.master.recv_match(type='COMMAND_ACK', blocking=True, timeout=1.0)
            if msg and msg.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
                if msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                    self.get_logger().info("✅ Vehicle armed!")
                    break
                else:
                    self.get_logger().error(f"❌ Arming failed with result: {msg.result}")
                    break
            
            if time.time() - start_time > timeout:
                self.get_logger().warn("⚠️ Arming timeout, but continuing operation...")
                break
    
    def leak_callback(self, msg):
        """Handle leak detection"""
        if msg.data:
            self.leak_detected = True
            self.get_logger().error("🚨 LEAK DETECTED! Aborting mission!")
            # Emergency surface
            self.execute_surface(emergency=True)
            self.stop_mission()
    
    def object_detection_callback(self, msg):
        """Process detected objects"""
        object_name = msg.data
        if object_name not in self.detected_objects:
            self.detected_objects.append(object_name)
            self.get_logger().info(f"🎯 New object detected: {object_name}")
    
    def pressure_callback(self, msg):
        """Calculate depth from pressure"""
        # Simple conversion from pressure to depth
        # Actual calculation depends on your sensor calibration
        pressure_pascal = msg.fluid_pressure
        # Example: depth in meters = (pressure - atmospheric) / (density * gravity)
        # This is a simplification; you'll need to calibrate for your specific sensor
        self.current_depth = (pressure_pascal - 101325) / (1000 * 9.8)
    
    def send_thruster_command(self, values):
        """Send commands to thrusters using the ROS2 topic"""
        if self.leak_detected:
            # Safety feature - no thruster commands if leak detected
            return
            
        msg = Float32MultiArray()
        msg.data = values
        self.thruster_pub.publish(msg)
        self.get_logger().info(f"🚀 Thruster command: {values}")
    
    def mission_step(self):
        """Execute the current mission step"""
        if not self.mission_active:
            return
            
        if self.current_mission_step >= len(self.mission_steps):
            self.get_logger().info("✅ Mission completed!")
            self.stop_mission()
            return
            
        step = self.mission_steps[self.current_mission_step]
        
        if step["type"] == "dive":
            self.execute_dive(step)
        elif step["type"] == "move":
            self.execute_move(step)
        elif step["type"] == "turn":
            self.execute_turn(step)
        elif step["type"] == "search":
            self.execute_search(step)
        elif step["type"] == "surface":
            self.execute_surface()
        # Add more step types as needed
    
    def execute_dive(self, step):
        """Execute a dive to specified depth"""
        target_depth = step.get("depth", 1.0)  # Default 1 meter
        
        self.get_logger().info(f"🔽 Diving to {target_depth} meters")
        
        # Send TARGET_POSITION command for depth control
        self.master.mav.set_position_target_local_ned_send(
            0,  # timestamp
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            0b0000111111111000,  # mask (only use Z position)
            0, 0, target_depth,  # NED position (Z is depth)
            0, 0, 0,  # NED velocity
            0, 0, 0,  # NED acceleration
            0, 0  # yaw and yaw rate
        )
        
        # Wait until we reach target depth (with timeout)
        start_time = time.time()
        timeout = 30.0  # seconds
        depth_threshold = 0.2  # meters
        
        while time.time() - start_time < timeout:
            if abs(self.current_depth - target_depth) < depth_threshold:
                self.get_logger().info(f"✅ Reached target depth: {self.current_depth} meters")
                self.current_mission_step += 1
                return
            time.sleep(1.0)
        
        # If timeout, move to next step anyway
        self.get_logger().warn(f"⚠️ Timeout reaching depth {target_depth}, continuing mission")
        self.current_mission_step += 1
    
    def execute_move(self, step):
        """Execute a movement in a specified direction and distance"""
        direction = step["direction"]
        distance = step.get("distance", 3.0)  # Default 3 meters
        
        # Calculate velocity components based on current heading
        heading_rad = math.radians(self.current_heading)
        vx = 0.0
        vy = 0.0
        
        if direction == "forward":
            vx = math.cos(heading_rad)
            vy = math.sin(heading_rad)
        elif direction == "backward":
            vx = -math.cos(heading_rad)
            vy = -math.sin(heading_rad)
        elif direction == "left":
            vx = -math.sin(heading_rad)
            vy = math.cos(heading_rad)
        elif direction == "right":
            vx = math.sin(heading_rad)
            vy = -math.cos(heading_rad)
        
        speed = 0.5  # m/s
        vx *= speed
        vy *= speed
        
        # Estimated time to travel the distance
        duration = distance / speed
        
        self.get_logger().info(f"➡️ Moving {direction} for {distance} meters ({duration:.1f} seconds)")
        
        # Send velocity command
        self.master.mav.set_position_target_local_ned_send(
            0,  # timestamp
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            0b0000111111000111,  # mask (only use velocity)
            0, 0, 0,  # NED position
            vx, vy, 0,  # NED velocity
            0, 0, 0,  # NED acceleration
            0, 0  # yaw and yaw rate
        )
        
        # Sleep for the estimated duration
        time.sleep(duration)
        
        # Send stop command
        self.master.mav.set_position_target_local_ned_send(
            0,  # timestamp
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            0b0000111111000111,  # mask (only use velocity)
            0, 0, 0,  # NED position
            0, 0, 0,  # NED velocity
            0, 0, 0,  # NED acceleration
            0, 0  # yaw and yaw rate
        )
        
        self.get_logger().info("✅ Movement complete")
        self.current_mission_step += 1
    
    def execute_turn(self, step):
        """Execute a turn to a specific heading"""
        target_heading = step.get("heading", 0)  # Default 0 degrees (North)
        
        self.get_logger().info(f"🔄 Turning to heading {target_heading} degrees")
        
        # Send heading command
        self.master.mav.set_position_target_local_ned_send(
            0,  # timestamp
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            0b0000111111111111,  # mask (only use yaw)
            0, 0, 0,  # NED position
            0, 0, 0,  # NED velocity
            0, 0, 0,  # NED acceleration
            math.radians(target_heading), 0  # yaw and yaw rate
        )
        
        # Wait for turn completion (with timeout)
        start_time = time.time()
        timeout = 15.0  # seconds
        heading_threshold = 5.0  # degrees
        
        # Sleep for a reasonable turn time
        # In a real implementation, you'd use IMU/compass feedback
        turn_time = abs(target_heading - self.current_heading) / 30.0  # Assuming 30 deg/sec
        time.sleep(max(turn_time, 3.0))  # At least 3 seconds
        
        # Update current heading (in a real system, this would come from sensors)
        self.current_heading = target_heading
        
        self.get_logger().info(f"✅ Turn complete, heading: {self.current_heading} degrees")
        self.current_mission_step += 1
    
    def execute_search(self, step):
        """Execute a search pattern looking for a target"""
        target = step["target"]
        timeout = step.get("timeout", 30)  # Default 30 seconds
        
        self.get_logger().info(f"🔍 Searching for {target}")
        
        # Simple search pattern - rotate slowly
        yaw_rate = 0.2  # rad/sec
        
        # Send yaw rate command
        self.master.mav.set_position_target_local_ned_send(
            0,  # timestamp
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            0b0000111111111110,  # mask (only use yaw rate)
            0, 0, 0,  # NED position
            0, 0, 0,  # NED velocity
            0, 0, 0,  # NED acceleration
            0, yaw_rate  # yaw and yaw rate
        )
        
        # Continue until object found or timeout
        start_time = time.time()
        
        while time.time() - start_time < timeout:
            if target in self.detected_objects:
                self.get_logger().info(f"✅ Found {target}!")
                
                # Stop rotation
                self.master.mav.set_position_target_local_ned_send(
                    0,  # timestamp
                    self.master.target_system,
                    self.master.target_component,
                    mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                    0b0000111111111110,  # mask (only use yaw rate)
                    0, 0, 0,  # NED position
                    0, 0, 0,  # NED velocity
                    0, 0, 0,  # NED acceleration
                    0, 0  # yaw and yaw rate
                )
                
                self.current_mission_step += 1
                return
            
            time.sleep(0.1)
        
        # If timeout, stop rotation and move to next step
        self.master.mav.set_position_target_local_ned_send(
            0,  # timestamp
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            0b0000111111111110,  # mask (only use yaw rate)
            0, 0, 0,  # NED position
            0, 0, 0,  # NED velocity
            0, 0, 0,  # NED acceleration
            0, 0  # yaw and yaw rate
        )
        
        self.get_logger().warn(f"⚠️ Timeout searching for {target}")
        self.current_mission_step += 1
    
    def execute_surface(self, emergency=False):
        """Surface the vehicle, optionally as an emergency procedure"""
        if emergency:
            self.get_logger().warn("🚨 EMERGENCY SURFACING!")
            
            # Fast ascent for emergency
            self.master.mav.set_position_target_local_ned_send(
                0,  # timestamp
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                0b0000111111111000,  # mask (only use Z position)
                0, 0, 0,  # Surface (Z=0)
                0, 0, 0,  # NED velocity
                0, 0, 0,  # NED acceleration
                0, 0  # yaw and yaw rate
            )
        else:
            self.get_logger().info("🔼 Surfacing vehicle")
            
            # Normal, controlled ascent
            self.master.mav.set_position_target_local_ned_send(
                0,  # timestamp
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                0b0000111111111000,  # mask (only use Z position)
                0, 0, 0,  # Surface (Z=0)
                0, 0, 0,  # NED velocity
                0, 0, 0,  # NED acceleration
                0, 0  # yaw and yaw rate
            )
        
        # Wait until we reach the surface (with timeout)
        start_time = time.time()
        timeout = 60.0 if not emergency else 30.0  # shorter timeout for emergency
        depth_threshold = 0.3  # meters
        
        while time.time() - start_time < timeout:
            if self.current_depth < depth_threshold:
                self.get_logger().info("✅ Reached the surface!")
                if not emergency:
                    self.current_mission_step += 1
                return
            time.sleep(1.0)
        
        # If timeout, just continue
        self.get_logger().warn("⚠️ Timeout while surfacing")
        if not emergency:
            self.current_mission_step += 1

def main(args=None):
    rclpy.init(args=args)
    node = AutonomousMissionControl()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 Mission Control shutting down...")
    finally:
        node.stop_mission()  # Make sure thrusters stop
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()