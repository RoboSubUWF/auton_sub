import time
import rclpy
from rclpy.node import Node

from auton_sub.utils import arm, disarm
from auton_sub.utils.guided import set_guided_mode

from auton_sub.motion.robot_control import RobotControl  # Updated import to match your structure


class StraightLeftMission(Node):
    def __init__(self):
        super().__init__('straight_left_mission')

        self.robot_control = RobotControl()
        self.get_logger().info("[INFO] Straight Left Mission Node Initialized")

        # Mission parameters
        self.target_depth = 2      # meters below surface (positive = down)
        self.forward_distance_1 = 13.0  # meters
        self.forward_distance_2 = 1.0   # meters  
        self.forward_distance_3 = 13.0  # meters
        self.pause_time = 2.0         # seconds to pause between steps
        self.turn_duration = 4.0      # seconds to rotate 90 degrees left
        self.forward_speed = 1     # forward movement speed (m/s in guided mode)
        self.turn_speed = 1         # yaw rate for turning (rad/s)
        
        # Depth tolerance
        self.depth_tolerance = 0.25    # 25cm tolerance for depth

    def descend_to_depth(self, target_depth=2):
        """Descend to the specified depth and maintain it"""
        self.get_logger().info(f"[DEPTH] Descending to {target_depth}m depth...")
        
        # Log current depth before setting target
        current_depth = self.robot_control.get_current_depth()
        self.get_logger().info(f"[DEPTH] Current depth: {current_depth:.2f}m")
        
        # Set target depth using guided mode
        self.robot_control.set_depth(target_depth)
        self.robot_control.set_max_descent_rate(True)
        
        # Wait and monitor depth changes
        max_wait_time = 30.0  # 20 seconds max for descent
        start_time = time.time()
        
        while (time.time() - start_time) < max_wait_time:
            current = self.robot_control.get_current_depth()
            error = abs(current - target_depth)
            
            if error < self.depth_tolerance:
                self.get_logger().info(f"[DEPTH] ✅ Target depth achieved: {current:.2f}m (target: {target_depth}m)")
                self.robot_control.set_max_descent_rate(False) #dont turn off thrusters (keep current depth)
                return True
                
            # Log progress every 2 seconds
            elapsed = time.time() - start_time
            if int(elapsed) % 3 == 0 and elapsed > 2.0:
                self.get_logger().info(f"[DEPTH] Descending... Current: {current:.2f}m, Target: {target_depth}m, Error: {error:.2f}m")
            
            self.sleep(0.5)
        
        final_depth = self.robot_control.get_current_depth()
        self.get_logger().warn(f"[DEPTH] ⏰ Descent timeout - Final depth: {final_depth:.2f}m (target: {target_depth}m)")
        self.robot_control.set_max_descent_rate(False)
        return abs(final_depth - target_depth) < (self.depth_tolerance * 2)  # Accept with larger tolerance

    def move_forward_distance(self, distance_meters, description=""):
        """Move forward for a specific distance using actual DVL position feedback"""
        self.get_logger().info(f"[MOTION] {description} - Moving forward {distance_meters}m")
        
        # Record starting position
        start_pos = self.robot_control.get_current_position()
        start_time = time.time()
        
        # Move forward
        self.robot_control.set_movement_command(forward=self.forward_speed, yaw=0.0)
        
        # Monitor actual distance traveled using DVL
        max_time = distance_meters / abs(self.forward_speed) * 2.0  # Safety timeout (2x expected time)
        
        while (time.time() - start_time) < max_time:
            current_pos = self.robot_control.get_current_position()
            
            # Calculate actual distance traveled using DVL position
            distance_traveled = ((current_pos['x'] - start_pos['x'])**2 + 
                            (current_pos['y'] - start_pos['y'])**2)**0.5
            
            # Check if we've reached the target distance
            if distance_traveled >= distance_meters:
                self.get_logger().info(f"[MOTION] ✅ Target distance reached: {distance_traveled:.2f}m")
                break
                
            # Log progress every 3 seconds
            elapsed = time.time() - start_time
            if int(elapsed) % 3 == 0 and elapsed > 2.0:
                depth_error = abs(current_pos['z'] - self.target_depth)
                self.get_logger().info(f"[MOTION] Progress: {distance_traveled:.1f}m/{distance_meters}m, depth: {current_pos['z']:.2f}m")
            
            self.sleep(0.2)
        
        # Stop movement
        self.robot_control.set_movement_command(forward=0.0, yaw=0.0)
        
        final_pos = self.robot_control.get_current_position()
        final_distance = ((final_pos['x'] - start_pos['x'])**2 + 
                        (final_pos['y'] - start_pos['y'])**2)**0.5
        self.get_logger().info(f"[MOTION] ✅ {description} complete - Distance traveled: {final_distance:.2f}m")

    def turn_left_90_degrees(self, description=""):
        """Turn left 90 degrees"""
        self.get_logger().info(f"[MOTION] {description} - Turning left 90 degrees")
        
        # Record starting orientation
        start_yaw = self.robot_control.orientation['yaw']
        start_time = time.time()
        
        self.robot_control.set_movement_command(forward=0.0, yaw=-self.turn_speed)
        
        while (time.time() - start_time) < self.turn_duration:
            current_pos = self.robot_control.get_current_position()
            elapsed = time.time() - start_time
            
            # Log progress every 2 seconds
            if int(elapsed) % 2 == 0 and elapsed > 1.0:
                yaw_change = abs(self.robot_control.orientation['yaw'] - start_yaw)
                depth_error = abs(current_pos['z'] - self.target_depth)
                self.get_logger().info(f"[MOTION] Turning... Yaw change: {yaw_change:.2f} rad, depth: {current_pos['z']:.2f}m (error: {depth_error:.2f}m)")
            
            self.sleep(0.2)
        
        # Stop turning but keep depth control
        self.robot_control.set_movement_command(forward=0.0, yaw=0.0)
        
        final_yaw = self.robot_control.orientation['yaw']
        yaw_change = abs(final_yaw - start_yaw)
        final_pos = self.robot_control.get_current_position()
        self.get_logger().info(f"[MOTION] ✅ {description} complete - Yaw change: {yaw_change:.2f} rad (~{yaw_change*57.3:.0f} degrees), Final depth: {final_pos['z']:.2f}m")

    def pause_and_monitor_depth(self, pause_duration):
        """Pause while maintaining depth using guided mode"""
        self.get_logger().info(f"[MOTION] Pausing {pause_duration}s while maintaining depth...")
        
        
        start_time = time.time()
        while (time.time() - start_time) < pause_duration:
            current_depth = self.robot_control.get_current_depth()
            
            # Check if we're drifting from target depth
            depth_error = abs(current_depth - self.target_depth)
            if depth_error > (self.depth_tolerance * 2):
                self.get_logger().warn(f"[DEPTH] Depth drift detected: {current_depth:.2f}m (target: {self.target_depth}m)")
                # Re-set target depth
                self.robot_control.set_depth(self.target_depth)
            
            self.sleep(0.5)

    def run(self):
        self.get_logger().info("[INFO] Starting Straight Left Mission with Depth Control")
        
        try:
            # Step 1: Arm the vehicle using imported utility
            self.get_logger().info("[INFO] Arming vehicle...")
            arm_node = arm.ArmerNode()
            # Give some time for arming to complete
            self.sleep(2.0)
            
            # Step 2: Set GUIDED mode using imported utility
            self.get_logger().info("[INFO] Setting GUIDED mode...")
            if not set_guided_mode():
                self.get_logger().error("[ERROR] Failed to set GUIDED mode")
                return False
               
            
            # Step 3: Wait for the robot control system to be ready
            self.get_logger().info("[INFO] Waiting for robot control system...")
            self.sleep(2.0)
            
            # Step 4: Descend to target depth
            if not self.descend_to_depth(self.target_depth):
                self.get_logger().error("[ERROR] Failed to reach target depth")
                return False
            
            self.pause_and_monitor_depth(self.pause_time)
            
            # Step 5: Move forward 13 meters
            self.move_forward_distance(self.forward_distance_1, "First forward segment")
            self.pause_and_monitor_depth(self.pause_time)
            
            # Step 6: Turn left 90 degrees
            self.turn_left_90_degrees("First left turn")
            self.pause_and_monitor_depth(self.pause_time)
            
            # Step 7: Move forward 1 meter
            self.move_forward_distance(self.forward_distance_2, "Short forward segment")
            self.pause_and_monitor_depth(self.pause_time)
            
            # Step 8: Turn left 90 degrees again
            self.turn_left_90_degrees("Second left turn")
            self.pause_and_monitor_depth(self.pause_time)
            
            # Step 9: Move forward 13 meters
            self.move_forward_distance(self.forward_distance_3, "Final forward segment")
            
            # Final stop and maintain position
            self.robot_control.set_movement_command(forward=0.0, yaw=0.0)
            final_pos = self.robot_control.get_current_position()
            self.get_logger().info(f"[INFO] ✅ Mission Complete! Final position: x={final_pos['x']:.2f}m, y={final_pos['y']:.2f}m, depth={final_pos['z']:.2f}m")
            
            return True

        except KeyboardInterrupt:
            self.get_logger().info("[INFO] Mission interrupted")
            return False
        except Exception as e:
            self.get_logger().error(f"[ERROR] Mission failed: {e}")
            return False
        finally:
            # Ensure movement is stopped
            self.robot_control.set_movement_command(forward=0.0, yaw=0.0)
            self.get_logger().info("[INFO] All movement stopped. Depth control still active.")

    def sleep(self, seconds):
        """Sleep while allowing ROS to continue spinning"""
        end_time = time.time() + seconds
        while rclpy.ok() and time.time() < end_time:
            rclpy.spin_once(self, timeout_sec=0.1)

    def cleanup(self):
        """Clean up the mission by stopping movement"""
        self.robot_control.set_movement_command(forward=0.0, yaw=0.0)
        self.get_logger().info("[INFO] Straight Left mission cleanup complete")



def main(args=None):
    rclpy.init(args=args)
    node = StraightLeftMission()
    
    success = False
    try:
        success = node.run()
        
        if success:
            node.get_logger().info("[INFO] 🏁 Mission completed successfully!")
        else:
            node.get_logger().error("[ERROR] 🚫 Mission failed!")
            
    except Exception as e:
        node.get_logger().error(f"[ERROR] Exception occurred: {e}")
    finally:
        # Disarm the vehicle using imported utility
        node.get_logger().info("[INFO] Disarming vehicle...")
        try:
            disarm_node = disarm.DisarmerNode()
            time.sleep(2.0)  # Give time for disarming to complete
            node.get_logger().info("[INFO] ✅ Vehicle disarmed")
        except Exception as e:
            node.get_logger().error(f"[ERROR] Failed to disarm: {e}")
        
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
