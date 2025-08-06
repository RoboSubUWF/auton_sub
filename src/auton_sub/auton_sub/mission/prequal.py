import time
import rclpy
from rclpy.node import Node
import math
from auton_sub.utils import arm, disarm
from auton_sub.utils.guided import set_guided_mode

from auton_sub.motion.robot_control import RobotControl


class StraightLeftMission(Node):
    def __init__(self):
        super().__init__('straight_left_mission')

        self.robot_control = RobotControl()
        self.get_logger().info("[INFO] Straight Left Mission Node Initialized")

        # Mission parameters - FULL SPEED OPERATION
        self.target_depth = 2.0      # meters below surface (positive = down)
        self.forward_distance_1 = 13.0  # meters
        self.pause_time = 2.0         # seconds to pause between steps
        self.forward_speed = 1.0     # Full speed forward command (1.0 = max)
        
        
        # Tolerances
        self.depth_tolerance = 0.25    # 25cm tolerance for depth
        self.distance_tolerance = 0.5  # 50cm tolerance for distance

    def descend_to_depth(self, target_depth=2.0):
        """Descend to the specified depth and maintain it"""
        self.get_logger().info(f"[DEPTH] Descending to {target_depth}m depth...")
        
        # Log current depth before setting target
        current_depth = self.robot_control.get_current_depth()
        self.get_logger().info(f"[DEPTH] Current depth: {current_depth:.2f}m")
        
        # Set target depth
        self.robot_control.set_depth(target_depth)
        self.robot_control.set_max_descent_rate(True)
        
        # Wait and monitor depth changes
        max_wait_time = 30.0  # 30 seconds max for descent
        start_time = time.time()
        
        while (time.time() - start_time) < max_wait_time:
            current = self.robot_control.get_current_depth()
            error = abs(current - target_depth)
            
            if error < self.depth_tolerance:
                self.get_logger().info(f"[DEPTH] ✅ Target depth achieved: {current:.2f}m (target: {target_depth}m)")
                self.robot_control.set_max_descent_rate(False)
                return True
                
            # Log progress every 3 seconds
            elapsed = time.time() - start_time
            if int(elapsed) % 3 == 0 and elapsed > 2.0:
                self.get_logger().info(f"[DEPTH] Descending... Current: {current:.2f}m, Target: {target_depth}m, Error: {error:.2f}m")
            
            time.sleep(0.5)
        
        final_depth = self.robot_control.get_current_depth()
        self.get_logger().warn(f"[DEPTH] ⏰ Descent timeout - Final depth: {final_depth:.2f}m (target: {target_depth}m)")
        self.robot_control.set_max_descent_rate(False)
        return abs(final_depth - target_depth) < (self.depth_tolerance * 2)

    def move_forward_distance(self, distance_meters, description=""):
        """Move forward for a specific distance using DVL-based EKF position feedback"""
        self.get_logger().info(f"[MOTION] {description} - Moving forward {distance_meters}m")
        
        # Record starting position from DVL/EKF
        start_pos = self.robot_control.get_current_position()
        
        # Check if we have valid position data
        if not start_pos.get('valid', False):
            self.get_logger().error("[MOTION] ❌ No valid DVL/EKF position data - cannot perform distance-based movement")
            return False
            
        self.get_logger().info(f"[MOTION] Starting position: x={start_pos['x']:.2f}m, y={start_pos['y']:.2f}m, depth={start_pos['z']:.2f}m")
        start_time = time.time()
        
        # Start moving forward
        self.robot_control.set_movement_command(forward=self.forward_speed, yaw=0.0)
        
        # Monitor distance traveled using DVL-based position
        max_time = (distance_meters / 0.5) + 15.0  # Safety timeout with buffer
        last_log_time = start_time
        
        while (time.time() - start_time) < max_time:
            current_pos = self.robot_control.get_current_position()
            
            # Check position validity
            if not current_pos.get('valid', False):
                self.get_logger().warn("[MOTION] ⚠️  DVL/EKF position lost - continuing with time-based estimate")
                time.sleep(0.5)
                continue
            
            # Calculate actual distance traveled using DVL position
            dx = current_pos['x'] - start_pos['x']
            dy = current_pos['y'] - start_pos['y']
            distance_traveled = (dx**2 + dy**2)**0.5
            
            # Check if target distance reached
            if distance_traveled >= (distance_meters - self.distance_tolerance):
                self.get_logger().info(f"[MOTION] ✅ Target distance reached: {distance_traveled:.2f}m (DVL-based)")
                break
                
            # Log progress every 3 seconds with DVL status
            current_time = time.time()
            if (current_time - last_log_time) >= 3.0:
                depth_error = abs(current_pos['z'] - self.target_depth)
                velocity = self.robot_control.get_current_velocity()
                self.get_logger().info(f"[MOTION] DVL Progress: {distance_traveled:.1f}m/{distance_meters}m, "
                                     f"depth: {current_pos['z']:.2f}m (±{depth_error:.2f}m), "
                                     f"vel: {velocity['x']:.2f},{velocity['y']:.2f}m/s")
                last_log_time = current_time
            
            time.sleep(0.2)
        
        # Stop forward movement
        self.robot_control.set_movement_command(forward=0.0, yaw=0.0)
        
        # Log final results with DVL data
        final_pos = self.robot_control.get_current_position()
        if final_pos.get('valid', False):
            final_distance = ((final_pos['x'] - start_pos['x'])**2 + 
                             (final_pos['y'] - start_pos['y'])**2)**0.5
            self.get_logger().info(f"[MOTION] ✅ {description} complete (DVL-verified) - "
                                  f"Distance: {final_distance:.2f}m, Final pos: x={final_pos['x']:.2f}, y={final_pos['y']:.2f}, depth={final_pos['z']:.2f}m")
        else:
            self.get_logger().warn(f"[MOTION] ⚠️  {description} complete (DVL lost) - Position uncertain")
        
        return True

    def pause_and_monitor_depth(self, pause_duration):
        """Pause while maintaining depth"""
        self.get_logger().info(f"[MOTION] Pausing {pause_duration}s while maintaining depth...")
        
        start_time = time.time()
        while (time.time() - start_time) < pause_duration:
            current_depth = self.robot_control.get_current_depth()
            
            # Check depth drift
            depth_error = abs(current_depth - self.target_depth)
            if depth_error > (self.depth_tolerance * 2):
                self.get_logger().warn(f"[DEPTH] Depth drift: {current_depth:.2f}m "
                                     f"(target: {self.target_depth}m)")
                # Re-set target depth
                self.robot_control.set_depth(self.target_depth)
            
            time.sleep(0.5)

    def run(self):
        self.get_logger().info("[INFO] Starting Straight Left Mission with Heading Control")
        
        try:
            # Step 1: Arm the vehicle
            self.get_logger().info("[INFO] Arming vehicle...")
            arm_node = arm.ArmerNode()
            time.sleep(2.0)
            
            # Step 2: Set GUIDED mode
            self.get_logger().info("[INFO] Setting GUIDED mode...")
            if not set_guided_mode():
                self.get_logger().error("[ERROR] Failed to set GUIDED mode")
                return False
               
            # Step 3: Wait for systems to be ready and DVL data
            self.get_logger().info("[INFO] Waiting for control systems and DVL data...")
            
            max_wait = 10.0
            wait_start = time.time()
            while (time.time() - wait_start) < max_wait:
                pos = self.robot_control.get_current_position()
                if pos.get('valid', False):
                    self.get_logger().info(f"[INFO] ✅ DVL/EKF position valid: x={pos['x']:.2f}, y={pos['y']:.2f}, z={pos['z']:.2f}m")
                    break
                self.get_logger().info("[INFO] Waiting for DVL/EKF position data...")
                self.sleep(1.0)
            else:
                self.get_logger().warn("[WARNING] DVL/EKF position not available - mission may be inaccurate")
            
            time.sleep(2.0)

            # Step 4: Descend to target depth
            if not self.descend_to_depth(self.target_depth):
                self.get_logger().error("[ERROR] Failed to reach target depth")
                return False

            # Step 5: Execute forward movement
            if not self.move_forward_distance(self.forward_distance_1, "Primary forward movement"):
                self.get_logger().error("[ERROR] Failed to complete forward movement")
                return False

            # Step 6: Final pause and depth check
            self.pause_and_monitor_depth(self.pause_time)

            self.get_logger().info("[INFO] ✅ Mission completed successfully!")
            return True
            # Wait for valid position data (DVL + EKF)

        except KeyboardInterrupt:
            self.get_logger().info("[INFO] Mission interrupted")
            return False
        except Exception as e:
            self.get_logger().error(f"[ERROR] Mission failed: {e}")
            return False
        
            
def main():
        rclpy.init()
        mission = StraightLeftMission()
        try:
            success = mission.run()
            if success:
                mission.get_logger().info("[INFO] Mission completed successfully!")
            else:
                mission.get_logger().error("[ERROR] Mission failed!")
        except KeyboardInterrupt:
            mission.get_logger().info("[INFO] Mission interrupted by user")
        finally:
            mission.robot_control.stop()
            mission.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()    