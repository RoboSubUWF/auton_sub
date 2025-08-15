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
        self.get_logger().info("[INFO] Straight Left Mission Node Initialized (MAVROS Vision Topics Mode)")

        # Mission parameters - FULL SPEED OPERATION
        self.target_depth = -0.5      # meters below surface (positive = down)
        self.forward_distance_1 = 13.0  # meters
        self.pause_time = 2.0         # seconds to pause between steps
        self.forward_speed = 1.0     # Full speed forward command (1.0 = max)
        
        # Tolerances - adjusted for MAVROS vision topics operation
        self.depth_tolerance = 0.3    # 30cm tolerance for depth (MAVROS vision pose)
        self.distance_tolerance = 0.5  # 50cm tolerance for distance

    def descend_to_depth(self, target_depth=-0.5):
        """Descend to the specified depth using MAVROS vision pose data"""
        self.get_logger().info(f"[DEPTH] Descending to {target_depth}m depth using MAVROS vision pose data...")
        
        # Log current depth before setting target
        current_depth = self.robot_control.get_current_depth()
        self.get_logger().info(f"[DEPTH] Current depth: {current_depth:.2f}m (MAVROS vision pose)")
        
        # Set target depth
        self.robot_control.set_depth(target_depth)
        self.robot_control.set_max_descent_rate(True)
        
        # Wait and monitor depth changes using MAVROS vision pose data
        max_wait_time = 30.0  # 30 seconds max for descent
        start_time = time.time()
        
        while (time.time() - start_time) < max_wait_time: 
            current = self.robot_control.get_current_depth()  # MAVROS vision pose z
            error = abs(current - target_depth)
            
            if error < self.depth_tolerance:
                self.get_logger().info(f"[DEPTH] ✅ Target depth achieved: {current:.2f}m (target: {target_depth}m) [MAVROS-VISION]")
                self.robot_control.set_max_descent_rate(False)
                return True
                
            # Log progress every 3 seconds
            elapsed = time.time() - start_time
            if int(elapsed) % 3 == 0 and elapsed > 2.0: 
                vision_pos = self.robot_control.get_current_position()
                vision_vel = self.robot_control.get_current_velocity()
                status = "VISION_OK" if vision_pos['valid'] else "VISION_STALE"
                vel_status = "SPEED_OK" if vision_vel.get('valid', False) else "SPEED_STALE"
                self.get_logger().info(f"[DEPTH] Descending... Current: {current:.2f}m, Target: {target_depth}m, "
                                     f"Error: {error:.2f}m, Z-vel: {vision_vel['z']:.3f}m/s ({status}, {vel_status})")
            
            time.sleep(0.5)
        
        final_depth = self.robot_control.get_current_depth()
        self.get_logger().warn(f"[DEPTH] ⏰ Descent timeout - Final depth: {final_depth:.2f}m (target: {target_depth}m) [MAVROS-VISION]")
        self.robot_control.set_max_descent_rate(False)
        return abs(final_depth - target_depth) < (self.depth_tolerance * 2)

    def move_forward_distance(self, distance_meters, description=""):
        """Move forward for a specific distance using MAVROS vision pose feedback"""
        self.get_logger().info(f"[MOTION] {description} - Moving forward {distance_meters}m (MAVROS Vision Pose)")
        
        # Record starting position from MAVROS vision pose
        start_pos = self.robot_control.get_current_position()
        
        # Check if we have valid MAVROS vision data
        if not start_pos.get('valid', False):
            self.get_logger().error("[MOTION] ❌ No valid MAVROS vision pose data - cannot perform distance-based movement")
            # Try to continue with time-based estimate as fallback
            return self.move_forward_time_based(distance_meters, description)
            
        self.get_logger().info(f"[MOTION] Starting position (MAVROS-VISION): x={start_pos['x']:.2f}m, y={start_pos['y']:.2f}m, depth={start_pos['z']:.2f}m")
        start_time = time.time()
        
        # Start moving forward
        self.robot_control.set_movement_command(forward=self.forward_speed, yaw=0.0)
        
        # Monitor distance traveled using MAVROS vision pose
        max_time = (distance_meters / 0.5) + 15.0  # Safety timeout with buffer
        last_log_time = start_time 
        
        while (time.time() - start_time) < max_time:
            current_pos = self.robot_control.get_current_position()
            
            # Check MAVROS vision validity
            if not current_pos.get('valid', False):
                self.get_logger().warn("[MOTION] ⚠️ MAVROS vision pose lost - continuing with timeout fallback")
                time.sleep(0.5)
                continue
            
            # Calculate actual distance traveled using MAVROS vision pose
            dx = current_pos['x'] - start_pos['x']
            dy = current_pos['y'] - start_pos['y']
            distance_traveled = (dx**2 + dy**2)**0.5
            
            # Check if target distance reached
            if distance_traveled >= (distance_meters - self.distance_tolerance):
                self.get_logger().info(f"[MOTION] ✅ Target distance reached: {distance_traveled:.2f}m (MAVROS-VISION verified)")
                break
                
            # Log progress every 3 seconds with MAVROS vision status
            current_time = time.time()
            if (current_time - last_log_time) >= 3.0:
                depth_error = abs(current_pos['z'] - self.target_depth)
                velocity = self.robot_control.get_current_velocity()
                heading = math.degrees(current_pos['yaw'])
                vel_status = "SPEED_OK" if velocity.get('valid', False) else "SPEED_STALE"
                
                self.get_logger().info(f"[MOTION] MAVROS-VISION Progress: {distance_traveled:.1f}m/{distance_meters}m, "
                                     f"depth: {current_pos['z']:.2f}m (±{depth_error:.2f}m), "
                                     f"vel: fwd={velocity['x']:.2f}, lat={velocity['y']:.2f}m/s, "
                                     f"heading: {heading:.1f}° ({vel_status})")
                last_log_time = current_time
            
            time.sleep(0.2)
        
        # Stop forward movement
        self.robot_control.set_movement_command(forward=0.0, yaw=0.0)
        
        # Log final results with MAVROS vision data
        final_pos = self.robot_control.get_current_position()
        if final_pos.get('valid', False):
            final_distance = ((final_pos['x'] - start_pos['x'])**2 + 
                             (final_pos['y'] - start_pos['y'])**2)**0.5
            heading_change = math.degrees(final_pos['yaw'] - start_pos['yaw'])
            self.get_logger().info(f"[MOTION] ✅ {description} complete (MAVROS-VISION verified) - "
                                  f"Distance: {final_distance:.2f}m, "
                                  f"Final pos: x={final_pos['x']:.2f}, y={final_pos['y']:.2f}, depth={final_pos['z']:.2f}m, "
                                  f"Heading change: {heading_change:.1f}°")
        else:
            self.get_logger().warn(f"[MOTION] ⚠️ {description} complete (MAVROS-VISION lost) - Position uncertain")
        
        return True

    def move_forward_time_based(self, distance_meters, description=""):
        """Fallback time-based forward movement when MAVROS vision data is unavailable"""
        self.get_logger().warn(f"[MOTION] {description} - Using TIME-BASED fallback (no MAVROS vision pose)")
        
        # Estimate time based on expected speed
        estimated_speed = 1.0  # m/s estimated forward speed at full throttle
        estimated_time = distance_meters / estimated_speed
        
        self.get_logger().info(f"[MOTION] Estimated time for {distance_meters}m at {estimated_speed}m/s: {estimated_time:.1f}s")
        
        # Start moving forward
        self.robot_control.set_movement_command(forward=self.forward_speed, yaw=0.0)
        
        # Move for estimated time while monitoring depth and velocity from MAVROS vision
        start_time = time.time()
        last_log_time = start_time
        
        while (time.time() - start_time) < estimated_time:
            current_time = time.time()
            elapsed = current_time - start_time
            
            # Log progress every 2 seconds
            if (current_time - last_log_time) >= 2.0:
                current_depth = self.robot_control.get_current_depth()
                velocity = self.robot_control.get_current_velocity()
                estimated_distance = elapsed * estimated_speed
                vel_status = "SPEED_OK" if velocity.get('valid', False) else "SPEED_STALE"
                
                self.get_logger().info(f"[MOTION] Time-based progress: {elapsed:.1f}s/{estimated_time:.1f}s "
                                     f"(~{estimated_distance:.1f}m/{distance_meters}m), "
                                     f"depth: {current_depth:.2f}m, vision-vel: {velocity['x']:.2f}m/s ({vel_status})")
                last_log_time = current_time
            
            time.sleep(0.2)
        
        # Stop forward movement
        self.robot_control.set_movement_command(forward=0.0, yaw=0.0)
        
        final_depth = self.robot_control.get_current_depth()
        final_velocity = self.robot_control.get_current_velocity()
        vel_status = "SPEED_OK" if final_velocity.get('valid', False) else "SPEED_STALE"
        self.get_logger().warn(f"[MOTION] ⚠️ {description} complete (TIME-BASED fallback) - "
                              f"Estimated distance: {distance_meters}m, "
                              f"Final depth: {final_depth:.2f}m, final vel: {final_velocity['x']:.2f}m/s ({vel_status})")
        return True

    def pause_and_monitor_depth(self, pause_duration):
        """Pause while maintaining depth using MAVROS vision pose data"""
        self.get_logger().info(f"[MOTION] Pausing {pause_duration}s while maintaining depth (MAVROS vision monitoring)")
        
        start_time = time.time()
        log_interval = 2.0  # Log every 2 seconds during pause
        last_log_time = start_time
        
        while (time.time() - start_time) < pause_duration:
            current_depth = self.robot_control.get_current_depth()  # MAVROS vision pose z
            current_time = time.time()
            
            # Check depth drift
            depth_error = abs(current_depth - self.target_depth)
            if depth_error > (self.depth_tolerance * 2):
                self.get_logger().warn(f"[DEPTH] Depth drift detected (MAVROS-VISION): {current_depth:.2f}m "
                                     f"(target: {self.target_depth}m, error: {depth_error:.2f}m)")
                # Re-set target depth
                self.robot_control.set_depth(self.target_depth)
            
            # Log status during pause
            if (current_time - last_log_time) >= log_interval:
                velocity = self.robot_control.get_current_velocity()
                pos = self.robot_control.get_current_position()
                remaining_time = pause_duration - (current_time - start_time)
                pose_status = "POSE_OK" if pos.get('valid', False) else "POSE_STALE"
                speed_status = "SPEED_OK" if velocity.get('valid', False) else "SPEED_STALE"
                
                self.get_logger().info(f"[PAUSE] Holding position - depth: {current_depth:.2f}m "
                                     f"(±{depth_error:.2f}m), vel: {velocity['z']:.3f}m/s, "
                                     f"remaining: {remaining_time:.1f}s ({pose_status}, {speed_status})")
                last_log_time = current_time
            
            time.sleep(0.5)

    def wait_for_mavros_vision_data(self, max_wait_time=15.0):
        """Wait for valid MAVROS vision data before starting mission"""
        self.get_logger().info("[INFO] Waiting for MAVROS vision pose and speed data...")
        
        wait_start = time.time()
        while (time.time() - wait_start) < max_wait_time:
            pos = self.robot_control.get_current_position()
            vel = self.robot_control.get_current_velocity()
            
            if pos.get('valid', False):
                # Log detailed MAVROS vision status
                self.get_logger().info(f"[INFO] ✅ MAVROS vision data available:")
                self.get_logger().info(f"[INFO]   Vision Pose: x={pos['x']:.3f}m, y={pos['y']:.3f}m, z={pos['z']:.3f}m")
                self.get_logger().info(f"[INFO]   Vision Speed: x={vel['x']:.3f}m/s, y={vel['y']:.3f}m/s, z={vel['z']:.3f}m/s")
                self.get_logger().info(f"[INFO]   IMU Heading: {math.degrees(pos['yaw']):.1f}°")
                vel_status = "SPEED_OK" if vel.get('valid', False) else "SPEED_STALE"
                self.get_logger().info(f"[INFO]   Speed Status: {vel_status}")
                return True
            
            elapsed = time.time() - wait_start
            self.get_logger().info(f"[INFO] Waiting for MAVROS vision data... ({elapsed:.1f}s)")
            time.sleep(1.0)
        
        self.get_logger().warn(f"[WARNING] MAVROS vision data not available after {max_wait_time}s - mission may be less accurate")
        return False

    def log_mission_start_status(self):
        """Log detailed status at mission start"""
        pos = self.robot_control.get_current_position()
        vel = self.robot_control.get_current_velocity()
        
        self.get_logger().info("[INFO] === MISSION START STATUS (MAVROS-VISION) ===")
        
        if pos.get('valid', False):
            self.get_logger().info(f"[INFO] MAVROS Vision Pose Status: VALID")
            self.get_logger().info(f"[INFO] Initial Position: x={pos['x']:.3f}m, y={pos['y']:.3f}m, z={pos['z']:.3f}m")
            self.get_logger().info(f"[INFO] Initial Heading: {math.degrees(pos['yaw']):.1f}°")
        else:
            self.get_logger().warn("[WARNING] MAVROS Vision Pose Status: INVALID")
        
        if vel.get('valid', False):
            self.get_logger().info(f"[INFO] MAVROS Vision Speed Status: VALID")
            self.get_logger().info(f"[INFO] Initial Velocity: x={vel['x']:.3f}m/s, y={vel['y']:.3f}m/s, z={vel['z']:.3f}m/s")
        else:
            self.get_logger().warn("[WARNING] MAVROS Vision Speed Status: INVALID")
        
        self.get_logger().info(f"[INFO] Target Depth: {self.target_depth}m")
        self.get_logger().info(f"[INFO] Forward Distance: {self.forward_distance_1}m")
        self.get_logger().info(f"[INFO] Depth Tolerance: ±{self.depth_tolerance}m")
        self.get_logger().info(f"[INFO] Distance Tolerance: ±{self.distance_tolerance}m")
        self.get_logger().info("[INFO] ")

    def run(self):
        self.get_logger().info("[INFO] Starting Straight Left Mission with MAVROS Vision Topics Control")
        
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
               
            # Step 3: Wait for MAVROS vision data to be ready
            self.get_logger().info("[INFO] Waiting for MAVROS vision data and control systems...")
            vision_ready = self.wait_for_mavros_vision_data(max_wait_time=15.0)
            
            if not vision_ready:
                self.get_logger().warn("[WARNING] Proceeding without confirmed MAVROS vision data")
            
            # Step 4: Log mission status
            self.log_mission_start_status()
            time.sleep(2.0)

            # Step 5: Descend to target depth (using MAVROS vision pose)
            if not self.descend_to_depth(self.target_depth):
                self.get_logger().error("[ERROR] Failed to reach target depth")
                return False

            # Step 6: Execute forward movement (using MAVROS vision pose feedback)
            if not self.move_forward_distance(self.forward_distance_1, "Primary forward movement"):
                self.get_logger().error("[ERROR] Failed to complete forward movement")
                return False

            # Step 7: Final pause and depth check
            self.pause_and_monitor_depth(self.pause_time)

            # Step 8: Log final mission status
            final_pos = self.robot_control.get_current_position()
            final_vel = self.robot_control.get_current_velocity()
            
            if final_pos.get('valid', False):
                self.get_logger().info("[INFO] === MISSION COMPLETE STATUS (MAVROS-VISION) ===")
                self.get_logger().info(f"[INFO] Final Position: x={final_pos['x']:.3f}m, y={final_pos['y']:.3f}m, z={final_pos['z']:.3f}m")
                self.get_logger().info(f"[INFO] Final Heading: {math.degrees(final_pos['yaw']):.1f}°")
                if final_vel.get('valid', False):
                    self.get_logger().info(f"[INFO] Final Velocity: x={final_vel['x']:.3f}m/s, y={final_vel['y']:.3f}m/s, z={final_vel['z']:.3f}m/s")
                self.get_logger().info("[INFO]")

            self.get_logger().info("[INFO] ✅ Mission completed successfully! (MAVROS Vision Topics Mode)")
            return True

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
            mission.get_logger().info("[INFO] Mission interrupted ")
        finally:
            mission.robot_control.stop()
            mission.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()