import time
import rclpy
from rclpy.node import Node
import math
from auton_sub.utils import arm, disarm
from auton_sub.utils.guided import set_guided_mode
from rclpy.executors import SingleThreadedExecutor
import threading

from auton_sub.motion.robot_control import RobotControl


class StraightLeftMission(Node):
    def __init__(self):
        super().__init__('straight_left_mission')

        self.robot_control = RobotControl()
        self._rc_exec = SingleThreadedExecutor()
        self._rc_exec.add_node(self.robot_control)
        self._rc_spin_thread = threading.Thread(target=self._rc_exec.spin, daemon=True)
        self._rc_spin_thread.start()
        self.get_logger().info("[INFO] Straight Left Mission Node Initialized (MAVROS Vision Topics Mode)")

        # Mission parameters - FULL SPEED OPERATION
        self.target_depth = -0.4      # meters below surface (positive = down)
        self.forward_distance_1 = 13.0  # meters
        self.pause_time = 2.0         # seconds to pause between steps
        self.forward_speed = 1.0     # Full speed forward command (1.0 = max)
        self.descent_delay_before_forward = 2.0  # Start forward movement after 2s of descent
        
        # Tolerances - adjusted for MAVROS vision topics operation
        self.depth_tolerance = 0.3    # 30cm tolerance for depth (MAVROS vision pose)
        self.distance_tolerance = 0.5  # 50cm tolerance for distance

    def descend_and_move_forward(self, target_depth=-0.5, forward_distance=13.0):
        """Descend to depth while simultaneously moving forward after initial descent delay"""
        self.get_logger().info(f"[MISSION] Starting concurrent descent to {target_depth}m and forward movement {forward_distance}m")
        
        # Log current position before starting
        current_depth = self.robot_control.get_current_depth()
        start_pos = self.robot_control.get_current_position()
        
        self.get_logger().info(f"[START] Current depth: {current_depth:.2f}m (MAVROS vision pose)")
        if start_pos.get('valid', False):
            self.get_logger().info(f"[START] Starting position: x={start_pos['x']:.2f}m, y={start_pos['y']:.2f}m")
        
        # Set target depth and start descending
        self.robot_control.set_depth(target_depth)
        self.robot_control.set_max_descent_rate(True)
        
        # Mission timing
        mission_start_time = time.time()
        forward_start_time = mission_start_time + self.descent_delay_before_forward
        forward_started = False
        descent_completed = False
        forward_completed = False
        
        # Maximum time limits
        max_descent_time = 30.0
        max_forward_time = (forward_distance / 0.5) + 15.0  # Generous time for forward movement
        max_total_time = max(max_descent_time, max_forward_time + self.descent_delay_before_forward)
        
        last_log_time = mission_start_time
        
        while (time.time() - mission_start_time) < max_total_time:
            current_time = time.time()
            elapsed_time = current_time - mission_start_time
            
            # Get current status
            current_depth = self.robot_control.get_current_depth()
            current_pos = self.robot_control.get_current_position()
            current_vel = self.robot_control.get_current_velocity()
            
            # Check depth progress
            depth_error = abs(current_depth - target_depth)
            if not descent_completed and depth_error < self.depth_tolerance:
                self.get_logger().info(f"[DEPTH] ✅ Target depth achieved: {current_depth:.2f}m (target: {target_depth}m)")
                self.robot_control.set_max_descent_rate(False)  # Switch to normal depth control
                descent_completed = True
            
            # Start forward movement after delay
            if not forward_started and elapsed_time >= self.descent_delay_before_forward:
                self.get_logger().info(f"[MOTION] 🚀 Starting forward movement after {self.descent_delay_before_forward}s descent delay")
                self.robot_control.set_movement_command(forward=self.forward_speed, yaw=0.0)
                forward_started = True
                forward_start_pos = current_pos.copy()  # Record position when forward movement starts
            
            # Check forward movement progress (only after forward movement has started)
            if forward_started and not forward_completed and start_pos.get('valid', False) and current_pos.get('valid', False):
                # Calculate distance traveled from mission start
                dx = current_pos['x'] - start_pos['x']
                dy = current_pos['y'] - start_pos['y']
                distance_traveled = (dx**2 + dy**2)**0.5
                
                if distance_traveled >= (forward_distance - self.distance_tolerance):
                    self.get_logger().info(f"[MOTION] ✅ Target distance reached: {distance_traveled:.2f}m")
                    self.robot_control.set_movement_command(forward=0.0, yaw=0.0)  # Stop forward movement
                    forward_completed = True
            
            # Check if both objectives are complete
            if descent_completed and forward_completed:
                self.get_logger().info("[MISSION] ✅ Both descent and forward movement completed!")
                break
            
            # Logging every 2 seconds
            if (current_time - last_log_time) >= 2.0:
                # Calculate progress
                if start_pos.get('valid', False) and current_pos.get('valid', False):
                    dx = current_pos['x'] - start_pos['x']
                    dy = current_pos['y'] - start_pos['y']
                    distance_traveled = (dx**2 + dy**2)**0.5
                else:
                    distance_traveled = 0.0
                
                # Status indicators
                vision_status = "VISION_OK" if current_pos.get('valid', False) else "VISION_STALE"
                vel_status = "SPEED_OK" if current_vel.get('valid', False) else "SPEED_STALE"
                
                # Determine current phase
                if not forward_started:
                    phase = f"DESCENT_ONLY (forward in {self.descent_delay_before_forward - elapsed_time:.1f}s)"
                elif not forward_completed:
                    phase = "DESCENT+FORWARD"
                else:
                    phase = "DEPTH_HOLD"
                
                self.get_logger().info(f"[STATUS] {phase} | "
                                     f"Depth: {current_depth:.2f}m→{target_depth}m (±{depth_error:.2f}m) | "
                                     f"Distance: {distance_traveled:.1f}m/{forward_distance}m | "
                                     f"Vel: fwd={current_vel['x']:.2f}, down={current_vel['z']:.3f}m/s | "
                                     f"Time: {elapsed_time:.1f}s ({vision_status}, {vel_status})")
                
                last_log_time = current_time
            
            time.sleep(0.2)
        
        # Final status
        final_pos = self.robot_control.get_current_position()
        final_depth = self.robot_control.get_current_depth()
        
        # Stop all movement
        self.robot_control.set_movement_command(forward=0.0, yaw=0.0)
        self.robot_control.set_max_descent_rate(False)
        
        # Calculate final results
        if start_pos.get('valid', False) and final_pos.get('valid', False):
            final_distance = ((final_pos['x'] - start_pos['x'])**2 + 
                             (final_pos['y'] - start_pos['y'])**2)**0.5
            heading_change = math.degrees(final_pos['yaw'] - start_pos['yaw'])
            
            self.get_logger().info(f"[FINAL] Mission Results (MAVROS-VISION):")
            self.get_logger().info(f"[FINAL]   Depth: {final_depth:.2f}m (target: {target_depth}m, error: {abs(final_depth - target_depth):.2f}m)")
            self.get_logger().info(f"[FINAL]   Distance: {final_distance:.2f}m (target: {forward_distance}m)")
            self.get_logger().info(f"[FINAL]   Final position: x={final_pos['x']:.2f}, y={final_pos['y']:.2f}")
            self.get_logger().info(f"[FINAL]   Heading change: {heading_change:.1f}°")
        else:
            self.get_logger().warn(f"[FINAL] Mission completed but MAVROS vision data unavailable for final measurements")
            self.get_logger().info(f"[FINAL]   Final depth: {final_depth:.2f}m (target: {target_depth}m)")
        
        # Determine success
        depth_success = abs(final_depth - target_depth) < (self.depth_tolerance * 2)
        distance_success = (not start_pos.get('valid', False)) or forward_completed  # Success if no vision data or completed
        
        success = depth_success and distance_success
        if success:
            self.get_logger().info("[FINAL] ✅ Concurrent descent and forward movement SUCCESS")
        else:
            self.get_logger().warn(f"[FINAL] ⚠️ Mission partially completed - depth: {'✅' if depth_success else '❌'}, distance: {'✅' if distance_success else '❌'}")
        
        return success

    def pause_and_monitor_depth(self, pause_duration):
        """Pause while maintaining depth using MAVROS vision pose data"""
        self.get_logger().info(f"[MOTION] Pausing {pause_duration}s while maintaining depth (MAVROS vision monitoring)...")
        
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
        self.get_logger().info(f"[INFO] Descent Delay Before Forward: {self.descent_delay_before_forward}s")
        self.get_logger().info(f"[INFO] Depth Tolerance: ±{self.depth_tolerance}m")
        self.get_logger().info(f"[INFO] Distance Tolerance: ±{self.distance_tolerance}m")
        self.get_logger().info("[INFO] ==========================================")

    def run(self):
        self.get_logger().info("[INFO] Starting Straight Left Mission with CONCURRENT Descent and Forward Movement")
        
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

            # Step 5: Execute concurrent descent and forward movement
            if not self.descend_and_move_forward(self.target_depth, self.forward_distance_1):
                self.get_logger().error("[ERROR] Failed to complete concurrent descent and forward movement")
                return False

            # Step 6: Final pause and depth check
            self.pause_and_monitor_depth(self.pause_time)

            # Step 7: Log final mission status
            final_pos = self.robot_control.get_current_position()
            final_vel = self.robot_control.get_current_velocity()
            
            if final_pos.get('valid', False):
                self.get_logger().info("[INFO] === MISSION COMPLETE STATUS (MAVROS-VISION) ===")
                self.get_logger().info(f"[INFO] Final Position: x={final_pos['x']:.3f}m, y={final_pos['y']:.3f}m, z={final_pos['z']:.3f}m")
                self.get_logger().info(f"[INFO] Final Heading: {math.degrees(final_pos['yaw']):.1f}°")
                if final_vel.get('valid', False):
                    self.get_logger().info(f"[INFO] Final Velocity: x={final_vel['x']:.3f}m/s, y={final_vel['y']:.3f}m/s, z={final_vel['z']:.3f}m/s")
                self.get_logger().info("[INFO] ==========================================")

            self.get_logger().info("[INFO] ✅ Mission completed successfully! (CONCURRENT Descent + Forward Movement)")
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
            mission.get_logger().info("[INFO] Mission interrupted by user")
        finally:
            mission.robot_control.stop()
            mission.destroy_node()
            mission._rc_exec.shutdown()
            mission._rc_spin_thread.join(timeout=1.0)
            rclpy.shutdown()

if __name__ == '__main__':
    main()