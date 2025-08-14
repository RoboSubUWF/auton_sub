#!/usr/bin/env python3
"""
Complete Sequential Mission Controller - ROS2 Node Version
Runs all missions in sequence: coin_toss -> gate -> path -> slalom -> path -> ocean_cleanup -> home
without surfacing between missions. Maintains depth continuity throughout, then surfaces at the end.
"""

import rclpy
import time
import sys
import os
from rclpy.node import Node

# Import all mission classes
from auton_sub.mission.coin_toss import CoinTossMission
from auton_sub.mission.gate import GateMission
from auton_sub.mission.path import PathFollowingMission
from auton_sub.mission.slalom import SlalomMission
from auton_sub.mission.Ocean_Cleanup import OceanCleanupMission
from auton_sub.mission.home import HomeMission


class CompleteMissionController(Node):
    def __init__(self):
        super().__init__('complete_mission_controller')
        self.get_logger().info("[CONTROLLER] Complete Mission Controller initialized")
        
        # Mission state tracking
        self.shark_side = None  # Will be passed from gate to slalom
        self.current_depth = 2.0  # Target depth maintained throughout
        
    def cleanup_mission(self, mission, mission_name):
        """Clean up a mission's vision system and robot control"""
        if mission is None:
            return
            
        try:
            self.get_logger().info(f"[CONTROLLER] Cleaning up {mission_name} mission...")
            
            # Disable detection
            mission.toggle_detection(False)
            
            # Unload model to save resources
            from std_msgs.msg import String
            msg = String()
            msg.data = ""
            mission.model_command_pub.publish(msg)
            
            # Stop all movement
            mission.robot_control.set_movement_command(forward=0.0, yaw=0.0)
            
            # Get final state
            final_pos = mission.robot_control.get_current_position()
            final_depth = mission.robot_control.get_current_depth()
            self.get_logger().info(f"[CONTROLLER] {mission_name} final state: "
                                 f"x={final_pos.get('x', 0):.2f}m, y={final_pos.get('y', 0):.2f}m, "
                                 f"depth={final_depth:.2f}m")
            
            # Stop robot control and destroy node
            mission.robot_control.stop()
            mission.destroy_node()
            
            self.get_logger().info(f"[CONTROLLER] ✅ {mission_name} cleanup completed")
            
        except Exception as e:
            self.get_logger().warn(f"[CONTROLLER] Warning during {mission_name} cleanup: {e}")

    def cleanup_home_mission(self, mission, mission_name):
        """Special cleanup for home mission - motors are already stopped"""
        if mission is None:
            return
            
        try:
            self.get_logger().info(f"[CONTROLLER] Cleaning up {mission_name} mission...")
            
            # Home mission already stops all motors and surfaces, so minimal cleanup needed
            # Just ensure detection is disabled and model unloaded
            try:
                mission.toggle_detection(False)
                from std_msgs.msg import String
                msg = String()
                msg.data = ""
                mission.model_command_pub.publish(msg)
            except:
                pass  # May already be disabled
            
            # Get final state
            final_pos = mission.robot_control.get_current_position()
            final_depth = mission.robot_control.get_current_depth()
            self.get_logger().info(f"[CONTROLLER] {mission_name} final state: "
                                 f"x={final_pos.get('x', 0):.2f}m, y={final_pos.get('y', 0):.2f}m, "
                                 f"depth={final_depth:.2f}m")
            
            # Robot control already stopped by home mission, just destroy node
            mission.destroy_node()
            
            self.get_logger().info(f"[CONTROLLER] ✅ {mission_name} cleanup completed")
            
        except Exception as e:
            self.get_logger().warn(f"[CONTROLLER] Warning during {mission_name} cleanup: {e}")

    def transition_pause(self, duration=3.0, message=""):
        """Pause between missions with status message"""
        if message:
            self.get_logger().info(f"[CONTROLLER] {message}")
        self.get_logger().info(f"[CONTROLLER] Pausing {duration} seconds before next mission...")
        time.sleep(duration)

    def run_all_missions(self):
        """Main function to run all missions sequentially"""
        self.get_logger().info("[CONTROLLER] ========== STARTING COMPLETE MISSION SEQUENCE ==========")
        self.get_logger().info("[CONTROLLER] Mission order: coin_toss -> gate -> path -> slalom -> path -> ocean_cleanup -> home")
        
        overall_success = False
        current_mission = None
        mission_results = {
            'coin_toss': False,
            'gate': False,
            'path_1': False,
            'slalom': False,
            'path_2': False,
            'ocean_cleanup': False,
            'home': False
        }
        
        try:
            # ========== MISSION 1: COIN TOSS ==========
            self.get_logger().info("[CONTROLLER] ========== MISSION 1: COIN TOSS ==========")
            
            current_mission = CoinTossMission()
            mission_results['coin_toss'] = current_mission.run()
            
            if mission_results['coin_toss']:
                self.get_logger().info("[CONTROLLER] ✅ MISSION 1: Coin Toss completed successfully!")
                self.get_logger().info("[CONTROLLER] Submarine is at 2m depth and positioned for gate approach")
                self.cleanup_mission(current_mission, "Coin Toss")
                current_mission = None
                self.transition_pause(3.0, "Submarine maintaining 2m depth for gate mission")
            else:
                self.get_logger().error("[CONTROLLER] ❌ MISSION 1: Coin Toss failed! Aborting sequence.")
                return False

            # ========== MISSION 2: GATE ==========
            self.get_logger().info("[CONTROLLER] ========== MISSION 2: GATE ==========")
            
            current_mission = GateMission()
            mission_results['gate'] = current_mission.run()
            
            if mission_results['gate']:
                self.get_logger().info("[CONTROLLER] ✅ MISSION 2: Gate completed successfully!")
                
                # Extract shark side information for slalom mission
                self.shark_side = current_mission.shark_side
                self.get_logger().info(f"[CONTROLLER] Shark was detected on {self.shark_side} side - passing to slalom mission")
                
                self.cleanup_mission(current_mission, "Gate")
                current_mission = None
                self.transition_pause(3.0, "Submarine has passed through gate, maintaining depth for path following")
            else:
                self.get_logger().error("[CONTROLLER] ❌ MISSION 2: Gate failed! Aborting sequence.")
                return False

            # ========== MISSION 3: PATH FOLLOWING (First) ==========
            self.get_logger().info("[CONTROLLER] ========== MISSION 3: PATH FOLLOWING (First) ==========")
            
            current_mission = PathFollowingMission()
            mission_results['path_1'] = current_mission.run()
            
            if mission_results['path_1']:
                self.get_logger().info("[CONTROLLER] ✅ MISSION 3: First Path Following completed successfully!")
                self.get_logger().info("[CONTROLLER] Submarine has followed red path and is positioned for slalom")
                self.cleanup_mission(current_mission, "First Path Following")
                current_mission = None
                self.transition_pause(3.0, "Submarine maintaining depth for slalom mission")
            else:
                self.get_logger().error("[CONTROLLER] ❌ MISSION 3: First Path Following failed! Aborting sequence.")
                return False

            # ========== MISSION 4: SLALOM ==========
            self.get_logger().info("[CONTROLLER] ========== MISSION 4: SLALOM ==========")
            
            # Pass shark side information to slalom mission
            side_for_slalom = self.shark_side if self.shark_side else "left"  # Default to left if not detected
            self.get_logger().info(f"[CONTROLLER] Configuring slalom for {side_for_slalom} side pattern")
            
            current_mission = SlalomMission(side=side_for_slalom)
            mission_results['slalom'] = current_mission.run()
            
            if mission_results['slalom']:
                self.get_logger().info("[CONTROLLER] ✅ MISSION 4: Slalom completed successfully!")
                self.get_logger().info(f"[CONTROLLER] Submarine completed {side_for_slalom} side slalom pattern")
                self.cleanup_mission(current_mission, "Slalom")
                current_mission = None
                self.transition_pause(3.0, "Submarine maintaining depth for second path following")
            else:
                self.get_logger().error("[CONTROLLER] ❌ MISSION 4: Slalom failed! Aborting sequence.")
                return False

            # ========== MISSION 5: PATH FOLLOWING (Second) ==========
            self.get_logger().info("[CONTROLLER] ========== MISSION 5: PATH FOLLOWING (Second) ==========")
            
            current_mission = PathFollowingMission()
            mission_results['path_2'] = current_mission.run()
            
            if mission_results['path_2']:
                self.get_logger().info("[CONTROLLER] ✅ MISSION 5: Second Path Following completed successfully!")
                self.get_logger().info("[CONTROLLER] Submarine has followed path to ocean cleanup area")
                self.cleanup_mission(current_mission, "Second Path Following")
                current_mission = None
                self.transition_pause(3.0, "Submarine maintaining depth for ocean cleanup mission")
            else:
                self.get_logger().error("[CONTROLLER] ❌ MISSION 5: Second Path Following failed! Aborting sequence.")
                return False

            # ========== MISSION 6: OCEAN CLEANUP ==========
            self.get_logger().info("[CONTROLLER] ========== MISSION 6: OCEAN CLEANUP ==========")
            
            current_mission = OceanCleanupMission()
            mission_results['ocean_cleanup'] = current_mission.run()
            
            if mission_results['ocean_cleanup']:
                self.get_logger().info("[CONTROLLER] ✅ MISSION 6: Ocean Cleanup completed successfully!")
                self.get_logger().info("[CONTROLLER] Submarine has completed ocean cleanup tasks")
                self.cleanup_mission(current_mission, "Ocean Cleanup")
                current_mission = None
                self.transition_pause(3.0, "Submarine maintaining depth for home mission")
            else:
                self.get_logger().error("[CONTROLLER] ❌ MISSION 6: Ocean Cleanup failed! Continuing to home mission anyway...")
                # Don't return False here - still attempt to go home even if cleanup failed
                if current_mission:
                    self.cleanup_mission(current_mission, "Ocean Cleanup")
                    current_mission = None

            # ========== MISSION 7: HOME ==========
            self.get_logger().info("[CONTROLLER] ========== MISSION 7: HOME (RETURN AND SURFACE) ==========")
            self.get_logger().info("[CONTROLLER] Beginning return journey - finding gate from behind and surfacing")
            
            current_mission = HomeMission()
            mission_results['home'] = current_mission.run()
            
            if mission_results['home']:
                self.get_logger().info("[CONTROLLER] ✅ MISSION 7: Home completed successfully!")
                self.get_logger().info("[CONTROLLER] 🏠 Submarine has returned through gate and surfaced!")
                self.get_logger().info("[CONTROLLER] 🛑 All motors stopped by home mission")
                overall_success = True
                self.cleanup_home_mission(current_mission, "Home")
                current_mission = None
                self.get_logger().info("[CONTROLLER] 🏁 ALL MISSIONS INCLUDING HOME COMPLETED SUCCESSFULLY!")
            else:
                self.get_logger().error("[CONTROLLER] ❌ MISSION 7: Home failed!")
                self.get_logger().warn("[CONTROLLER] Submarine may still be submerged - manual intervention may be required")
                
        except KeyboardInterrupt:
            self.get_logger().info("[CONTROLLER] Mission sequence interrupted by user")
        except Exception as e:
            self.get_logger().error(f"[CONTROLLER] Exception occurred: {e}")
            import traceback
            traceback.print_exc()
        finally:
            # Final cleanup
            self.get_logger().info("[CONTROLLER] Performing final cleanup...")
            
            if current_mission:
                try:
                    current_mission.robot_control.set_movement_command(forward=0.0, yaw=0.0)
                    current_mission.robot_control.stop()
                    current_mission.destroy_node()
                except Exception as e:
                    self.get_logger().error(f"[CONTROLLER] Error cleaning up current mission: {e}")
            
            # Print final mission summary
            self.get_logger().info("[CONTROLLER] ========== MISSION SUMMARY ==========")
            for mission_name, success in mission_results.items():
                status = "✅ SUCCESS" if success else "❌ FAILED"
                self.get_logger().info(f"[CONTROLLER] {mission_name.upper().replace('_', ' ')}: {status}")
            
            success_count = sum(mission_results.values())
            total_missions = len(mission_results)
            self.get_logger().info(f"[CONTROLLER] Overall: {success_count}/{total_missions} missions successful")
            
            if overall_success:
                self.get_logger().info("[CONTROLLER] 🎉 COMPLETE MISSION SEQUENCE WITH HOME SUCCESSFUL!")
                self.get_logger().info("[CONTROLLER] 🌊 Submarine has completed all competition tasks:")
                self.get_logger().info("[CONTROLLER] 🪙 - Coin toss identification")
                self.get_logger().info("[CONTROLLER] 🚪 - Gate passage with shark/sawfish detection")
                self.get_logger().info("[CONTROLLER] 🔴 - Red path following (x2)")
                self.get_logger().info("[CONTROLLER] 🚧 - Slalom navigation through pipes")
                self.get_logger().info("[CONTROLLER] 🧹 - Ocean cleanup with item collection")
                self.get_logger().info("[CONTROLLER] 🏠 - Home mission: return through gate and surface")
                self.get_logger().info("[CONTROLLER] 🏆 ALL ROBOSUB COMPETITION TASKS + HOME COMPLETED!")
                self.get_logger().info("[CONTROLLER] 🛑 SUBMARINE SURFACED AND ALL MOTORS STOPPED!")
            else:
                self.get_logger().error("[CONTROLLER] 🚫 Mission sequence failed!")
                self.get_logger().info("[CONTROLLER] Check individual mission logs for failure details")
                # If home mission failed, ensure motors are stopped
                self.get_logger().info("[CONTROLLER] Ensuring all motors are stopped as safety measure...")
        
        return overall_success


def main(args=None):
    """Main entry point for ROS2"""
    rclpy.init(args=args)
    
    controller = CompleteMissionController()
    
    try:
        # Print system information
        controller.get_logger().info("[CONTROLLER] ========== SYSTEM INFO ==========")
        controller.get_logger().info("[CONTROLLER] Platform: Jetson Orin Nano")
        controller.get_logger().info("[CONTROLLER] Flight Controller: Pixhawk")
        controller.get_logger().info("[CONTROLLER] Thrusters: 6 (no lateral movement)")
        controller.get_logger().info("[CONTROLLER] Cameras: 2 (front + bottom)")
        controller.get_logger().info("[CONTROLLER] Mission Depth: 2.0m (maintained until home)")
        controller.get_logger().info("[CONTROLLER] Final Action: Surface and stop all motors")
        controller.get_logger().info("[CONTROLLER] ==========================================")
        
        success = controller.run_all_missions()
        
        if success:
            controller.get_logger().info("[CONTROLLER] 🏁 All missions including home completed successfully!")
            controller.get_logger().info("[CONTROLLER] 🛑 Submarine is surfaced with all motors stopped!")
        else:
            controller.get_logger().error("[CONTROLLER] ❌ Mission sequence failed!")
            
    except KeyboardInterrupt:
        controller.get_logger().info("[CONTROLLER] Interrupted by user")
    except Exception as e:
        controller.get_logger().error(f"[CONTROLLER] Fatal error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()