#!/usr/bin/env python3
"""
Sequential Mission Controller - ROS2 Node Version
Runs coin_toss mission followed immediately by gate mission
without surfacing between missions.
"""

import rclpy
import time
import sys
import os
from rclpy.node import Node

# Import the mission classes
from auton_sub.mission.coin_toss import CoinTossMission
from auton_sub.mission.gate import GateMission


class SequentialMissionController(Node):
    def __init__(self):
        super().__init__('sequential_mission_controller')
        self.get_logger().info("[CONTROLLER] Sequential Mission Controller initialized")
    
    def run_missions(self):
        """Main function to run both missions sequentially"""
        self.get_logger().info("[CONTROLLER] ========== STARTING SEQUENTIAL MISSIONS ==========")
        
        overall_success = False
        coin_toss_mission = None
        gate_mission = None
        
        try:
            # ========== MISSION 1: COIN TOSS ==========
            self.get_logger().info("[CONTROLLER] ========== COIN TOSS MISSION ==========")
            
            coin_toss_mission = CoinTossMission()
            
            # Run the coin toss mission
            coin_toss_success = coin_toss_mission.run()
            
            if coin_toss_success:
                self.get_logger().info("[CONTROLLER] ✅ Coin Toss Mission completed successfully!")
                self.get_logger().info("[CONTROLLER] Submarine is at 2m depth and facing the gate")
                
                # Clean up coin toss vision system but keep robot control
                try:
                    self.get_logger().info("[CONTROLLER] Cleaning up coin toss vision system...")
                    coin_toss_mission.toggle_detection(False)
                    
                    # Unload model to save resources
                    from std_msgs.msg import String
                    msg = String()
                    msg.data = ""
                    coin_toss_mission.model_command_pub.publish(msg)
                    
                    # Brief pause to ensure cleanup
                    time.sleep(2.0)
                    
                except Exception as e:
                    self.get_logger().warn(f"[CONTROLLER] Warning during coin toss cleanup: {e}")
                
                # Get final position from coin toss
                final_pos = coin_toss_mission.robot_control.get_current_position()
                final_depth = coin_toss_mission.robot_control.get_current_depth()
                self.get_logger().info(f"[CONTROLLER] Current state: x={final_pos.get('x', 0):.2f}m, y={final_pos.get('y', 0):.2f}m, depth={final_depth:.2f}m")
                
                # Stop coin toss robot control
                coin_toss_mission.robot_control.stop()
                coin_toss_mission.destroy_node()
                coin_toss_mission = None
                
                # Brief pause between missions
                self.get_logger().info("[CONTROLLER] Pausing 3 seconds before starting Gate Mission...")
                time.sleep(3.0)
                
                # ========== MISSION 2: GATE ==========
                self.get_logger().info("[CONTROLLER] ========== GATE MISSION ==========")
                
                gate_mission = GateMission()
                
                # The gate mission should start with the assumption it's already at 2m depth
                gate_success = gate_mission.run()
                
                if gate_success:
                    self.get_logger().info("[CONTROLLER] ✅ Gate Mission completed successfully!")
                    self.get_logger().info("[CONTROLLER] 🏁 ALL MISSIONS COMPLETED SUCCESSFULLY!")
                    overall_success = True
                else:
                    self.get_logger().error("[CONTROLLER] ❌ Gate Mission failed!")
                    
            else:
                self.get_logger().error("[CONTROLLER] ❌ Coin Toss Mission failed! Aborting sequence.")
                
        except KeyboardInterrupt:
            self.get_logger().info("[CONTROLLER] Mission sequence interrupted by user")
        except Exception as e:
            self.get_logger().error(f"[CONTROLLER] Exception occurred: {e}")
            import traceback
            traceback.print_exc()
        finally:
            # Cleanup both missions
            self.get_logger().info("[CONTROLLER] Performing final cleanup...")
            
            if coin_toss_mission:
                try:
                    coin_toss_mission.robot_control.set_movement_command(forward=0.0, yaw=0.0)
                    coin_toss_mission.robot_control.stop()
                    coin_toss_mission.destroy_node()
                except Exception as e:
                    self.get_logger().error(f"[CONTROLLER] Error cleaning up coin toss: {e}")
                    
            if gate_mission:
                try:
                    gate_mission.robot_control.set_movement_command(forward=0.0, yaw=0.0)
                    gate_mission.robot_control.stop()
                    gate_mission.destroy_node()
                except Exception as e:
                    self.get_logger().error(f"[CONTROLLER] Error cleaning up gate mission: {e}")
            
            if overall_success:
                self.get_logger().info("[CONTROLLER] 🎉 Mission sequence completed successfully!")
            else:
                self.get_logger().error("[CONTROLLER] 🚫 Mission sequence failed!")
        
        return overall_success


def main(args=None):
    """Main entry point for ROS2"""
    rclpy.init(args=args)
    
    controller = SequentialMissionController()
    
    try:
        success = controller.run_missions()
        if success:
            controller.get_logger().info("[CONTROLLER] All missions completed successfully!")
        else:
            controller.get_logger().error("[CONTROLLER] Mission sequence failed!")
    except KeyboardInterrupt:
        controller.get_logger().info("[CONTROLLER] Interrupted by user")
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()