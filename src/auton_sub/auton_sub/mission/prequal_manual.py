import time
import rclpy
from rclpy.node import Node
import math
from auton_sub.utils import arm, disarm
#from auton_sub.utils.guided import set_guided_mode

from rclpy.executors import SingleThreadedExecutor
import threading
from auton_sub.motion.robot_control import RobotControl


class ManualMission(Node):
    def __init__(self):
        super().__init__('manual_mission')

        self.robot_control = RobotControl()
        self._rc_exec = SingleThreadedExecutor()
        self._rc_exec.add_node(self.robot_control)
        self._rc_spin_thread = threading.Thread(target=self._rc_exec.spin, daemon=True)
        self._rc_spin_thread.start()
        self.get_logger().info("[INFO] Manual Mission Node Initialized")
    
    def manualcontrol(self, forward, lateral, vertical, yaw_rate, duration):
        forward = float(forward)
        lateral = float(lateral)
        vertical = float(vertical)
        yaw_rate = float(yaw_rate)
        duration = float(duration)

        try:
            self.get_logger().info(f"[INFO] Sending manual control: forward={forward}, lateral={lateral}, vertical={vertical}, yaw={yaw_rate} for {duration}s")
            self.robot_control.send_manual_control(forward, lateral, vertical, yaw_rate)
            time.sleep(duration)
            self.stop_thrusters()
            self.get_logger().info("[INFO] Manual control completed")
        except Exception as e:
            self.get_logger().error(f"[ERROR] Manual control failed: {e}")
            self.stop_thrusters()  # Always try to stop on error

    def stop_thrusters(self):
        """Stop all thrusters by sending neutral commands"""
        try:
            self.get_logger().info("[INFO] Stopping thrusters...")
            self.robot_control.send_manual_control(0.0, 0.0, 0.0, 0.0)
            time.sleep(0.1)  # Brief delay to ensure command is sent
        except Exception as e:
            self.get_logger().error(f"[ERROR] Failed to stop thrusters: {e}")

    def run(self):
        self.get_logger().info("[INFO] Starting Manual Mission")
        
        try:
            # Step 1: Arm the vehicle
            self.get_logger().info("[INFO] Arming vehicle...")
            arm_node = arm.ArmerNode()
            time.sleep(2.0)
                            
            # Step 2: Manual thruster test
            self.get_logger().info("[INFO] Starting thruster test...")
            self.manualcontrol(1.0, 0.0, 0.0, 0.0, 3.0)
            
            self.get_logger().info("[INFO] Mission completed successfully!")
            return True

        except KeyboardInterrupt:
            self.get_logger().info("[INFO] Mission interrupted")
            self.stop_thrusters()
            return False
        except Exception as e:
            self.get_logger().error(f"[ERROR] Mission failed: {e}")
            self.stop_thrusters()
            return False

    def cleanup(self):
        """Clean up resources"""
        try:
            self.get_logger().info("[INFO] Cleaning up...")
            self.stop_thrusters()
            
            # Stop the robot control if it has a stop method
            if hasattr(self.robot_control, 'stop') and callable(getattr(self.robot_control, 'stop')):
                self.robot_control.stop()
            else:
                self.get_logger().warn("[WARN] RobotControl doesn't have a stop method")
                
        except Exception as e:
            self.get_logger().error(f"[ERROR] Cleanup failed: {e}")
            
def main():
    rclpy.init()
    mission = None
    
    try:
        mission = ManualMission()
        success = mission.run()
        
        if success:
            mission.get_logger().info("[INFO] Mission completed successfully!")
        else:
            mission.get_logger().error("[ERROR] Mission failed!")
            
    except KeyboardInterrupt:
        if mission:
            mission.get_logger().info("[INFO] Mission interrupted")
    except Exception as e:
        if mission:
            mission.get_logger().error(f"[ERROR] Unexpected error: {e}")
        else:
            print(f"[ERROR] Failed to create mission: {e}")
    finally:
        if mission:
            mission.cleanup()
            mission.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()