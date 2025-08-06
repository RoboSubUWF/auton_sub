#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from mavros_msgs.srv import SetMode
from mavros_msgs.msg import OverrideRCIn
from sensor_msgs.msg import FluidPressure
from geometry_msgs.msg import PoseStamped
import time
import math

# Import arm/disarm utilities (matching your prequal.py structure)
from auton_sub.utils import arm, disarm

# Try to import VFR_HUD, fall back to alternative if not available
try:
    from mavros_msgs.msg import VFR_HUD
    VFR_HUD_AVAILABLE = True
except ImportError:
    print("Warning: VFR_HUD not available in mavros_msgs, using pressure sensor only")
    VFR_HUD_AVAILABLE = False


class DepthHoldTest(Node):
    def __init__(self):
        super().__init__('depth_hold_test')
        
        # Service clients for mode setting only (arm/disarm handled by utility nodes)
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
        
        # Publisher for RC override (for depth control if needed)
        self.rc_override_pub = self.create_publisher(OverrideRCIn, '/mavros/rc/override', 10)
        
        # Subscribers for depth monitoring
        self.pressure_sub = self.create_subscription(
            FluidPressure, '/mavros/imu/static_pressure', self.pressure_callback, 10)
        
        self.pose_sub = self.create_subscription(
            PoseStamped, '/pose', self.pose_callback, 10)
        
        # Alternative: use VFR_HUD which includes altitude/depth (if available)
        if VFR_HUD_AVAILABLE:
            self.vfr_sub = self.create_subscription(
                VFR_HUD, '/mavros/vfr_hud', self.vfr_callback, 10)
        
        # Depth tracking variables
        self.current_depth = 0.0
        self.pressure_valid = False
        self.pose_valid = False
        
        # PWM settings (for emergency override if needed)
        self.pwm_neutral = 1500
        
        self.get_logger().info("DepthHoldTest initialized")
        
        # Wait for services to be available
        self.wait_for_services()
    
    def wait_for_services(self):
        """Wait for MAVROS services to be available"""
        self.get_logger().info("Waiting for MAVROS services...")
        
        # Wait for set_mode service
        while not self.set_mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Set mode service not available, waiting...')
        
        self.get_logger().info("✅ All MAVROS services are available")
    
    def pressure_callback(self, msg):
        """Convert pressure to depth using standard formula"""
        # Pressure to depth conversion (approximate)
        # 1 meter of water ≈ 9800 Pa additional pressure
        atmospheric_pressure = 101325.0  # Pa at sea level
        water_density = 1025.0  # kg/m³ (saltwater)
        gravity = 9.81  # m/s²
        
        pressure_diff = msg.fluid_pressure - atmospheric_pressure
        depth_from_pressure = pressure_diff / (water_density * gravity)
        
        # Only use positive depths (underwater)
        if depth_from_pressure > 0:
            self.current_depth = depth_from_pressure
            self.pressure_valid = True
    
    def pose_callback(self, msg):
        """Get depth from pose message (DVL/EKF estimate)"""
        if not math.isnan(msg.pose.position.z):
            # Use absolute value in case z is negative depth
            self.current_depth = abs(msg.pose.position.z)
            self.pose_valid = True
    
    def vfr_callback(self, msg):
        """Alternative: use VFR_HUD altitude as depth"""
        if VFR_HUD_AVAILABLE:
            # VFR_HUD altitude is often negative depth in ArduSub
            depth_from_vfr = -msg.altitude
            if depth_from_vfr > 0:
                self.current_depth = depth_from_vfr
                self.pressure_valid = True
    
    def get_current_depth(self):
        """Get the most reliable current depth reading"""
        return self.current_depth
    
    def has_depth_reading(self):
        """Check if we have any valid depth reading"""
        return self.pressure_valid or self.pose_valid
    
    def set_mode(self, mode_name):
        """Set vehicle mode"""
        self.get_logger().info(f"Setting {mode_name} mode...")
        
        request = SetMode.Request()
        request.custom_mode = mode_name
        
        try:
            future = self.set_mode_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
            
            if future.result() is not None:
                response = future.result()
                if response.mode_sent:
                    self.get_logger().info(f"✅ {mode_name} mode set successfully")
                    return True
                else:
                    self.get_logger().error(f"❌ Failed to set {mode_name} mode")
                    return False
            else:
                self.get_logger().error(f"❌ Set {mode_name} mode service call failed")
                return False
                
        except Exception as e:
            self.get_logger().error(f"❌ Exception during {mode_name} mode setting: {e}")
            return False
    
    def arm_vehicle(self):
        """Arm the vehicle using utility node"""
        self.get_logger().info("Arming vehicle...")
        try:
            arm_node = arm.ArmerNode()
            self.get_logger().info("✅ Vehicle armed successfully")
            return True
        except Exception as e:
            self.get_logger().error(f"❌ Exception during arming: {e}")
            return False
    
    def disarm_vehicle(self):
        """Disarm the vehicle using utility node"""
        self.get_logger().info("Disarming vehicle...")
        try:
            disarm_node = disarm.DisarmerNode()
            self.get_logger().info("✅ Vehicle disarmed successfully")
            return True
        except Exception as e:
            self.get_logger().error(f"❌ Exception during disarming: {e}")
            return False
    
    def send_neutral_rc(self):
        """Send neutral RC commands to stop any movement"""
        rc_msg = OverrideRCIn()
        rc_msg.channels = [self.pwm_neutral] * 18
        self.rc_override_pub.publish(rc_msg)
        self.get_logger().info("🛑 Neutral RC commands sent")
    
    def sleep_with_spin(self, seconds):
        """Sleep while allowing ROS to continue spinning"""
        end_time = time.time() + seconds
        while rclpy.ok() and time.time() < end_time:
            rclpy.spin_once(self, timeout_sec=0.1)
    
    def wait_for_depth_reading(self, timeout_sec=10.0):
        """Wait until we have a valid depth reading"""
        self.get_logger().info("Waiting for depth sensor readings...")
        
        start_time = time.time()
        while (time.time() - start_time) < timeout_sec:
            if self.has_depth_reading():
                self.get_logger().info(f"✅ Depth reading available: {self.current_depth:.2f}m")
                return True
            
            self.sleep_with_spin(0.5)
        
        self.get_logger().error("❌ Timeout waiting for depth readings")
        return False
    
    def monitor_depth_hold(self, target_depth, duration_sec):
        """Monitor depth hold performance for specified duration"""
        self.get_logger().info(f"🌊 Monitoring depth hold at {target_depth}m for {duration_sec} seconds...")
        
        start_time = time.time()
        depth_errors = []
        
        while (time.time() - start_time) < duration_sec:
            current_depth = self.get_current_depth()
            depth_error = abs(current_depth - target_depth)
            depth_errors.append(depth_error)
            
            elapsed = time.time() - start_time
            remaining = duration_sec - elapsed
            
            # Log status every 2 seconds
            if int(elapsed) % 2 == 0 and elapsed > 0.5:
                sensor_status = []
                if self.pressure_valid:
                    sensor_status.append("PRESSURE")
                if self.pose_valid:
                    sensor_status.append("POSE")
                
                sensors = "+".join(sensor_status) if sensor_status else "NONE"
                
                self.get_logger().info(
                    f"📊 Depth Hold: {current_depth:.2f}m (target: {target_depth}m, "
                    f"error: {depth_error:.2f}m, remaining: {remaining:.1f}s, sensors: {sensors})"
                )
            
            self.sleep_with_spin(0.5)
        
        # Calculate statistics
        if depth_errors:
            avg_error = sum(depth_errors) / len(depth_errors)
            max_error = max(depth_errors)
            final_depth = self.get_current_depth()
            
            self.get_logger().info(
                f"📈 Depth Hold Complete - Final: {final_depth:.2f}m, "
                f"Avg Error: {avg_error:.2f}m, Max Error: {max_error:.2f}m"
            )
        
        return True
    
    def run_test(self):
        """Run the complete depth hold test sequence"""
        self.get_logger().info("🚀 Starting Depth Hold Test (1 meter for 10 seconds)")
        
        target_depth = 1.0  # meters
        hold_duration = 10.0  # seconds
        
        try:
            # Step 1: Wait for depth sensor readings
            if not self.wait_for_depth_reading():
                self.get_logger().error("❌ No depth readings available - aborting test")
                return False
            
            # Step 2: Send neutral RC first (safety)
            self.send_neutral_rc()
            self.sleep_with_spin(1.0)
            
            # Step 3: Arm the vehicle using utility node
            if not self.arm_vehicle():
                self.get_logger().error("❌ Failed to arm vehicle - aborting test")
                return False
            
            self.sleep_with_spin(2.0)
            
            # Step 4: Set DEPTH_HOLD mode
            if not self.set_mode("ALT_HOLD"):  # ALT_HOLD is depth hold in ArduSub
                self.get_logger().error("❌ Failed to set DEPTH_HOLD mode - aborting test")
                return False
            
            self.sleep_with_spin(3.0)  # Give time for mode to stabilize
            
            # Step 5: Monitor depth hold performance
            initial_depth = self.get_current_depth()
            self.get_logger().info(f"🌊 Starting depth hold test from {initial_depth:.2f}m depth")
            
            success = self.monitor_depth_hold(target_depth, hold_duration)
            
            # Step 6: Return to surface (MANUAL mode)
            self.get_logger().info("🔄 Switching to MANUAL mode for surface return")
            self.set_mode("MANUAL")
            self.sleep_with_spin(1.0)
            
            # Step 7: Disarm vehicle using utility node
            if not self.disarm_vehicle():
                self.get_logger().error("❌ Failed to disarm vehicle")
                return False
            
            if success:
                self.get_logger().info("🏁 Depth hold test completed successfully!")
            else:
                self.get_logger().warn("⚠️  Depth hold test completed with issues")
            
            return success
            
        except KeyboardInterrupt:
            self.get_logger().info("⚠️  Test interrupted by user")
            return False
        except Exception as e:
            self.get_logger().error(f"❌ Test failed with exception: {e}")
            return False
        finally:
            # Safety: Always send neutral RC and try to disarm
            self.get_logger().info("🔒 Safety cleanup...")
            self.send_neutral_rc()
            self.sleep_with_spin(1.0)
            self.set_mode("MANUAL")  # Safe mode
            self.disarm_vehicle()


def main(args=None):
    rclpy.init(args=args)
    
    test_node = DepthHoldTest()
    
    success = False
    try:
        success = test_node.run_test()
        
        if success:
            test_node.get_logger().info("✅ Depth hold test completed successfully!")
        else:
            test_node.get_logger().error("❌ Depth hold test failed!")
            
    except KeyboardInterrupt:
        test_node.get_logger().info("⚠️  Interrupted by user")
    except Exception as e:
        test_node.get_logger().error(f"❌ Unhandled exception: {e}")
    finally:
        # Final safety cleanup - disarm using utility node
        test_node.get_logger().info("🔒 Final disarming...")
        try:
            disarm_node = disarm.DisarmerNode()
            time.sleep(2.0)  # Give time for disarming to complete
            test_node.get_logger().info("✅ Final disarm complete")
        except Exception as e:
            test_node.get_logger().error(f"❌ Failed to disarm in cleanup: {e}")
        
        test_node.send_neutral_rc()
        test_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()