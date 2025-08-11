#!/usr/bin/env python3
"""
Simple Claw Open - GPIO Pin 7 HIGH
Just opens the claw, GPIO state stays HIGH until changed
"""
from auton_sub.claw.simple_claw_control import open_claw

def main():
    """Main function - just open the claw"""
    print("Opening submarine claw...")
    
    success = open_claw()
    
    if success:
        print("✅ Claw opened successfully - GPIO pin 7 is now HIGH")
        print("🔧 GPIO state will remain HIGH until explicitly changed")
    else:
        print("❌ Failed to open claw")
    
    return success

if __name__ == '__main__':
    import sys
    success = main()
    sys.exit(0 if success else 1)