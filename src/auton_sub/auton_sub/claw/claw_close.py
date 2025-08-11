#!/usr/bin/env python3
"""
Simple Claw Close - GPIO Pin 7 LOW  
Just closes the claw, GPIO state stays LOW until changed
"""
from auton_sub.claw.simple_claw_control import close_claw

def main():
    """Main function - just close the claw"""
    print("Closing submarine claw...")
    
    success = close_claw()
    
    if success:
        print("✅ Claw closed successfully - GPIO pin 7 is now LOW")
        print("🔧 GPIO state will remain LOW until explicitly changed")
    else:
        print("❌ Failed to close claw")
    
    return success

if __name__ == '__main__':
    import sys
    success = main()
    sys.exit(0 if success else 1)