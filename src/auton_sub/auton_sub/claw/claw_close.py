#was unable to successfully test since the claw motor broke
#!/usr/bin/env python3
"""
Simple Claw Close - GPIO Pin 7 LOW  
Just closes the claw, GPIO state stays LOW until changed
"""
import sys
import os

def main():
    """Main function - just close the claw"""
    try:
        from auton_sub.claw.claw_control import close_claw
        
        print("Closing submarine claw...")
        
        success = close_claw()
        
        if success:
            print("✅ Claw closed successfully - GPIO pin 7 is now LOW")
            print("🔧 GPIO state will remain LOW until explicitly changed")
            return True
        else:
            print("❌ Failed to close claw")
            return False
            
    except Exception as e:
        print(f"❌ Error in claw_close: {e}")
        return False

if __name__ == '__main__':
    success = main()
    
    if success:
        print("Debug: Operation successful, exiting cleanly")
        # Use os._exit to bypass atexit handlers that are causing issues
        os._exit(0)
    else:
        print("Debug: Operation failed")
        os._exit(1)
