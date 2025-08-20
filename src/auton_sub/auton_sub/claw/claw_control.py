#same as last
#!/usr/bin/env python3
"""
Simple Claw Control for Jetson Orin Nano
Just open and close functions - GPIO state persists between calls
"""
import Jetson.GPIO as GPIO
import time

# Pin Definitions
CLAW_PIN = 7  # GPIO pin 7 for claw control
_gpio_initialized = False

def _init_gpio():
    """Initialize GPIO if not already done"""
    global _gpio_initialized
    if not _gpio_initialized:
        try:
            GPIO.setmode(GPIO.BOARD)  # Use physical pin numbering
            GPIO.setup(CLAW_PIN, GPIO.OUT, initial=GPIO.LOW)
            _gpio_initialized = True
            print(f"✅ GPIO pin {CLAW_PIN} initialized")
        except Exception as e:
            print(f"❌ GPIO initialization failed: {e}")
            raise
    return _gpio_initialized

def open_claw():
    """
    Open the claw - GPIO pin goes HIGH and stays HIGH
    Returns True if successful
    """
    try:
        _init_gpio()
        GPIO.output(CLAW_PIN, GPIO.HIGH)
        print("🦀 Claw OPENED (GPIO pin 7 HIGH)")
        time.sleep(0.1)  # Brief delay for signal stability
        return True
    except Exception as e:
        print(f"❌ Failed to open claw: {e}")
        return False

def close_claw():
    """
    Close the claw - GPIO pin goes LOW and stays LOW
    Returns True if successful
    """
    try:
        _init_gpio()
        GPIO.output(CLAW_PIN, GPIO.LOW)
        print("🦀 Claw CLOSED (GPIO pin 7 LOW)")
        time.sleep(0.1)  # Brief delay for signal stability
        return True
    except Exception as e:
        print(f"❌ Failed to close claw: {e}")
        return False

def get_claw_state():
    """
    Get current GPIO pin state (not the actual claw position)
    Returns True if pin is HIGH (should be open), False if LOW (should be closed)
    """
    try:
        _init_gpio()
        return bool(GPIO.input(CLAW_PIN))
    except Exception as e:
        print(f"❌ Failed to read claw state: {e}")
        return False

def emergency_open():
    """Emergency claw open - always tries to open regardless of current state"""
    print("🚨 EMERGENCY CLAW OPEN!")
    return open_claw()

def cleanup_gpio():
    """Manual GPIO cleanup - call this if you need to clean up"""
    try:
        GPIO.cleanup()
        print("🧹 GPIO cleaned up")
        return True
    except Exception as e:
        print(f"⚠️ GPIO cleanup warning: {e}")
        return False

# Standalone script functionality
if __name__ == '__main__':
    import sys
    
    if len(sys.argv) > 1:
        command = sys.argv[1].lower()
        if command == 'open':
            success = open_claw()
        elif command == 'close':
            success = close_claw()
        elif command == 'state':
            state = get_claw_state()
            print(f"Claw state: {'OPEN' if state else 'CLOSED'}")
            success = True
        elif command == 'test':
            print("🧪 Testing claw control...")
            print("Opening claw...")
            open_claw()
            time.sleep(2)
            print("Closing claw...")
            close_claw()
            time.sleep(2)
            print("Opening claw again...")
            open_claw()
            print("✅ Test complete")
            success = True
        elif command == 'cleanup':
            success = cleanup_gpio()
        else:
            print("Usage: python claw_control.py [open|close|state|test|cleanup]")
            success = False
    else:
        print("Usage: python claw_control.py [open|close|state|test|cleanup]")
        success = False
    
    sys.exit(0 if success else 1)
