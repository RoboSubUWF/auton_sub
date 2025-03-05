import Jetson.GPIO as GPIO
import time

# Pin configuration
SENSOR_PIN = 12  # Replace with your GPIO pin number

# GPIO setup
GPIO.setmode(GPIO.BCM)  # Use BCM pin numbering
GPIO.setup(SENSOR_PIN, GPIO.IN)

print("Starting leak sensor monitoring. Press Ctrl+C to exit.")

try:
    while True:
        if GPIO.input(SENSOR_PIN) == GPIO.LOW:
            print("No leak detected.")
        else:
            print("Leak Detected!")
        time.sleep(1)  # Delay for stability
except KeyboardInterrupt:
    print("\nExiting...")
finally:
    GPIO.cleanup()  # Cleanup GPIO resources
