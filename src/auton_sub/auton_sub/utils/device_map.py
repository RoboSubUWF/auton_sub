#!/usr/bin/env python3
# might be useful for later. Was taken from another team's github, but has not been updated. When the cameras are plugged into different ports or in a different order they register the paths to them diferently on the jetson.
import os
import sys
import json
import subprocess

# Set the config path for your Orin Nano submarine (these files do not yet exist. Add later if needed.)
CONFIG_PATH = "/home/robosub/ros2_ws/auton_sub/auton_sub/config/jetson_orin_sub.json"
USB_LINK_SCRIPT = "/home/robosub/ros2_ws/auton_sub/auton_sub/utils/usbLink.sh"


def load_config(path=CONFIG_PATH):
    with open(path, "r") as f:
        return json.load(f)


def run_usb_link():
    result = subprocess.run(["bash", USB_LINK_SCRIPT], capture_output=True, text=True)
    return result.stdout.splitlines()


def find_device_by_id(device_id):
    """Match a device ID from config to a /dev/tty or /dev/video path using usbLink.sh"""
    usb_list = run_usb_link()

    for line in usb_list:
        if "/dev/tty" in line or "/dev/video" in line:
            parts = line.split(" - ")
            if len(parts) == 2 and device_id in parts[1]:
                return parts[0]

    # Device not found
    print(f"Device ID '{device_id}' not found.")
    print("Available devices from usbLink.sh:")
    for line in usb_list:
        print(line)
    return None


def get_device_path(name, config):
    """Look up device path from config by name"""
    device_id = config.get(name)
    if not device_id:
        print(f"[ERROR] No device ID found in config for '{name}'")
        return None
    return find_device_by_id(device_id)


if __name__ == "__main__":
    config = load_config()

    # Use command line arg, default to pixhawk
    if len(sys.argv) > 1:
        name = sys.argv[1]
    else:
        name = "pixhawk"

    path = get_device_path(name, config)
    print(path if path else "Device not found.")
