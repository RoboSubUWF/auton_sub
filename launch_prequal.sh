#!/bin/bash
sleep 6
cd /home/robosub/ros2_ws

# Set up log file

timestamp=$(date +"%Y-%m-%d_%H-%M-%S")
exec > /home/robosub/logs/log$(date +'%Y-%m-%d%H-%M-%S').txt 2>&1

echo "Starting ROS 2 launch script..."

# Source ROS 2 and workspace
source /opt/ros/humble/setup.bash
source /home/robosub/ros2_ws/install/setup.bash

# Print ROS environment to log
env | grep ROS

# Launch
ros2 launch auton_sub prequal_launch.py

