#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction
import os

def generate_launch_description():
    return LaunchDescription([
        # USB Camera Node with corrected format
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam',
            output='screen',
            parameters=[{
                'video_device': '/dev/video0',
                'image_width': 800,
                'image_height': 600,
                'pixel_format': 'yuyv2rgb',  # Changed from mjpeg to yuyv (supported format)
                'camera_frame_id': 'usb_cam',
                'framerate': 15.0,
                'io_method': 'mmap',
                'camera_info_url': '',
                'camera_name': 'usb_cam',
                'brightness': -1,  # Auto
                'contrast': -1,    # Auto
                'saturation': -1,  # Auto
                'sharpness': -1,   # Auto
                'gain': -1,        # Auto
                'auto_white_balance': True,
                'white_balance': 4000
            }],
            remappings=[
                ('image_raw', '/camera/image_raw'),
                ('camera_info', '/camera/camera_info')
            ]
        ),
        
        # Delay the object detection node to let camera initialize
        TimerAction(
            period=3.0,  # Wait 3 seconds
            actions=[
                Node(
                    package='auton_sub',
                    executable='object_detection',
                    name='jetson_object_detection',
                    output='screen'
                )
            ]
        )
    ])