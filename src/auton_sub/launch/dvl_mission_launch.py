#!/usr/bin/env python3
"""
Launch file for DVL + MAVROS integration
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.actions import TimerAction
import os


def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('auton_sub')  # Replace with your package name
    
    # MAVROS parameters file
    mavros_params_file = os.path.join(pkg_dir, 'config', 'mavros_params.yaml')
    
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'fcu_url',
            default_value='serial:///dev/ttyTHS1:57600',
            description='FCU connection URL'
        ),
        
        DeclareLaunchArgument(
            'dvl_port',
            default_value='/dev/ttyUSB0',
            description='DVL serial port'
        ),
        
        # DVL Node
        Node(
            package='auton_sub',  # Replace with your package name
            executable='dvl_node',
            
            output='screen',
            parameters=[{
                'dvl_port': LaunchConfiguration('dvl_port'),
            }],
            remappings=[
                # Optional: remap topics if needed
            ]
        ),
        
        # DVL-MAVROS Bridge
        Node(
            package='auton_sub',  # Replace with your package name
            executable='dvl_mavros_bridge',
            
            output='screen'
        ),
        
        # MAVROS Node
        Node(
            package='mavros',
            executable='mavros_node',
            
            output='screen',
            parameters=[mavros_params_file, {
                'fcu_url': LaunchConfiguration('fcu_url'),
            }],
            remappings=[
                # Optional: remap MAVROS topics if needed
            ]
        ),
        Node(
            package='auton_sub',
            executable='front_multi',
            output='screen',
            parameters=[
                # Add any parameters for the detection node
            ]
        ),
        
        # Small delay to ensure detection node is ready
        TimerAction(
            period=3.0,
            actions=[
                # Coin toss mission node - starts after detection node is ready
                Node(
                    package='auton_sub',
                    executable='coin_toss',
                    output='screen',
                    parameters=[
                        # Add any parameters for the mission node
                    ]
                )
            ]
        ), 
    ])