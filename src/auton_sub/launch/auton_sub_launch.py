from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os
def generate_launch_description():
    # Get the path to your config file
    config_file = PathJoinSubstitution([
        FindPackageShare('auton_sub'),
        'configs',
        'mavros_params.yaml'
    ])

    return LaunchDescription([
        # No thruster node needed in GUIDED mode
        Node(
            package='mavros',
            executable='mavros_node',
            name='mavros',
            parameters=[config_file],
            output='screen'
        ),
        Node(
            package='auton_sub',
            executable='leak_node',
            name='leak_node'
        ),
        Node(
            package='auton_sub',
            executable='object_detection',
            name='object_detection',
            output='screen'
        ),
        Node(
            package='auton_sub',
            executable='mission_control',
            name='guided_mission_control'
        ),
        Node(
            package='auton_sub',
            executable='claw',
            name='claw'
        ),
        Node(
            package='auton_sub',
            executable='dvl_node',
            name='dvl_node',
            output='screen'
        ),
    ])