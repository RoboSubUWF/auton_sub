#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, LogInfo, GroupAction, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    # Package directory
    auton_sub_share = FindPackageShare('auton_sub')
    
    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', 
        default_value='false',
        description='Use simulation time'
    )
    
    auto_start_arg = DeclareLaunchArgument(
        'auto_start',
        default_value='true', 
        description='Auto start mission'
    )
    
    # ✅ CRITICAL FIX: Clear any existing MAVROS processes
    cleanup_mavros = ExecuteProcess(
        cmd=['pkill', '-f', 'mavros_node'],
        output='screen',
        on_exit=None  # Continue regardless of exit code
    )
    
    # DVL Node
    dvl_node = Node(
        package='auton_sub',
        executable='dvl_node',
        name='dvl_node',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }],
        # ✅ Add namespace to avoid conflicts
        namespace='sensors'
    )
    
    # DVL-MAVROS Bridge - Start after DVL
    dvl_mavros_bridge = TimerAction(
        period=3.0,  # Wait for DVL to initialize
        actions=[
            LogInfo(msg="🌉 Starting DVL-MAVROS Bridge..."),
            Node(
                package='auton_sub',
                executable='dvl_mavros_bridge', 
                name='dvl_mavros_bridge',
                output='screen',
                parameters=[{
                    'use_sim_time': LaunchConfiguration('use_sim_time')
                }]
            )
        ]
    )
    
    # ✅ MAVROS Node with explicit environment and delays
    mavros_node = TimerAction(
        period=5.0,  # Wait for DVL bridge to start
        actions=[
            LogInfo(msg="🚀 Starting MAVROS..."),
            Node(
                package='mavros',
                executable='mavros_node',
                name='mavros',
                output='screen',
                parameters=[
                    PathJoinSubstitution([
                        auton_sub_share,
                        'config',
                        'mavros_params.yaml'
                    ]),
                    {
                        'use_sim_time': LaunchConfiguration('use_sim_time')
                    }
                ],
                # ✅ CRITICAL: Set environment variables to fix memory issues
                additional_env={'RMW_IMPLEMENTATION': 'rmw_cyclone_dx'},
                remappings=[
                    # Avoid topic conflicts
                    ('~/local_position/pose', '/mavros/local_position/pose'),
                    ('~/setpoint_position/local', '/mavros/setpoint_position/local')
                ]
            )
        ]
    )
    
    # TF Static Transform (DVL to base_link)
    dvl_tf_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='dvl_tf_broadcaster',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'dvl_link'],
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )
    
    # Mission Control Node (delayed start) - Only after everything is ready
    mission_control_delayed = TimerAction(
        period=12.0,  # Wait longer for MAVROS to fully initialize
        actions=[
            LogInfo(msg="🚀 Starting Mission Control..."),
            Node(
                package='auton_sub',
                executable='mission_control',
                name='mission_control',
                output='screen',
                parameters=[{
                    'auto_start_delay': 15.0,  # Even longer delay for auto-start
                    'use_sim_time': LaunchConfiguration('use_sim_time')
                }],
                condition=IfCondition(LaunchConfiguration('auto_start'))
            )
        ]
    )

    return LaunchDescription([
        use_sim_time_arg,
        auto_start_arg,
        
        LogInfo(msg="🌊 Starting Autonomous Submarine System..."),
        
        # ✅ Step 1: Cleanup any existing MAVROS
        cleanup_mavros,
        
        # ✅ Step 2: Start DVL first
        TimerAction(
            period=1.0,  # Small delay after cleanup
            actions=[
                LogInfo(msg="📡 Starting DVL..."),
                dvl_node,
                dvl_tf_publisher
            ]
        ),
        
        # ✅ Step 3: Start DVL-MAVROS Bridge
        dvl_mavros_bridge,
        
        # ✅ Step 4: Start MAVROS
        mavros_node,
        
        # ✅ Step 5: Start Mission Control (optional)
        mission_control_delayed,
        
        LogInfo(msg="✅ Launch sequence initiated!"),
    ])