import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='auton_sub',
            executable='thruster_control',
            name='thruster_control'
        ),
        launch_ros.actions.Node(
            package='auton_sub',
            executable='leak_sensor',
            name='leak_sensor'
        ),
        launch_ros.actions.Node(
            package='auton_sub',
            executable='object_detection',
            name='object_detection',
            output='screen'
        ),
        launch_ros.actions.Node(
            package='auton_sub',
            executable='gopro',
            name='gopro',
            output='screen'
        ),
        launch_ros.actions.Node(
            package='auton_sub',
            executable='navigation',
            name='navigation'
        ),
        launch_ros.actions.Node(
            package='auton_sub',
            executable='thruster_node',
            name='thruster_node'
        ),
        launch_ros.actions.Node(
            package='auton_sub',
            executable='leak_node',
            name='leak_node'
        ),
        launch_ros.actions.Node(
            package='auton_sub',
            executable='mission_control',
            name='mission_control'
        ),
        launch_ros.actions.Node(
            package='auton_sub',
            executable='claw',
            name='claw'
        ),
    ])
