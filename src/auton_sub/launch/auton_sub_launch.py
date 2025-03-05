import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='mavros',
            executable='mavros_node',
            name='mavros',
            parameters=['config/mavros_params.yaml']
        ),
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
            name='object_detection'
        ),
        launch_ros.actions.Node(
            package='auton_sub',
            executable='navigation',
            name='navigation'
        ),
    ])

