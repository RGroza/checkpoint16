from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='wheel_velocities_publisher',
            executable='wheel_velocities_publisher',
            name='wheel_velocities_publisher',
            output='screen'
        ),
        Node(
            package='kinematic_model',
            executable='kinematic_model',
            name='kinematic_model',
            output='screen'
        )
    ])
