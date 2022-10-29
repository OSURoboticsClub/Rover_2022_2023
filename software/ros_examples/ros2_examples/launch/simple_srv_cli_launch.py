from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros2_examples',
            executable='simple_srv2',
            name='server'
        ),
        Node(
            package='ros2_examples',
            executable='simple_cli2',
            name='client'
        )
    ])
