from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros2_examples',
            executable='simple_action_srv2',
            name='action_server'
        ),
        Node(
            package='ros2_examples',
            executable='simple_action_cli2',
            name='action_client'
        )
    ])
