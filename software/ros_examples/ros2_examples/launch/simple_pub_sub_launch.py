from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros2_examples',
            executable='simple_sub2',
            name='subscriber'
        ),
        Node(
            package='ros2_examples',
            executable='simple_pub2',
            name='publisher'
        )
    ])
