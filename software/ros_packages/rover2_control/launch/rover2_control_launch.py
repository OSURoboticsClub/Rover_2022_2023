from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rover2_control',
            executable='drive_control',
            name='drive_control'
        )
    ])