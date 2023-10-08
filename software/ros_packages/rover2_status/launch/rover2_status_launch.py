from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config = {
        'emulate_tty': True,
        'output': 'screen',
        'respawn': True
    }

    return LaunchDescription([
        Node(
            package='rover2_status',
            executable='system_statuses_node',
            name='rover_status',
            **config
        )
    ])