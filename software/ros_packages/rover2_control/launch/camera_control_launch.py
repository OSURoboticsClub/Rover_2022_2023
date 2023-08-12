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
            package='rover2_control',
            executable='iris_controller',
            name='iris_controller',
            parameters=[{
                '~port': '/dev/rover/ttyIRIS',
                '~hertz': 30
            }],
            **config
        ),
        Node(
            package='rover2_control',
            executable='tower_and_pan_tilt_control',
            name='tower_and_pan_tilt',
            **config
        ),
        Node(
            package='rover2_control',
            executable='chassis_pan_tilt_control',
            name='chassis_pan_tilt',
            **config
        )
    ])
