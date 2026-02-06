from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='motor_interface',
            executable='motor_interface',
            parameters=[{
                'port': '/dev/stm32',

                # cmd_vel -> discrete STM32 commands uses duty-cycling
                # to approximate lower speeds safely.
                'lin_deadband': 0.02,
                'ang_deadband': 0.02,
                'max_lin_ref': 0.22,
                'max_ang_ref': 0.8,
            }],
            output='screen'
        )
    ])
