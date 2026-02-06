from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        # ----- BASE_LINK → LASER (STATIC, GIỮ NGUYÊN) -----
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=[
                "-0.05", "0.0", "0.38",   # x y z
                "3.141592653589793", "0", "0",           # roll pitch yaw
                "base_link", "laser"
            ],
        ),
    ])
