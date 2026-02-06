from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="v4l2_camera",
            executable="v4l2_camera_node",
            name="follow_camera",
            output="screen",
            parameters=[
                {"video_device": "/dev/camera_follow"},
                {"pixel_format": "YUYV"},
                {"image_width": 640},
                {"image_height": 480},
                {"output_encoding": "bgr8"},
            ],
            remappings=[
                ("image_raw", "/camera_follow/image_raw"),
            ],
        )
    ])
