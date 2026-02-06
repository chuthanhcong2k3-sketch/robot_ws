import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    cfg_dir = os.path.join(
        get_package_share_directory("project_target_nav"),
        "config"
    )

    target_nav_node = Node(
        package="project_target_nav",
        executable="target_nav_node",
        name="target_nav_node",
        output="screen",
        parameters=[
            {"image_topic": "/camera_follow/image_raw"},
            {"scan_topic": "/scan"},
            {"cmd_topic": "/cmd_vel"},

            # tuning
            {"stop_dist": 1.5},
            {"stop_band": 0.05},
            {"kp_yaw": 1.3},
            {"search_wz": 0.6},
            {"fwd_vx": 0.25},

            # load lidar orientation config (cách 3)
            os.path.join(cfg_dir, "lidar_orientation.yaml"),
        ],
    )

    return LaunchDescription([target_nav_node])

