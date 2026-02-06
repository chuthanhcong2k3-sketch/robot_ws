import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    pkg_dir = get_package_share_directory("auto_explore")
    params_file = os.path.join(pkg_dir, "config", "mapper_params_online_sync.yaml")
    rviz_file = os.path.join(pkg_dir, "rviz", "explore.rviz")

    return LaunchDescription([
        Node(
            package="slam_toolbox",
            executable="sync_slam_toolbox_node",
            name="slam_toolbox",
            output="screen",
            parameters=[
                params_file
            ]
        ),
 # RViz auto load config
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d',
                rviz_file
            ]
        ),
    ])
