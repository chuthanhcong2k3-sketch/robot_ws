from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    # Lấy đường dẫn đến folder share của package
    pkg_dir = get_package_share_directory('ydlidar_ros2_driver')

    # Dùng file params mặc định (bạn có thể đổi sang ydlidar_x3.yaml nếu muốn)
    params_file = os.path.join(pkg_dir, 'params', 'ydlidar.yaml')

    print("=== YDLIDAR LAUNCH ===")
    print("Using params file:", params_file)

    lidar_node = Node(
        package='ydlidar_ros2_driver',
        executable='ydlidar_ros2_driver_node',
        name='ydlidar_ros2_driver_node',
        output='screen',
        parameters=[params_file]
    )

    return LaunchDescription([
        lidar_node
    ])
