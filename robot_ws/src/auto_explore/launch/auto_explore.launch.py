from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    auto_explore_node = Node(
        package="auto_explore",
        executable="auto_explorer",
        name="auto_explorer",
        output="screen",
        parameters=[
            # Reduce overall speed ~1/2 for safer motion
            {"forward_speed": 0.10},
            {"turn_speed": 0.225},
            {"safe_dist": 2.0},
            {"side_angle": 60.0},

            # cách 3: chỉnh hướng "front" theo lidar
            # nếu front thực tế là phía sau -> 180.0
            {"front_angle_center_deg": 180.0},
        ],
        remappings=[
            ("scan", "/scan"),
            ("cmd_vel", "/cmd_vel"),
        ]
    )

    return LaunchDescription([auto_explore_node])
