from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="auto_explore",
            executable="grid_nav",
            name="grid_nav",
            output="screen",
            parameters=[{
                # topics
                "map_topic": "/map",
                "goal_topic": "/goal_pose",
                "scan_topic": "/scan",
                "cmd_topic": "/cmd_vel",
                "plan_topic": "/plan",

                # frames
                "map_frame": "map",
                "base_frame": "base_link",

                # planner
                "occupied_thresh": 50,
                "unknown_is_obstacle": True,
                "inflation_radius_m": 0.18,

                # replanning / control
                "plan_rate_hz": 2.0,
                "control_rate_hz": 20.0,
                # Slightly larger lookahead reduces jitter and helps produce a
                # forward command that clears the motor-interface deadband.
                "lookahead_m": 0.55,
                "goal_tol_m": 0.12,
                # Reduce overall speed ~1/2
                "max_lin": 0.11,
                "max_ang": 0.4,
                "k_lin": 0.9,
                "k_ang": 2.0,
                "angle_stop_rad": 0.55,

                # obstacle handling
                "use_scan_obstacles": True,
                "scan_obstacle_hold_sec": 2.0,
                # Safety in front when going to B (meters)
                "safe_dist_front": 2.0,
                "front_half_angle_deg": 18.0,
            }],
            remappings=[
                ("scan", "/scan"),
                ("cmd_vel", "/cmd_vel"),
                ("map", "/map"),
                ("goal_pose", "/goal_pose"),
                ("plan", "/plan"),
            ]
        )
    ])
