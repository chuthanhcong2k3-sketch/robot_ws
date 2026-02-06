#!/bin/bash
set -e

# ===== ROS ENV =====
source /opt/ros/humble/setup.bash
source /home/cong19102003/robot_ws/install/setup.bash

export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# ===== ROS BRIDGE =====
ros2 launch rosbridge_server rosbridge_websocket_launch.py &

sleep 2

# ===== MOTOR INTERFACE =====
ros2 launch motor_interface motor_interface.launch.py &

sleep 2

# ===== MODE MANAGER =====
cd /home/cong19102003/robot_ws/web_control
/usr/bin/python3 mode_manager.py &

# ===== GIU PROCESS =====
wait
