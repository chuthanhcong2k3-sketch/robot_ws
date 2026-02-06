cmake_minimum_required(VERSION 3.5)
project(motor_interface)

# Tìm package ROS2 cần dùng
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Build node C++
add_executable(motor_interface src/motor_interface.cpp)

# Link dependencies
ament_target_dependencies(motor_interface
  rclcpp
  std_msgs
  geometry_msgs
)

# Install executable vào lib/
install(TARGETS motor_interface
  DESTINATION lib/${PROJECT_NAME})

# Install folder launch vào share/
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
