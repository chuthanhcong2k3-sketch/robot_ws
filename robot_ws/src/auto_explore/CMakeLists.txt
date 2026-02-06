cmake_minimum_required(VERSION 3.5)
project(auto_explore)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# ----- auto_explorer cũ -----
add_executable(auto_explorer src/auto_explorer.cpp)
ament_target_dependencies(auto_explorer
  rclcpp sensor_msgs geometry_msgs)

# ----- goto_goal mới -----
add_executable(goto_goal src/goto_goal.cpp)
ament_target_dependencies(goto_goal
  rclcpp sensor_msgs geometry_msgs
  tf2 tf2_ros tf2_geometry_msgs)

# ----- grid_nav (A* planner + follower + replanning) -----
add_executable(grid_nav src/grid_nav.cpp)
ament_target_dependencies(grid_nav
  rclcpp sensor_msgs geometry_msgs nav_msgs
  tf2 tf2_ros tf2_geometry_msgs)

install(TARGETS
  auto_explorer
  goto_goal
  grid_nav
  DESTINATION lib/${PROJECT_NAME})

# cài launch & rviz nếu có
install(DIRECTORY launch rviz config
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL)

ament_package()

