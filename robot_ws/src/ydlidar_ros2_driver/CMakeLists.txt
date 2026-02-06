cmake_minimum_required(VERSION 3.5)
project(ydlidar_ros2_driver)

# ============================================================
# C++ Standard
# ============================================================
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ============================================================
# Dependencies
# ============================================================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(ydlidar_sdk REQUIRED)

# ============================================================
# Include folders
# ============================================================
include_directories(
  ${PROJECT_SOURCE_DIR}/src
  ${YDLIDAR_SDK_INCLUDE_DIRS}
)

link_directories(
  ${YDLIDAR_SDK_LIBRARY_DIRS}
)

# ============================================================
# Build node
# ============================================================
add_executable(ydlidar_ros2_driver_node
  src/ydlidar_ros2_driver_node.cpp
)

ament_target_dependencies(ydlidar_ros2_driver_node
  rclcpp
  sensor_msgs
  visualization_msgs
  geometry_msgs
  std_srvs
)

target_link_libraries(ydlidar_ros2_driver_node
  ${YDLIDAR_SDK_LIBRARIES}
)

# ============================================================
# Build client
# ============================================================
add_executable(ydlidar_ros2_driver_client
  src/ydlidar_ros2_driver_client.cpp
)

ament_target_dependencies(ydlidar_ros2_driver_client
  rclcpp
  sensor_msgs
  visualization_msgs
  geometry_msgs
  std_srvs
)

# ============================================================
# Install targets + launch/config folders
# ============================================================
install(TARGETS
  ydlidar_ros2_driver_node
  ydlidar_ros2_driver_client
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch params startup config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
