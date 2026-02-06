cmake_minimum_required(VERSION 3.8)
project(project_target_nav)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(target_nav_node src/target_nav_node.cpp)
ament_target_dependencies(target_nav_node
  rclcpp
  sensor_msgs
  geometry_msgs
  cv_bridge
)

target_link_libraries(target_nav_node ${OpenCV_LIBS})

install(TARGETS target_nav_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
