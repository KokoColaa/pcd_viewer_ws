cmake_minimum_required(VERSION 3.8)
project(pcd_viewer)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(yaml-cpp REQUIRED)

add_executable(pcd_viewer_node src/pcd_viewer_node.cpp)
ament_target_dependencies(
  pcd_viewer_node
  rclcpp
  sensor_msgs
  tf2_ros
  pcl_conversions
)

target_link_libraries(pcd_viewer_node
  yaml-cpp
)

install(TARGETS pcd_viewer_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

install(FILES
  rmul_2024.pcd
  DESTINATION share/${PROJECT_NAME}
)

ament_package()