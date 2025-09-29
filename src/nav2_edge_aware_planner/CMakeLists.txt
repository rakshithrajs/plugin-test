cmake_minimum_required(VERSION 3.8)
project(nav2_edge_aware_planner)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_util REQUIRED)

add_library(nav2_edge_aware_planner_plugin SHARED src/edge_aware_planner.cpp)
ament_target_dependencies(nav2_edge_aware_planner_plugin
  rclcpp rclcpp_lifecycle nav2_core nav2_costmap_2d pluginlib tf2_ros nav_msgs geometry_msgs nav2_util tf2_geometry_msgs)

# Install library
install(TARGETS nav2_edge_aware_planner_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

# Install plugin description xml
install(FILES plugin.xml
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY params
  DESTINATION share/${PROJECT_NAME}/
)

# Make plugin visible to nav2_core via pluginlib
pluginlib_export_plugin_description_file(nav2_core plugin.xml)

ament_package()
