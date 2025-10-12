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
target_include_directories(nav2_edge_aware_planner_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(nav2_edge_aware_planner_plugin
rclcpp rclcpp_lifecycle nav2_core nav2_costmap_2d pluginlib tf2_ros nav_msgs geometry_msgs nav2_util tf2_geometry_msgs)

add_executable(test_plugin src/test_plugin.cpp)
ament_target_dependencies(test_plugin
  pluginlib
  nav2_core
  rclcpp
)

# Install library
install(TARGETS nav2_edge_aware_planner_plugin
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

ament_export_libraries(nav2_edge_aware_planner_plugin)
ament_export_targets(export_${PROJECT_NAME})

install(TARGETS test_plugin
  DESTINATION lib/${PROJECT_NAME}
)

# Install plugin description xml
install(FILES edge_aware_planner_plugin.xml
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY launch params rviz
  DESTINATION share/${PROJECT_NAME}/
)

# Make plugin visible to nav2_core via pluginlib
pluginlib_export_plugin_description_file(nav2_core edge_aware_planner_plugin.xml)

ament_package()
