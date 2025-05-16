cmake_minimum_required(VERSION 3.5)
project(lider_controller)

find_package(ament_cmake REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nav2_core REQUIRED)
find_package(pluginlib REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav2_costmap_2d REQUIRED)

include_directories(
  include
)

add_library(${PROJECT_NAME} SHARED
  src/lider_controller.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp_lifecycle
  nav2_core
  pluginlib
  geometry_msgs
  nav_msgs
  tf2_ros
  nav2_costmap_2d
)

pluginlib_export_plugin_description_file(nav2_core lider_controller_plugins.xml)

install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION lib
)
install(DIRECTORY include/
  DESTINATION include
)
install(FILES lider_controller_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()