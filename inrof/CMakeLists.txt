cmake_minimum_required(VERSION 3.5)
project(inrof)

# 必要な依存パッケージ
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

# インクルードディレクトリ
include_directories(include)

# cmd_vel_publisherノード
add_executable(cmd_vel_publisher src/cmd_vel_publisher.cpp)
ament_target_dependencies(cmd_vel_publisher rclcpp geometry_msgs)

# auto_driveノード
add_executable(auto_drive src/inrof_auto_drive.cpp)
ament_target_dependencies(auto_drive rclcpp geometry_msgs)

# twist_markerノード
add_executable(twist_marker src/twist_marker.cpp)
ament_target_dependencies(twist_marker rclcpp geometry_msgs visualization_msgs)

# インストール設定
install(TARGETS
  cmd_vel_publisher
  auto_drive
  twist_marker
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()