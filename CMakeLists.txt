cmake_minimum_required(VERSION 3.8)
project(low_level_planner_executor)

# Use C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(custom_interfaces REQUIRED)

# Declare executable
add_executable(moveit_pose_action_server src/moveit_pose_action_server.cpp)
add_executable(low_level_planner_executor src/low_level_planner_executor.cpp)

# Link dependencies
ament_target_dependencies(moveit_pose_action_server
  rclcpp
  rclcpp_action
  geometry_msgs
  moveit_ros_planning_interface
  moveit_msgs
  tf2_geometry_msgs
  custom_interfaces
)
ament_target_dependencies(low_level_planner_executor
  rclcpp
  rclcpp_action
  geometry_msgs
  moveit_ros_planning_interface
  custom_interfaces
)

# Install executables
install(TARGETS
  moveit_pose_action_server
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS 
  low_level_planner_executor
  DESTINATION lib/${PROJECT_NAME}
)

# Export package
ament_package()
