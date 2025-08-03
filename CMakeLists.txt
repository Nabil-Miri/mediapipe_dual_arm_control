cmake_minimum_required(VERSION 3.8)
project(mediapipe_dual_arm_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_ros_move_group REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(moveit_servo REQUIRED)

# Add executables
add_executable(dual_arm_servo_node src/dual_arm_servo_node.cpp)

# Target dependencies

ament_target_dependencies(dual_arm_servo_node
  rclcpp
  std_msgs
  geometry_msgs
  trajectory_msgs
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_ros_move_group
  moveit_visual_tools
  tf2_ros
  tf2_geometry_msgs
  control_msgs
  sensor_msgs
  moveit_servo
)
# Install the servo dual arm pose tracking demo executable
install(TARGETS dual_arm_servo_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install config files
install(DIRECTORY config/robot/ DESTINATION share/${PROJECT_NAME}/config/robot)
install(DIRECTORY config/planning/ DESTINATION share/${PROJECT_NAME}/config/planning)
install(DIRECTORY config/pose_tracking/ DESTINATION share/${PROJECT_NAME}/config/pose_tracking)
install(DIRECTORY config/rviz/ DESTINATION share/${PROJECT_NAME}/config/rviz)
install(DIRECTORY config/hand/ DESTINATION share/${PROJECT_NAME}/config/hand)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
