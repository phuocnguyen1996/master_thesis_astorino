cmake_minimum_required(VERSION 3.8)
project(send_to_astorino)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(move_as_planning src/move_as_planning.cpp)
ament_target_dependencies(move_as_planning rclcpp std_msgs sensor_msgs)
install(TARGETS
  move_as_planning
  DESTINATION lib/${PROJECT_NAME})

add_executable(enable_motors src/enable_motors.cpp)
ament_target_dependencies(enable_motors rclcpp std_msgs sensor_msgs)
install(TARGETS
  enable_motors
  DESTINATION lib/${PROJECT_NAME})

add_executable(check_movement src/check_movement.cpp)
ament_target_dependencies(check_movement rclcpp std_msgs sensor_msgs)
install(TARGETS
  check_movement
  DESTINATION lib/${PROJECT_NAME})

add_executable(open_gripper src/open_gripper.cpp)
ament_target_dependencies(open_gripper rclcpp std_msgs sensor_msgs)
install(TARGETS
  open_gripper
  DESTINATION lib/${PROJECT_NAME})

add_executable(move_to_zero_position src/move_to_zero_position.cpp)
ament_target_dependencies(move_to_zero_position rclcpp std_msgs sensor_msgs)
install(TARGETS
  move_to_zero_position
  DESTINATION lib/${PROJECT_NAME})

add_executable(move_to_turn_off_position src/move_to_turn_off_position.cpp)
ament_target_dependencies(move_to_turn_off_position rclcpp std_msgs sensor_msgs)
install(TARGETS
  move_to_turn_off_position
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
