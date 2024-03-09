cmake_minimum_required(VERSION 3.8)
project(astorino_planning)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(moveit_visual_tools REQUIRED)

add_executable(move_to_a_specific_position src/move_to_a_specific_position.cpp)
target_include_directories(move_to_a_specific_position PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(move_to_a_specific_position PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  move_to_a_specific_position
  "moveit_ros_planning_interface"
  "rclcpp"
  "tf2"
  "moveit_visual_tools"
)


add_executable(move_command_line src/move_command_line.cpp)
target_include_directories(move_command_line PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(move_command_line PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(
  move_command_line
  "moveit_ros_planning_interface"
  "rclcpp"
  "tf2"
  "moveit_visual_tools"
)

add_executable(execute src/execute.cpp)
target_include_directories(execute PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(execute PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(
  execute
  "moveit_ros_planning_interface"
  "rclcpp"
  "tf2"
  "moveit_visual_tools"
  "geometry_msgs"
)

add_executable(execute_multiple src/execute_multiple.cpp)
target_include_directories(execute_multiple PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(execute_multiple PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(
  execute_multiple
  "moveit_ros_planning_interface"
  "rclcpp"
  "tf2"
  "moveit_visual_tools"
  "geometry_msgs"
)


install(TARGETS 
  move_to_a_specific_position
  move_command_line
  execute
  execute_multiple
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
