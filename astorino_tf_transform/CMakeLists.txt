cmake_minimum_required(VERSION 3.8)
project(astorino_tf_transform)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(turtlesim REQUIRED)
find_package(message_filters REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)


if(TARGET tf2_geometry_msgs::tf2_geometry_msgs)
  get_target_property(_include_dirs tf2_geometry_msgs::tf2_geometry_msgs INTERFACE_INCLUDE_DIRECTORIES)
else()
  set(_include_dirs ${tf2_geometry_msgs_INCLUDE_DIRS})
endif()

find_file(TF2_CPP_HEADERS
  NAMES tf2_geometry_msgs.hpp
  PATHS ${_include_dirs}
  NO_CACHE
  PATH_SUFFIXES tf2_geometry_msgs
)

# add_executable(static_turtle_tf2_broadcaster src/static_turtle_tf2_broadcaster.cpp)
# ament_target_dependencies(
#    static_turtle_tf2_broadcaster
#    geometry_msgs
#    rclcpp
#    tf2
#    tf2_ros
# )

# add_executable(point_stamped_pub src/point_stamped_pub.cpp)
# ament_target_dependencies(
#    point_stamped_pub   
#    geometry_msgs
#    rclcpp
# )

# add_executable(point_stamped_to_world src/point_stamped_to_world.cpp)
# ament_target_dependencies(
#    point_stamped_to_world   
#    geometry_msgs
#    message_filters
#    rclcpp
#    tf2
#    tf2_geometry_msgs
#    tf2_ros
# )

# if(EXISTS ${TF2_CPP_HEADERS})
#   target_compile_definitions(point_stamped_to_world PUBLIC -DTF2_CPP_HEADERS)
# endif()


# add_executable(pose_stamped_pub src/pose_stamped_pub.cpp)
# ament_target_dependencies(
#    pose_stamped_pub  
#    geometry_msgs
#    rclcpp
#    tf2
# )

add_executable(pose_stamped_to_world src/pose_stamped_to_world.cpp)
ament_target_dependencies(
   pose_stamped_to_world
   geometry_msgs
   message_filters
   rclcpp
   tf2
   tf2_geometry_msgs
   tf2_ros
)

if(EXISTS ${TF2_CPP_HEADERS})
  target_compile_definitions(pose_stamped_to_world PUBLIC -DTF2_CPP_HEADERS)
endif()

add_executable(multiple_pose_stamped_to_world src/multiple_pose_stamped_to_world.cpp)
ament_target_dependencies(
   multiple_pose_stamped_to_world
   geometry_msgs
   message_filters
   rclcpp
   tf2
   tf2_geometry_msgs
   tf2_ros
)

if(EXISTS ${TF2_CPP_HEADERS})
  target_compile_definitions(multiple_pose_stamped_to_world PUBLIC -DTF2_CPP_HEADERS)
endif()




install(TARGETS
  #  static_turtle_tf2_broadcaster
  #  point_stamped_to_world
  #  point_stamped_pub
  #  pose_stamped_pub
   pose_stamped_to_world
   multiple_pose_stamped_to_world
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
