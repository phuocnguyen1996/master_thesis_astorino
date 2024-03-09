// This node is used to move the robot to a pose by inputing using comand line
// ros2 run astorino_planning move_command_line x y z quaternion.x quaternion.y quaternion.z quaternion.w
// The values of quaternion need to defined as more accurate as posible
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <tf2/transform_datatypes.h>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_command_line");
const double pi = 3.14159265358979323846;
int main(int argc, char** argv)
{
  // Initial setup, please read the moveit tutorial for more information
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto move_group_node = rclcpp::Node::make_shared("move_command_line", node_options);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(move_group_node);
  std::thread([&executor]() { executor.spin(); }).detach();    

  static const std::string PLANNING_GROUP = "arm";

  moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);
  const moveit::core::JointModelGroup* joint_model_group =
  move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  RCLCPP_INFO(LOGGER, "Planning frame: %s", move_group.getPlanningFrame().c_str());

  // We can also print the name of the end-effector link for this group.
  RCLCPP_INFO(LOGGER, "End effector link: %s", move_group.getEndEffectorLink().c_str());

  // We can get a list of all the groups in the robot:
  RCLCPP_INFO(LOGGER, "Available Planning Groups:");
  std::copy(move_group.getJointModelGroupNames().begin(), move_group.getJointModelGroupNames().end(),
          std::ostream_iterator<std::string>(std::cout, ", "));
 

  // Set up target pose
  tf2::Quaternion q;
  q.setRPY(0, pi, std::stod(argv[4]));
  geometry_msgs::msg::Pose target_pose1;

  // Convert command line input numerial values
  target_pose1.orientation.x = std::stod(argv[4]);
  target_pose1.orientation.y = std::stod(argv[5]);
  target_pose1.orientation.z = std::stod(argv[6]);
  target_pose1.orientation.w = std::stod(argv[7]);

  target_pose1.position.x = std::stod(argv[1]);
  target_pose1.position.y = std::stod(argv[2]);
  target_pose1.position.z = std::stod(argv[3]);

  // Planning
  move_group.setPoseTarget(target_pose1);
  move_group.setPlanningTime(10.0);
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

  // Execute
  if(success) {
      move_group.execute(my_plan);
  } 
  else {
      RCLCPP_ERROR(LOGGER, "Planning failed!");
  }

  rclcpp::shutdown();
  return 0;

}