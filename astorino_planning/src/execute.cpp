// This node is used to move the robot to a detected object, then pick and place it
// The execute_multiple node is the better version of this one
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <tf2/transform_datatypes.h>
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "std_msgs/msg/bool.hpp"
using std::placeholders::_1;
static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group_demo");
const double pi = 3.14159265358979323846;

class PositionToMove : public rclcpp::Node
{
public:
    PositionToMove(): Node("execute")
    {
        // Subcribe to the topic which has the pose of the object in robot frame
        sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
                "pose_to_move", 10, std::bind(&PositionToMove::call_back, this, _1));

        // Publish to open/close gripper
        pub_ = this->create_publisher<std_msgs::msg::Bool>("open_gripper", 10);
    }
private:
    void call_back(const geometry_msgs::msg::PoseStamped &msg)
    {
        // Whenever a msg comes, the pick-and-place process start
        if (count == 0)
        {
            rclcpp::NodeOptions node_options;
            node_options.automatically_declare_parameters_from_overrides(true);
            auto move_group_node = rclcpp::Node::make_shared("execute_node", node_options);
            rclcpp::executors::SingleThreadedExecutor executor;
            executor.add_node(move_group_node);
            std::thread([&executor]() { executor.spin(); }).detach();    

            static const std::string PLANNING_GROUP = "arm";

            moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);
            const moveit::core::JointModelGroup* joint_model_group =
            move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

            RCLCPP_INFO(LOGGER, "Planning frame: %s", move_group.getPlanningFrame().c_str());

           
            RCLCPP_INFO(LOGGER, "End effector link: %s", move_group.getEndEffectorLink().c_str());

            
            RCLCPP_INFO(LOGGER, "Available Planning Groups:");
            std::copy(move_group.getJointModelGroupNames().begin(), move_group.getJointModelGroupNames().end(),
                    std::ostream_iterator<std::string>(std::cout, ", "));

            // Continuously set, plan, and execute multiple target poses
            // Between each pose, the gripper may open or close
            // There was also a delay between poses, in order to wait for the real robot movement
            tf2::Quaternion q;
            geometry_msgs::msg::Pose target_pose1;

            // Pregrasp pose
            target_pose1.orientation.x = msg.pose.orientation.x;
            target_pose1.orientation.y = msg.pose.orientation.y;
            target_pose1.orientation.z = msg.pose.orientation.z;
            target_pose1.orientation.w = msg.pose.orientation.w;

            target_pose1.position.x = msg.pose.position.x;
            target_pose1.position.y = msg.pose.position.y - 0.025;
            target_pose1.position.z = msg.pose.position.z + 0.02;
            move_group.setPoseTarget(target_pose1);
            move_group.setPlanningTime(10.0);
            moveit::planning_interface::MoveGroupInterface::Plan my_plan;
            bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
            if(success) {
                move_group.execute(my_plan);
            } 
            else {
                RCLCPP_ERROR(LOGGER, "Planning failed!");
            }
            rclcpp::Rate sleep_rate(0.2);
            sleep_rate.sleep();
            std_msgs::msg::Bool gripper;
            gripper.data = true;
            pub_->publish(gripper);
            sleep_rate.sleep();

            // Grasp pose
            target_pose1.position.z = msg.pose.position.z - 0.01;
            move_group.setPoseTarget(target_pose1);
            move_group.setPlanningTime(10.0);
            moveit::planning_interface::MoveGroupInterface::Plan my_plan2;
            bool success2 = (move_group.plan(my_plan2) == moveit::core::MoveItErrorCode::SUCCESS);
            if(success2) {
                move_group.execute(my_plan2);
            } 
            else {
                RCLCPP_ERROR(LOGGER, "Planning failed!");
            }

            sleep_rate.sleep();
            gripper.data = false;
            pub_->publish(gripper);
            sleep_rate.sleep();

            // Move back to pregrasp pose before move to placement pose
            target_pose1.position.z = msg.pose.position.z + 0.05;
            move_group.setPoseTarget(target_pose1);
            move_group.setPlanningTime(10.0);
            moveit::planning_interface::MoveGroupInterface::Plan my_plan3;
            bool success3 = (move_group.plan(my_plan3) == moveit::core::MoveItErrorCode::SUCCESS);
            if(success3) {
                move_group.execute(my_plan3);
            } 
            else {
                RCLCPP_ERROR(LOGGER, "Planning failed!");
            }

            sleep_rate.sleep();

            // Pre-placement pose
            target_pose1.position.x = 0.354;
            target_pose1.position.y = -0.124;
            target_pose1.position.z = msg.pose.position.z + 0.15;
            move_group.setPoseTarget(target_pose1);
            move_group.setPlanningTime(10.0);

            moveit::planning_interface::MoveGroupInterface::Plan my_plan4;
            bool success4 = (move_group.plan(my_plan4) == moveit::core::MoveItErrorCode::SUCCESS);
            if(success4) {
                move_group.execute(my_plan4);
            } 
            else {
                RCLCPP_ERROR(LOGGER, "Planning failed!");
            }
            sleep_rate.sleep();


            // Placement pose
            target_pose1.position.z = msg.pose.position.z + 0.08;
            move_group.setPoseTarget(target_pose1);
            move_group.setPlanningTime(10.0);

            moveit::planning_interface::MoveGroupInterface::Plan my_plan5;
            bool success5 = (move_group.plan(my_plan5) == moveit::core::MoveItErrorCode::SUCCESS);
            if(success5) {
                move_group.execute(my_plan5);
            } 
            else {
                RCLCPP_ERROR(LOGGER, "Planning failed!");
            }
            sleep_rate.sleep();
            gripper.data = true;
            pub_->publish(gripper);
            sleep_rate.sleep();

            // Move up after finishing task
            target_pose1.position.z = msg.pose.position.z + 0.05;
            move_group.setPoseTarget(target_pose1);
            move_group.setPlanningTime(10.0);
            moveit::planning_interface::MoveGroupInterface::Plan my_plan6;
            bool success6 = (move_group.plan(my_plan6) == moveit::core::MoveItErrorCode::SUCCESS);
            if(success6) {
                move_group.execute(my_plan6);
            } 
            else {
                RCLCPP_ERROR(LOGGER, "Planning failed!");
            }
            sleep_rate.sleep();
            gripper.data = false;
            pub_->publish(gripper);
            sleep_rate.sleep();

            count++;
            rclcpp::shutdown();
        }
    }
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_;
    int count = 0;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PositionToMove>());
  rclcpp::shutdown();
  return 0;

}