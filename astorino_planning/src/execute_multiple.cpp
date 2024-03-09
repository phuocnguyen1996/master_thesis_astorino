// This node is used to continously pick and place detected objects
// Also visualize the objects in Rviz
// and use multiple executors to have a better performance
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>

#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <tf2/transform_datatypes.h>
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "std_msgs/msg/bool.hpp"

using std::placeholders::_1;
static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group_demo");
const double pi = 3.14159265358979323846;

// One executor to subcribe to the topic which has the poses of the objects in robot frame
class PositionToMove : public rclcpp::Node
{
public:
    PositionToMove(): Node("execute")
    {
        sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
                "pose_to_move", 10, std::bind(&PositionToMove::call_back, this, _1));
    }
    int count = 0;
    geometry_msgs::msg::PoseArray pose;

private:
    void call_back(const geometry_msgs::msg::PoseArray &msg)
    {
        pose = msg;
    }
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr sub_;
};


// One executor to subcribe to the topic which has the response signal from the real robot
// The response signal is true after the robot successfully execute a movement,
// for example move, open, close gripper
class ResponseSignal : public rclcpp::Node
{
public:
    ResponseSignal(): Node("execute_response")
    {
        sub_ = this->create_subscription<std_msgs::msg::Bool>(
                "response", 10, std::bind(&ResponseSignal::call_back, this, _1));
    }
    bool flag_finish = false;
private:
    void call_back(const std_msgs::msg::Bool &msg)
    {
        if (msg.data == true)
        {
            RCLCPP_INFO(this->get_logger(),"Checking True");
            flag_finish = true;
        }
        else
            RCLCPP_INFO(this->get_logger(),"Checking False");
    }
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_;
};

// One executor to publish to open/close gripper
class OpenGripper : public rclcpp::Node
{
public:
    OpenGripper(): Node("execute_gripper")
    {
        pub_ = this->create_publisher<std_msgs::msg::Bool>("open_gripper", 10);
    }
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_;
// private:
};


//Function to generate the trajectories to pick and place
geometry_msgs::msg::Pose* generate_steps(geometry_msgs::msg::Pose pose, int object_number);



int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
//   Three "nodes" are executing at the same time, which mean when the program is inside a callback,
// it does not block the other subcribers
  
  rclcpp::executors::MultiThreadedExecutor executor;
  auto node1 = std::make_shared<PositionToMove>();
  auto node2 = std::make_shared<ResponseSignal>();
  auto node3 = std::make_shared<OpenGripper>();

  //////////////////////////////////

  executor.add_node(node1);
  executor.add_node(node2);
  executor.add_node(node3);

// Initial setup, please read the moveit tutorial for more information
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto move_group_node = rclcpp::Node::make_shared("execute_node", node_options);
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






  ////Add objects in the visualization
  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools(move_group_node, "base_link", "visualization_marker_array",
                                                    move_group.getRobotModel());

  visual_tools.deleteAllMarkers();

  visual_tools.loadRemoteControl();
  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 1.0;
  visual_tools.publishText(text_pose, "Pick and place", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  moveit_msgs::msg::CollisionObject object_to_attach[node1->pose.poses.size()];
  shape_msgs::msg::SolidPrimitive box_primitive;
  box_primitive.type = box_primitive.BOX;
  box_primitive.dimensions.resize(3);
  box_primitive.dimensions[box_primitive.BOX_X] = 0.04;
  box_primitive.dimensions[box_primitive.BOX_Y] = 0.04;
  box_primitive.dimensions[box_primitive.BOX_Z] = 0.04;

  geometry_msgs::msg::Pose grab_pose[node1->pose.poses.size()];

  for (int i = 0; i < node1->pose.poses.size(); i++)
  {
    object_to_attach[i].id = "box" + std::to_string(i);

    object_to_attach[i].header.frame_id = "base_link";

    grab_pose[i] = node1->pose.poses[i];

    object_to_attach[i].primitives.push_back(box_primitive);
    object_to_attach[i].primitive_poses.push_back(grab_pose[i]);
    object_to_attach[i].operation = object_to_attach[i].ADD;
    planning_scene_interface.applyCollisionObject(object_to_attach[i]);
    RCLCPP_INFO(LOGGER, "Add object");
  }
  std::vector<std::string> touch_links;
  touch_links.push_back("tcp");
// ////


// Begin planning and executing
  for (int i = 0; i < node1->pose.poses.size(); i++)
  {

    geometry_msgs::msg::Pose* steps = generate_steps(grab_pose[i], i);

    // Use the my_plan function to generate target poses for picking and placing one object
    static moveit::planning_interface::MoveGroupInterface::Plan my_plan[6];
    for (int j = 0; j < 6; j++)
    {
        move_group.setPoseTarget(steps[j]);
        move_group.setPlanningTime(10.0);
                
        bool success = (move_group.plan(my_plan[j]) == moveit::core::MoveItErrorCode::SUCCESS);
        if(success) {
            move_group.execute(my_plan[j]);
        } 
        else {
            RCLCPP_ERROR(LOGGER, "Planning failed!");
        }

        // Waiting until the movement of the real robot is finished
        // Can delete for simulation or if using another method
        // Similar for the while(!node2->flag_finish below)
        while(!node2->flag_finish)
        {
        }
        node2->flag_finish = false;


        if (j == 1)
        {
            move_group.attachObject(object_to_attach[i].id, "tcp", touch_links);
            visual_tools.trigger();
            std_msgs::msg::Bool msg;
            msg.data = false;
            node3->pub_->publish(msg);
            while(!node2->flag_finish)
            {
            }
            node2->flag_finish = false;
        }
        if (j == 4)
        {
            move_group.detachObject(object_to_attach[i].id);
            visual_tools.trigger();
            std_msgs::msg::Bool msg;
            msg.data = true;
            node3->pub_->publish(msg);
            while(!node2->flag_finish)
            {
            }
            node2->flag_finish = false;
        }
    }
  }
  
  RCLCPP_INFO(node1->get_logger(),"Count is: %i", node1->count);
  rclcpp::shutdown();
  return 0;

}



// Generating 6 poses for picking and placing one object
geometry_msgs::msg::Pose* generate_steps(geometry_msgs::msg::Pose pose, int object_number)
{
    static geometry_msgs::msg::Pose step[6];

    // Pregrasp
    step[0].orientation.x = pose.orientation.x;
    step[0].orientation.y = pose.orientation.y;
    step[0].orientation.z = pose.orientation.z;
    step[0].orientation.w = pose.orientation.w;

    step[0].position.x = pose.position.x;
    step[0].position.y = pose.position.y;
    step[0].position.z = pose.position.z + 0.02;



    // Grasp
    step[1].orientation.x = pose.orientation.x;
    step[1].orientation.y = pose.orientation.y;
    step[1].orientation.z = pose.orientation.z;
    step[1].orientation.w = pose.orientation.w;

    step[1].position.x = pose.position.x;
    step[1].position.y = pose.position.y;
    step[1].position.z = pose.position.z;

    // Move up
    step[2].orientation.x = pose.orientation.x;
    step[2].orientation.y = pose.orientation.y;
    step[2].orientation.z = pose.orientation.z;
    step[2].orientation.w = pose.orientation.w;

    step[2].position.x = pose.position.x;
    step[2].position.y = pose.position.y;
    step[2].position.z = pose.position.z + 0.02;


    // Preplace
    step[3].orientation.x = pose.orientation.x;
    step[3].orientation.y = pose.orientation.y;
    step[3].orientation.z = pose.orientation.z;
    step[3].orientation.w = pose.orientation.w;

    step[3].position.x = 0.214 + 0.06*object_number;
    step[3].position.y = -0.2;
    step[3].position.z = 0.045 + 0.01;


    // Place
    step[4].orientation.x = pose.orientation.x;
    step[4].orientation.y = pose.orientation.y;
    step[4].orientation.z = pose.orientation.z;
    step[4].orientation.w = pose.orientation.w;

    step[4].position.x = 0.214 + 0.06*object_number;
    step[4].position.y = -0.2;
    step[4].position.z = 0.045;



    // Finish
    step[5].orientation.x = pose.orientation.x;
    step[5].orientation.y = pose.orientation.y;
    step[5].orientation.z = pose.orientation.z;
    step[5].orientation.w = pose.orientation.w;

    step[5].position.x = 0.214 + 0.06*object_number;
    step[5].position.y = -0.2 ;
    step[5].position.z = 0.045 + 0.15;


    return step;
}