// This node is used to transform the pose of an object in camera_frame to base_frame
#include <chrono>
#include <memory>
#include <string>

#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "message_filters/subscriber.h"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/create_timer_ros.h"
#include "tf2_ros/message_filter.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/static_transform_broadcaster.h"
#ifdef TF2_CPP_HEADERS
  #include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#else
  #include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#endif

// using namespace std::chrono_literals;
using namespace std::chrono_literals;

class PoseDrawer : public rclcpp::Node
{
public:
  PoseDrawer()
  : Node("pose_stamped_to_world")
  {
    // To have a deep understanding, please visit the tutorial for tf2,
    // particularly, this code is based on the link below
    //  https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Using-Stamped-Datatypes-With-Tf2-Ros-MessageFilter.html?highlight=tf2_buffer_
    tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    
    // Create a Publisher to publish the pose of object 
    pub_pose = this->create_publisher<geometry_msgs::msg::PoseStamped>("pose_to_move", 10);
    // Declare and acquire "target_frame" parameter, which is the "base_link"
    target_frame_ = this->declare_parameter<std::string>("target_frame", "base_link");

    std::chrono::duration<int> buffer_timeout(1);

    tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    // Create the timer interface before call to waitForTransform,
    // to avoid a tf2_ros::CreateTimerInterfaceException exception
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
      this->get_node_base_interface(),
      this->get_node_timers_interface());
    tf2_buffer_->setCreateTimerInterface(timer_interface);
    tf2_listener_ =
      std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);

    // Create a Subscriber to subscribe to the topic, which contains the information
    // about the pose of the object in "camera_frame"
    point_sub_.subscribe(this, "/pose_stamped");
    tf2_filter_ = std::make_shared<tf2_ros::MessageFilter<geometry_msgs::msg::PoseStamped>>(
      point_sub_, *tf2_buffer_, target_frame_, 100, this->get_node_logging_interface(),
      this->get_node_clock_interface(), buffer_timeout);
    // Register a callback with tf2_ros::MessageFilter to be called when transforms are available
    tf2_filter_->registerCallback(&PoseDrawer::msgCallback, this);
  }

private:
  void msgCallback(const geometry_msgs::msg::PoseStamped::SharedPtr point_ptr)
  {
    // Calculating the pose of the object in "camera_frame" using QUATERNION
    geometry_msgs::msg::PoseStamped point_out;
    geometry_msgs::msg::TransformStamped t;
    tf2::Quaternion q;
    tf2::Quaternion q2;
    tf2::Quaternion q3;
    double roll, pitch, yaw;
    try {
      tf2_buffer_->transform(*point_ptr, point_out, target_frame_);
      q.setX(point_out.pose.orientation.x);
      q.setY(point_out.pose.orientation.y);
      q.setZ(point_out.pose.orientation.z);
      q.setW(point_out.pose.orientation.w);
      tf2::Matrix3x3 matrix(q);
      matrix.getEulerYPR(roll, pitch, yaw);
      
      // Using one more quaternion to rotate the frame associated with the object
      // to make its z-direction as same as tcp
      q2.setX(-0.7071);
      q2.setY(0);
      q2.setZ(0);
      q2.setW(0.7071);
      q3 = q * q2;
 

      RCLCPP_INFO(
        this->get_logger(), "Pose: x:%f y:%f z:%f orientX: %f orientY: %f orientZ: %f orientW: %f\n",
        point_out.pose.position.x,
        point_out.pose.position.y,
        point_out.pose.position.z,
        q3.getX(),
        q3.getY(),
        q3.getZ(),
        q3.getW()
        );
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(
        // Print exception which was caught
        this->get_logger(), "Failure %s\n", ex.what());
    }

    // Create a transpose stamped to send the frame to Rviz
    t.header = point_out.header;
    t.child_frame_id = "box1";
    t.transform.translation.x = point_out.pose.position.x;
    t.transform.translation.y = point_out.pose.position.y;
    t.transform.translation.z = point_out.pose.position.z;
    t.transform.rotation.x = q3.getX();
    t.transform.rotation.y = q3.getY();
    t.transform.rotation.z = q3.getZ();
    t.transform.rotation.w = q3.getW();
    tf_static_broadcaster_->sendTransform(t);

    // Modify the pose to make it have the same z-direction as the tcp
    point_out.pose.orientation.x = q3.getX();
    point_out.pose.orientation.y = q3.getY();
    point_out.pose.orientation.z = q3.getZ();
    point_out.pose.orientation.w = q3.getW();


    pub_pose->publish(point_out);

  }

  std::string target_frame_;
  std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
  message_filters::Subscriber<geometry_msgs::msg::PoseStamped> point_sub_;
  std::shared_ptr<tf2_ros::MessageFilter<geometry_msgs::msg::PoseStamped>> tf2_filter_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_pose;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PoseDrawer>());
  rclcpp::shutdown();
  return 0;
}