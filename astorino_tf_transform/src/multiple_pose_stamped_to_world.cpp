// This node is used to transform the pose of multiple objects in camera_frame to base_frame
#include <chrono>
#include <memory>
#include <string>

#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
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


using namespace std::chrono_literals;

class PoseDrawer : public rclcpp::Node
{
public:
  PoseDrawer()
  : Node("turtle_tf2_pose_drawer")
  {
    tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    
    // Create a Publisher to publish the pose of object
    pub_pose = this->create_publisher<geometry_msgs::msg::PoseArray>("pose_to_move", 10);

    // Declare and acquire `target_frame` parameter
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

    point_sub_.subscribe(this, "/pose_stamped");
    tf2_filter_ = std::make_shared<tf2_ros::MessageFilter<geometry_msgs::msg::PoseArray>>(
      point_sub_, *tf2_buffer_, target_frame_, 100, this->get_node_logging_interface(),
      this->get_node_clock_interface(), buffer_timeout);
    // Register a callback with tf2_ros::MessageFilter to be called when transforms are available
    tf2_filter_->registerCallback(&PoseDrawer::msgCallback, this);
  }

private:
  void msgCallback(const geometry_msgs::msg::PoseArray::SharedPtr point_ptr)
  {
    // Calculating the pose of objects in "camera_frame" using QUATERNION
    int size = point_ptr->poses.size();
    RCLCPP_INFO(this->get_logger(), "Number of objects: %i", size);
    geometry_msgs::msg::PoseArray multiple_poses;
    multiple_poses.poses.resize(3);
    for (int i = 0; i < size; i++)
    {
        
        geometry_msgs::msg::PoseStamped point_out;
        geometry_msgs::msg::TransformStamped t;
        tf2::Quaternion q;
        tf2::Quaternion q2;
        tf2::Quaternion q3;
        double roll, pitch, yaw;

        geometry_msgs::msg::PoseStamped single_ptr;
        single_ptr.header = point_ptr->header;
        single_ptr.pose = point_ptr->poses[i];

        try {
            tf2_buffer_->transform(single_ptr, point_out, target_frame_);
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

        // Create a transpose stamped to send the frames to Rviz
        t.header = point_out.header;
        t.child_frame_id = "box" + std::to_string(i);
        t.transform.translation.x = point_out.pose.position.x;
        t.transform.translation.y = point_out.pose.position.y;
        t.transform.translation.z = point_out.pose.position.z;
        t.transform.rotation.x = q3.getX();
        t.transform.rotation.y = q3.getY();
        t.transform.rotation.z = q3.getZ();
        t.transform.rotation.w = q3.getW();
        tf_static_broadcaster_->sendTransform(t);

        point_out.pose.orientation.x = q3.getX();
        point_out.pose.orientation.y = q3.getY();
        point_out.pose.orientation.z = q3.getZ();
        point_out.pose.orientation.w = q3.getW();

        point_out.pose.position.y = point_out.pose.position.y - 0.025;


        multiple_poses.header = point_out.header;
        multiple_poses.poses[i] = point_out.pose;

    }
    pub_pose->publish(multiple_poses);

  }

  std::string target_frame_;
  std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
  message_filters::Subscriber<geometry_msgs::msg::PoseArray> point_sub_;
  std::shared_ptr<tf2_ros::MessageFilter<geometry_msgs::msg::PoseArray>> tf2_filter_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pub_pose;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PoseDrawer>());
  rclcpp::shutdown();
  return 0;
}