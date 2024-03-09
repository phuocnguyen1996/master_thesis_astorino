// This node is used to automatically send target positions of joints to microcontroller
// The target positions are subscribed from the "joint_states" topic
// "ros2 run move_as_planning"
#include "memory"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
const double pi = 3.14159265358979323846;
using std::placeholders::_1;

class JointStateToFloat : public rclcpp::Node
{
    public:
        JointStateToFloat(): Node("move_as_planning")
        {
            pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("rotate_motors", 10);
            sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
                "joint_states", 10, std::bind(&JointStateToFloat::call_back, this, _1));
        }

    private:
        void call_back(const sensor_msgs::msg::JointState &msg) 
        {
            // Check for the changes in the joint_states, 
            // start the process when the changes are large enough
            if ((abs(prev_values[0] - msg.position[0]) > 0.005) || 
                (abs(prev_values[1] - msg.position[1]) > 0.005) || 
                (abs(prev_values[2] - msg.position[2]) > 0.005) || 
                (abs(prev_values[3] - msg.position[3]) > 0.005) ||
                (abs(prev_values[4] - msg.position[4]) > 0.005) ||
                (abs(prev_values[5] - msg.position[5]) > 0.005))
            {
                prev_values[0] = msg.position[0];
                prev_values[1] = msg.position[1];
                prev_values[2] = msg.position[2];
                prev_values[3] = msg.position[3];
                prev_values[4] = msg.position[4];
                prev_values[5] = msg.position[5];
                flag = true;
                count = 0;
            }
            else
                count++;

            if (flag)
                RCLCPP_INFO(this->get_logger(), "Preparing for sending");
            else
                RCLCPP_INFO(this->get_logger(), "Nothing changes for sending");

            // Only send when there is no change anymore
            if (flag && (count > 100))
            {
                // Convert radian to degree values
                // Offset with the differences between the "motor zero position" and "robot zero position"
                auto new_msg = std_msgs::msg::Float64MultiArray();
                
                // Careful about the order of joints in the joint_states topic, 
                // should echo the topic before use
                
                // Joint 5 and 6 used differential joints, which means both joints are based 
                // on the movement of both motors

                // In the astorino, when only motor 5 rotates, if the joint 5 rotates x degrees,
                // joint 6 will be driven -0.5x degrees 
                new_msg.data.push_back(msg.position[2]/pi * 180);
                new_msg.data.push_back(msg.position[0]/pi * 180 + 90);
                new_msg.data.push_back(msg.position[1]/pi * 180 - 160.3);
                new_msg.data.push_back(msg.position[3]/pi * 180);
                new_msg.data.push_back(msg.position[4]/pi * 180 + 90);
                new_msg.data.push_back(msg.position[5]/pi * 180 + msg.position[4]*0.5/pi * 180 + 45);
                pub_->publish(new_msg);
                flag = false;
                RCLCPP_INFO(this->get_logger(), "Command sent");
            }

            if (count > 150)
                count = 0;
        }
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_;
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_;
        double prev_values[6] = {0.0};
        int count = 0;
        bool flag = false;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JointStateToFloat>());
    rclcpp::shutdown();
    return 0;
}
