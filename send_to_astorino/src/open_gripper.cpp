// This node is used to open or close the gripper
// "ros2 run send_to_astorino open_gripper true" -> open
// "ros2 run send_to_astorino open_gripper something_else" -> close
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"


using namespace std::chrono_literals;
class OpenGripper : public rclcpp::Node
{
    public:
        OpenGripper(bool enable): Node("open_gripper"), enable_(enable)
        {
            pub_ = this->create_publisher<std_msgs::msg::Bool>("open_gripper", 10);
            timer_ = this->create_wall_timer(
                500ms, std::bind(&OpenGripper::gripper_call_back, this));
        }

    private:
        void gripper_call_back() 
        {
            auto message = std_msgs::msg::Bool();
            message.data = enable_;

            // send once then shutdown the node
            pub_->publish(message);
            rclcpp::shutdown();
        }
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_;
        bool enable_;

};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    // Take the command line as inputs
    std::string input(argv[1]);
    if (input == "true")
        rclcpp::spin(std::make_shared<OpenGripper>(true));
    else
        rclcpp::spin(std::make_shared<OpenGripper>(false));
    rclcpp::shutdown();
    return 0;
}
