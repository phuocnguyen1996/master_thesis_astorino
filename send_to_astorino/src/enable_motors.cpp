// This node is used to enable motors
// "ros2 run send_to_astorino enable_motors true" -> enable
//"ros2 run send_to_astorino enable_motors something_else" -> disable
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"


using namespace std::chrono_literals;
class EnableMotors : public rclcpp::Node
{
    public:
        EnableMotors(bool enable): Node("enable_motors"), enable_(enable)
        {
            pub_ = this->create_publisher<std_msgs::msg::Bool>("enable_motors", 10);
            timer_ = this->create_wall_timer(
                500ms, std::bind(&EnableMotors::enable_call_back, this));
        }

    private:
        void enable_call_back() 
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
        rclcpp::spin(std::make_shared<EnableMotors>(true));
    else
        rclcpp::spin(std::make_shared<EnableMotors>(false));
    rclcpp::shutdown();
    return 0;
}
