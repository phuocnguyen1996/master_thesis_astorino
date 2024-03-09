// This node is used to automatically send zero positions of robot to microcontroller
#include "memory"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

using namespace std::chrono_literals;
class MoveToZeroPosition : public rclcpp::Node
{
    public:
        MoveToZeroPosition(): Node("move_to_zero_position")
        {
            pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("rotate_motors", 10);
            timer_ = this->create_wall_timer(
                500ms, std::bind(&MoveToZeroPosition::move_to_zero_call_back, this));
        }



    private:
        void move_to_zero_call_back() 
        {
            // The zero position of robot is the position when robot is fully stretched out upward
            auto new_msg = std_msgs::msg::Float64MultiArray();
            new_msg.data.push_back(0);
            new_msg.data.push_back(90.0);
            new_msg.data.push_back(-160.3);
            new_msg.data.push_back(0);
            new_msg.data.push_back(90);
            new_msg.data.push_back(45);

            // send once then shutdown the node
            pub_->publish(new_msg);
            rclcpp::shutdown();
        }
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<MoveToZeroPosition>());
    rclcpp::shutdown();
    return 0;
}
