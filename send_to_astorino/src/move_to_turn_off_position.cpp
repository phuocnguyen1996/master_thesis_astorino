// This node is used to automatically send turn-off positions of joints to microcontroller
#include "memory"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

using namespace std::chrono_literals;
class MoveToTurnOffPosition : public rclcpp::Node
{
    public:
        MoveToTurnOffPosition(): Node("move_to_turn_off_position")
        {
            pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("rotate_motors", 10);
            timer_ = this->create_wall_timer(
                500ms, std::bind(&MoveToTurnOffPosition::move_to_turn_off_call_back, this));
        }



    private:
        void move_to_turn_off_call_back() 
        {
            //The turn-off positions of joints are their zero values,
            // which is the position of the robot when it is turned off normally
            auto new_msg = std_msgs::msg::Float64MultiArray();
            new_msg.data.push_back(0);
            new_msg.data.push_back(0);
            new_msg.data.push_back(0);
            new_msg.data.push_back(0);
            new_msg.data.push_back(0);
            new_msg.data.push_back(0);

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

    rclcpp::spin(std::make_shared<MoveToTurnOffPosition>());
    rclcpp::shutdown();
    return 0;
}
