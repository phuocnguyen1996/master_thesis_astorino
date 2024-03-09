// This node is used to check the movement of 6 joints
// Using: ros2 run send_to_astorino check_movement a b c d e f
// a b c d e f is the input for target positions of the joints, for example:
// ros2 run send_to_astorino check_movement 5 0 0 0 0 0

#include "memory"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

using namespace std::chrono_literals;

// Create a class for the node publisher 
class CheckMovement : public rclcpp::Node
{
    public:
        CheckMovement(double angle[]): Node("check_movement")
        {
            pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("rotate_motors", 10);

            for (int i = 0; i < 6; i++)
                angle_[i] = angle[i];
            
            timer_ = this->create_wall_timer(
                500ms, std::bind(&CheckMovement::check_movement_call_back, this));
        }



    private:
        void check_movement_call_back() 
        {
            //Create message to publish
            auto new_msg = std_msgs::msg::Float64MultiArray();
            new_msg.data.push_back(angle_[0]);
            new_msg.data.push_back(angle_[1]);
            new_msg.data.push_back(angle_[2]);
            new_msg.data.push_back(angle_[3]);
            new_msg.data.push_back(angle_[4]);
            new_msg.data.push_back(angle_[5]);

            // send once then shutdown the node
            pub_->publish(new_msg);
            rclcpp::shutdown();
        }
        rclcpp::TimerBase::SharedPtr timer_;
    
        double angle_[6]; //member contains 6 target values
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    // Take the command line as inputs
    double input[6] = {std::stod(argv[1]), std::stod(argv[2]),
        std::stod(argv[3]), std::stod(argv[4]), std::stod(argv[5]), std::stod(argv[6])};

    rclcpp::spin(std::make_shared<CheckMovement>(input));
    rclcpp::shutdown();
    return 0;
}
