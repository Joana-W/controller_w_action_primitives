//This file contains the position of the hand for testing purposes
#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/float32_multi_array.hpp"
#include <cartesian_impedance_control/safety_bubble.hpp>

TestInput::TestInput() : Node("test_input"){
    test_hand_position_publisher_ = this->create_publisher<messages_fr3::msg::SetPose>("hand_position", 10);

    hold_force_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("hold_test_force", 10);
    RCLCPP_INFO(this->get_logger(), "test_input constructor started.");
}

void TestInput::test_input(){
    double hand_position_x;
    double hand_position_y;
    double hand_position_z;
    switch(chosenPrimitive){
        case 1:{ //static hand position for AVOID
            hand_position_x = 0.0;
            hand_position_y = 0.0;
            hand_position_z = 0.0;
            break;
        }
        case 2:{ //dynamic hand position for FOLLOW, moves along the x-axis
            hand_position_z = 0.0;
            hand_position_y = 0.0;
            double hand_position_x_array[10] = {0.0,0.05,0.1,0.15,0.2,0.25,0.3,0.35,0.4,0.45};
            for (int i = 0; i<10; ++i){
                hand_position_x = hand_position_x_array[i];
                std::this_thread::sleep_for(std::chrono::seconds(1));
            }
            break;
        }
        case 3:{ //static hand position for HOLD
            hand_position_x = 0.0;
            hand_position_y = 0.0;
            hand_position_z = 0.0;
           // auto hand_position = std_msgs::msg::Float32MultiArray();
            //hand_position.data = {hand_position_x, hand_position_y, hand_position_z};
            double force_hold_x = 25;
            double force_hold_y = 25;
            double force_hold_z = 25;
            std::array<double, 6> force_hold = {force_hold_x, force_hold_y, force_hold_z, 0, 0,0};
            auto force_hold_msg = std_msgs::msg::Float64MultiArray();
            for(long unsigned int i = 0; i < force_hold.size(); ++i){
                force_hold_msg.data.push_back(force_hold[i]);
            }
            hold_force_publisher_ ->publish(force_hold_msg);
            break;
        }
        default:{ //default = static hand position
            hand_position_x = 0.0;
            hand_position_y = 0.0;
            hand_position_z = 0.0;
            //auto hand_position = std_msgs::msg::Float32MultiArray();
            //hand_position.data = {hand_position_x, hand_position_y, hand_position_z};
            break;
        }
    }
    
    auto hand_position = messages_fr3::msg::SetPose();
    //auto pose_request = std::make_shared<messages_fr3::msg::SetPose::Request>();
    hand_position.x = hand_position_x;
    hand_position.y = hand_position_y;
    hand_position.z = hand_position_z;
    hand_position.qx = 0;
    hand_position.qy = 0.0;
    hand_position.qz = 0;
    RCLCPP_INFO(this->get_logger(), "void test_input started.");

    test_hand_position_publisher_->publish(hand_position);
}


int main(int argc, char *argv[]){
    rclcpp::init(argc, argv);
    auto test_input_node = std::make_shared<TestInput>();
    rclcpp::spin(test_input_node);
    rclcpp::shutdown();
    return 0;
}