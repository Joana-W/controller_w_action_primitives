//This file contains the position of the hand for testing purposes
#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/float32_multi_array.hpp"
#include <cartesian_impedance_control/safety_bubble.hpp>

TestInput::TestInput() : Node("test_input"){
    test_hand_position_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("hand_position", 10);

    hold_force_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("hold_test_force", 10);

    hand_goal_publisher = this->create_publisher<std_msgs::msg::Float64MultiArray>("hand_goal_topic", 10);
    RCLCPP_INFO(this->get_logger(), "test_input constructor started.");
    test_input();
}

void TestInput::test_input(){
    double hand_position_x;
    double hand_position_y;
    double hand_position_z;
    switch(chosenPrimitive){
        case 1:{ //static hand position for AVOID
            std::this_thread::sleep_for(std::chrono::milliseconds(20)); //sleep so, that all other nodes get activated before publishing
            hand_position_x = 0.5;
            hand_position_y = 0.0;
            hand_position_z = 0.4;
            auto hand_position_vector_msg = std_msgs::msg::Float64MultiArray();
            double hand_position_vector_array[6] = {hand_position_x, hand_position_y, hand_position_z, 0.0, 0.0, 0.0};
            for(long unsigned int i = 0; i < 6; ++i){
                    hand_position_vector_msg.data.push_back(hand_position_vector_array[i]);
                }
            RCLCPP_INFO(this->get_logger(), "hand_pose prepublish: [%f, %f, %f, %f, %f, %f]",
                hand_position_vector_msg.data[0], hand_position_vector_msg.data[1], hand_position_vector_msg.data[2],
                hand_position_vector_msg.data[3], hand_position_vector_msg.data[4], hand_position_vector_msg.data[5]); 
            test_hand_position_publisher_->publish(hand_position_vector_msg);
            break;
        }
        case 2:{ //dynamic hand position for FOLLOW, moves along the y-axis
            hand_position_z = 0.3;
            hand_position_x = 0.3;
            double hand_position_y_array[10] = {0,0.1,0.2,0.3,0.4,0.3,0.2,0.1,0,0}; 
            for (int i = 0; i<10; ++i){
                std::this_thread::sleep_for(std::chrono::seconds(5)); //move every 5 seconds
                hand_position_y = hand_position_y_array[i];
                double hand_position_vector_array[6] = {hand_position_x, hand_position_y, hand_position_z, 0.0, 0.0, 0.0};
                auto hand_position_vector_msg = std_msgs::msg::Float64MultiArray();
                for(long unsigned int i = 0; i < 6; ++i){
                    hand_position_vector_msg.data.push_back(hand_position_vector_array[i]);
                }
                hand_goal_publisher ->publish(hand_position_vector_msg);
            }
            break;
        }
        case 3:{ //static hand position for HOLD and external test force
            std::array<double, 6> force_hold = {force_hold_x, force_hold_y, force_hold_z, 0, 0,0}; //array containing the external test force
            auto force_hold_msg = std_msgs::msg::Float64MultiArray();
            for(long unsigned int i = 0; i < force_hold.size(); ++i){
                force_hold_msg.data.push_back(force_hold[i]);
            }
            for (long unsigned int i = 0; i < 5;++i){ //publishing the force for 5 seconds
                hold_force_publisher_ ->publish(force_hold_msg);
                std::this_thread::sleep_for(std::chrono::seconds(1));
            }
            auto force_hold_null_msg = std_msgs::msg::Float64MultiArray();
            for(long unsigned int i = 0; i < force_hold.size(); ++i){
                force_hold[i] = 0;
                force_hold_null_msg.data.push_back(force_hold[i]);
            } 
            hold_force_publisher_ ->publish(force_hold_null_msg); //publishing an external force equal to zero
            break;
        }
        default:{ //default = no hand position
            break;
        }
    } 
}


int main(int argc, char *argv[]){
    rclcpp::init(argc, argv);
    auto test_input_node = std::make_shared<TestInput>();
    rclcpp::spin(test_input_node);
    rclcpp::shutdown();
    return 0;
}