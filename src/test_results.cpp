#include <fstream>
#include <iostream>
#include <filesystem>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include "cartesian_impedance_control/cartesian_impedance_controller.hpp"
#include "cartesian_impedance_control/safety_bubble.hpp"

TestResult::TestResult() : Node("test_results"){
    hand_position_subscriber_ = this->create_subscription<std_msgs::msg::Float64MultiArray>("hand_position", 10,
    std::bind(&TestResult::new_pose_callback, this, std::placeholders::_1)); //hand position for Avoid

    franka_state_subscriber = this->create_subscription<franka_msgs::msg::FrankaRobotState>("franka_robot_state_broadcaster/robot_state", 10,
    std::bind(&TestResult::state_callback, this, std::placeholders::_1));

    goal_subscriber = this->create_subscription<std_msgs::msg::Float64MultiArray>("hand_goal_topic", 10,
        std::bind(&TestResult::goal_callback, this, std::placeholders::_1)); //Hand position for Follow
    
    jacobian_subscriber = this->create_subscription<std_msgs::msg::Float64MultiArray>("jacobian_topic", 10, 
    std::bind(&TestResult::jacobian_callback, this, std::placeholders::_1));

    timer_ = this->create_wall_timer( //defining duration of the test
        std::chrono::seconds(20), std::bind(&TestResult::timer_callback, this));

    timer_store_positions_ = this->create_wall_timer( //storing the position every 70 ms
        std::chrono::milliseconds(70), std::bind(&TestResult::store_positions, this));

    RCLCPP_INFO(this->get_logger(), "test_result constructor started.");
    }


void TestResult::state_callback(const franka_msgs::msg::FrankaRobotState::SharedPtr msg){
    if (inital_position_bool == false){
        inital_position_ = msg->o_t_ee;
        inital_joint_state_ = msg->measured_joint_state;
        inital_position_bool = true;
    }
    if (!initialized_){
        last_position_ = msg->o_t_ee;
        initialized_ = true;
    }
    double distance = sqrt(pow(msg->o_t_ee.pose.position.x - last_position_.pose.position.x, 2) + pow(msg->o_t_ee.pose.position.y-last_position_.pose.position.y, 2) + pow(msg->o_t_ee.pose.position.z - last_position_.pose.position.z, 2));
    total_distance += distance;
    last_position_ = msg->o_t_ee;
    current_position_ = msg->o_t_ee;
    current_joint_state_ = msg->measured_joint_state;
    save_to_csv();
}

void TestResult::timer_callback(){
    test_end = true;
}

void TestResult::store_positions(){
        path_x.push_back(current_position_.pose.position.x);
        path_y.push_back(current_position_.pose.position.y);
        path_z.push_back(current_position_.pose.position.z);
        path_ref.push_back(goal_pose[1]); //path of y-hand-position for follow
}

void TestResult::jacobian_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg){
    Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(msg->data.data());
}

void TestResult::new_pose_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg){
     for (size_t j = 0; j < 3; ++j) {
        hand_pose[j] = msg->data[j];
     }
}

void TestResult::goal_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg){
   for (size_t j = 0; j < 3; ++j) {
        goal_pose[j] = msg->data[j];
     }
}

bool TestResult::inside_check(geometry_msgs::msg::Point& position, std::array<double, 3>& hand_pose){
    //double distance = sqrt(pow(position.x-0.5,2)+pow(position.y-0,2)+pow(position.z-0.4,2)); //for Avoid: enter the hand position manually
    double distance = sqrt(pow(position.x-goal_pose[0],2)+pow(position.y-goal_pose[1],2)+pow(position.z-goal_pose[2],2)); //for follow
    if (inside){
        //RCLCPP_ERROR(this->get_logger(), "INSIDE.");
        if (distance < r_eq){
            inside_distance = r_eq - distance;
            if (inside_distance > max_inside_distance){
                max_inside_distance=inside_distance;
            }
        }
        return inside; //check whether the ee was inside before, if yes: bool should stay true
    } 
    else {
        if (distance < r_eq){
            inside_distance = r_eq - distance;
            if (inside_distance > max_inside_distance){
                max_inside_distance=inside_distance;
            }
        }
        return distance < r_eq;
    }
}

bool TestResult::outside_check(geometry_msgs::msg::Point& position, std::array<double, 3>& hand_pose){
    if (inside){
        return inside; //check whether outside before, if yes: bool should stay true
    } 
    else {
        double distance = sqrt(pow(position.x-inital_position_.pose.position.x,2)+pow(position.y-inital_position_.pose.position.y,2)+pow(position.z-inital_position_.pose.position.z,2));
        if (chosenPrimitive = 3){ //for hold
            return distance > r_eq;
        }
        if (chosenPrimitive = 2){ //for follow
            return distance > R;
        }
        else return 0; //should never occur (function is only called for hold and follow)
    }
}

bool TestResult::reached_goal_check(geometry_msgs::msg::Point& position, std::array<double, 3>& goal_pose){ //Tolerance of 0.01, enter the goal position by hand!!!
    if (abs(position.x- 0.4) < 0.01  and abs(position.y-0.4) < 0.01 and abs(position.z-0.4) < 0.01){
        return true;
    }
    else return false;
}

bool file_exists(const std::string& file_test_results){ //checks whether the file already exists
    std::ifstream file(file_test_results); 
    return file.good(); 
    }

void TestResult::save_to_csv(){
    auto position = current_position_.pose.position;
    auto inital_position = inital_position_.pose.position;
    std::ofstream csv_file;
    std::ofstream path_csv_file;

    Eigen::Map<const Eigen::Matrix<double, 7, 1>> inital_dq(inital_joint_state_.velocity.data());
    Eigen::Matrix<double, 6, 1> inital_velocity = jacobian * inital_dq;

    switch(chosenPrimitive){
        case 1:{ //AVOID
            inside = TestResult::inside_check(position, goal_pose); //calls function to check whether the ee is inside of r_eq
            std::string filename_avoid = "test_results_avoid.csv";
            if (test_end == true){
            bool file_already_exists_avoid = file_exists(filename_avoid); //calls function to check whether the csv file already exists
            goal_reached = TestResult::reached_goal_check(position, goal_pose);
            if (file_already_exists_avoid) {
                csv_file.open("test_results_avoid.csv", std::ios::app);
            } 
            else {
                csv_file.open("test_results_avoid.csv");
                csv_file << "Chosen Primitive, goal_x, goal_y, goal_z, inital_position_x, initial_position_y, inital_position_z, final_position_x, final_position_y, final_position_z, inital_velocity_x, inital_velocity_y, inital_velocity_z, R, r_eq, inside, inside_distance, goal_reached, total_distance" << "\n"; 
            }
            if (csv_file.is_open()){
                std::cout << R;
                csv_file << chosenPrimitive << "," << "0.4" << "," << "0.4" << "," << "0.4" << "," << inital_position.x << "," << inital_position.y
                << "," << inital_position.z << "," << position.x << "," << position.y << "," << position.z << "," << inital_velocity(0) << "," << inital_velocity(1) << "," << inital_velocity(2) << "," 
                << R << "," << r_eq << "," << inside << "," << max_inside_distance << "," << goal_reached << "," << total_distance << "\n"; 
                csv_file.close();
                RCLCPP_ERROR(this->get_logger(), "written to csv file.");
            }
            else{
                RCLCPP_ERROR(this->get_logger(), "Failed to open CSV file");
            }
            test_end = false;
            //saves the coordinates of the trajectory to a .csv file
            path_csv_file.open("path_avoid.csv");
            path_csv_file << "path_x, path_y, path_z" << "\n";
            for (unsigned int i = 0; i<path_x.size(); ++i){
                path_csv_file << path_x[i] << "," << path_y[i] << "," << path_z[i] << "\n";
            }
            }
            break;
            
        }
        case 2:{ //FOLLOW
            bool file_already_exists_follow = file_exists("test_results_follow.csv");
            inside = TestResult::inside_check(position, hand_pose);
           bool outside = TestResult::outside_check(position, hand_pose);
            if (test_end==true){
            if (file_already_exists_follow) {
                csv_file.open("test_results_follow.csv", std::ios::app);

            } 
            else {
                csv_file.open("test_results_follow.csv");
                csv_file << "Chosen Primitive, max_phase_x, max_phase_y, max_phase_z, inital_position_x, initial_position_y, inital_position_z, r_eq, inside, outside, total_distance \n"; 
            }
            if (csv_file.is_open()){
                csv_file << chosenPrimitive << "," << max_phase_x << "," << max_phase_y << "," << max_phase_z << "," << inital_position.x << "," << inital_position.y 
                << "," << inital_position.z << "," << r_eq << "," << inside << "," << outside << "," << total_distance << "\n"; 
                csv_file.close();
                RCLCPP_ERROR(this->get_logger(), "written to csv file.");

            }
            else{
                RCLCPP_ERROR(this->get_logger(), "Failed to open CSV file");
            }
            test_end = false;
            //saves the coordinates of the trajectory to a .csv file
            path_csv_file.open("path_follow.csv");
            path_csv_file << "path_x, path_y, path_z, path_ref" << "\n";
            for (unsigned int i = 0; i<path_x.size(); ++i){
                path_csv_file << path_x[i] << "," << path_y[i] << "," << path_z[i] << "," << path_ref[i] << "\n";
            }
            }
            break; 
        }
        case 3:{ //HOLD
            bool file_already_exists_hold = file_exists("test_results_hold.csv");
            inside = TestResult::outside_check(position, hand_pose); //eventough bool is called inside, it should still be true for the test to have the desired result (actually outside:) )
            if (test_end){
            if (file_already_exists_hold) {
                csv_file.open("test_results_hold.csv", std::ios::app);
            } 
            else {
                csv_file.open("test_results_hold.csv");
                csv_file << "Chosen Primitive, force_x, force_y, force_z, inital_position_x, initial_position_y, inital_position_z, final_pos_x, final_pos_y, final_pos_z, r_eq, outside, outside_distance \n"; 
            }
            if (csv_file.is_open()){
                csv_file << chosenPrimitive << "," << force_hold_x << "," << force_hold_y << "," << force_hold_z << "," << inital_position.x << "," << inital_position.y
                << "," << inital_position.z << "," << position.x << "," << position.y << "," << position.z <<  ","
                << r_eq << "," << inside << "," << "inside_distance" << "\n"; 
                csv_file.close();
            }
            else{
                RCLCPP_ERROR(this->get_logger(), "Failed to open CSV file");
            }
            test_end = false;
            //saves the coordinates of the trajectory to a .csv file
            path_csv_file.open("path_hold.csv");
            path_csv_file << "path_x, path_y, path_z" << "\n";
            for (unsigned int i = 0; i<path_x.size(); ++i){
                path_csv_file << path_x[i] << "," << path_y[i] << "," << path_z[i] << "\n";
            }
            }
            break;
        }
        default: { //used to see, how the trajectory of the end-effector is, when not primitive/safety bubble is acting
            //RCLCPP_ERROR(this->get_logger(), "no Primitive chosen.");
            inside = TestResult::inside_check(position, goal_pose); //calls function to check whether the ee is inside of r_eq
            std::string filename_check = "test_results_check.csv";
            if (test_end == true){
            bool file_already_exists_check = file_exists(filename_check); //calls function to check whether the csv file already exists
            goal_reached = TestResult::reached_goal_check(position, goal_pose);
            
            if (file_already_exists_check) {
                csv_file.open("test_results_check.csv", std::ios::app);
            } 
            else {
                csv_file.open("test_results_check.csv");
                csv_file << "Chosen Primitive, goal_x, goal_y, goal_z, inital_position_x, initial_position_y, inital_position_z, final_position_x, final_position_y, final_position_z, inital_velocity_x, inital_velocity_y, inital_velocity_z, goal_reached, total_distance" << "\n"; 
            }
            if (csv_file.is_open()){
                std::cout << R;
                csv_file << chosenPrimitive << "," << "0.4" << "," << "0.4" << "," << "0.4" << "," << inital_position.x << "," << inital_position.y
                << "," << inital_position.z << "," << position.x << "," << position.y << "," << position.z << "," << inital_velocity(0) << "," << inital_velocity(1) << "," << inital_velocity(2) << "," 
                << goal_reached << "," << total_distance << "\n"; 
                csv_file.close();
                RCLCPP_ERROR(this->get_logger(), "written to csv file.");
            }
            else{
                RCLCPP_ERROR(this->get_logger(), "Failed to open CSV file");
            }
            test_end = false;
            path_csv_file.open("path_check_hand in 0.5_2.csv");
            path_csv_file << "path_x, path_y, path_z" << "\n";
            for (unsigned int i = 0; i<300; ++i){
                path_csv_file << path_x[i] << "," << path_y[i] << "," << path_z[i] << "\n";
            }
            }
            break;
        }
    }
   
    }

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TestResult>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}