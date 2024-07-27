#include <fstream>
#include <iostream>
#include <filesystem>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include "cartesian_impedance_control/cartesian_impedance_controller.hpp"
#include "cartesian_impedance_control/safety_bubble.hpp"

TestResult::TestResult() : Node("test_results"){
    
    test_end_subscriber_ = this->create_subscription<std_msgs::msg::Bool>("test_end_bool", 10, 
    std::bind(&TestResult::test_end_callback, this, std::placeholders::_1));

    hand_position_subscriber_ = this->create_subscription<messages_fr3::msg::SetPose>("hand_position", 10,
    std::bind(&TestResult::new_pose_callback, this, std::placeholders::_1));
/*
    M_subscriber_ = this->create_subscription<std_msgs::msg::Float64MultiArray>("inertia_matrix_topic", 10,
        std::bind(&TestResult::inertia_callback, this, std::placeholders::_1));

    K_subscriber_ = this->create_subscription<std_msgs::msg::Float64MultiArray>("stiffness_matrix_topic", 10,
        std::bind(&TestResult::K_callback, this, std::placeholders::_1));

    D_subscriber_ = this->create_subscription<std_msgs::msg::Float64MultiArray>("damping_matrix_topic", 10,
        std::bind(&TestResult::D_callback, this, std::placeholders::_1));
    
    D_h_subscriber_ = this->create_subscription<std_msgs::msg::Float64MultiArray>("safety_damping_matrix_topic", 10,
        std::bind(&TestResult::D_h_callback, this, std::placeholders::_1));

    P_subscriber_ = this->create_subscription<std_msgs::msg::Float64MultiArray>("safety_stiffness_matrix_topic", 10,
        std::bind(&TestResult::P_callback, this, std::placeholders::_1));
*/
    franka_state_subscriber = this->create_subscription<franka_msgs::msg::FrankaRobotState>("franka_robot_state_broadcaster/robot_state", 10,
    std::bind(&TestResult::state_callback, this, std::placeholders::_1));

    goal_subscriber = this->create_subscription<messages_fr3::msg::SetPose>("original_pose", 10,
        std::bind(&TestResult::goal_callback, this, std::placeholders::_1));
    
    jacobian_subscriber = this->create_subscription<std_msgs::msg::Float64MultiArray>("jacobian_topic", 10, 
    std::bind(&TestResult::jacobian_callback, this, std::placeholders::_1));

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
        return;
    }
    double distance = sqrt(pow(msg->o_t_ee.pose.position.x - last_position_.pose.position.x, 2) + pow(msg->o_t_ee.pose.position.y-last_position_.pose.position.y, 2) + pow(msg->o_t_ee.pose.position.z - last_position_.pose.position.z, 2));
    total_distance += distance;
    current_position_ = msg->o_t_ee;
    current_joint_state_ = msg->measured_joint_state;
    save_to_csv();
}

void TestResult::store_positions(){
    while (!test_end){
        path_x.push_back(current_position_.pose.position.x);
        path_y.push_back(current_position_.pose.position.y);
        path_z.push_back(current_position_.pose.position.z);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
}

void TestResult::jacobian_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg){
    Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(msg->data.data());
}

void TestResult::new_pose_callback(const messages_fr3::msg::SetPose::SharedPtr msg){
    hand_pose = {msg->x, msg->y, msg->z};
}

void TestResult::goal_callback(const messages_fr3::msg::SetPose::SharedPtr msg){
    goal_pose = {msg->x, msg->y, msg->z};
}

void TestResult::test_end_callback(const std_msgs::msg::Bool::SharedPtr msg){
    if (msg->data && !previous_test_bool){
        test_end = true;
    }
    else test_end = false;
    previous_test_bool = msg->data;
    
} 

void TestResult::D_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg){
    Eigen::Map<Eigen::Matrix<double, 6, 6>> D(msg->data.data());
}

void TestResult::inertia_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg){
    Eigen::Map<Eigen::Matrix<double, 6, 6>> M(msg->data.data());
}

void TestResult::K_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg){
    Eigen::Map<Eigen::Matrix<double, 6, 6>> K(msg->data.data());
    }

void TestResult::P_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg){
    Eigen::Map<Eigen::Matrix<double, 6, 6>> P(msg->data.data());
}

void TestResult::D_h_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg){
    Eigen::Map<Eigen::Matrix<double, 6, 6>> D_h(msg->data.data());
}

bool TestResult::inside_check(geometry_msgs::msg::Point& position, std::array<double, 3>& hand_pose){
    if (inside){
        return inside; //check whether the ee was inside before, if yes: bool should stay true
    } 
    else {
        double distance = sqrt(pow(position.x-hand_pose[0],2)+pow(position.y-hand_pose[1],2)+pow(position.z-hand_pose[2],2));
        return distance < r_eq;
    }
}

bool TestResult::outside_check(geometry_msgs::msg::Point& position, std::array<double, 3>& hand_pose){
    if (inside){
        return inside; //check whether outside before, if yes: bool should stay true
    } 
    else {
        double distance = sqrt(pow(position.x-hand_pose[0],2)+pow(position.y-hand_pose[1],2)+pow(position.z-hand_pose[2],2));
        return distance > r_eq;
    }
}

bool TestResult::reached_goal_check(geometry_msgs::msg::Point& position, std::array<double, 3>& goal_pose){
    if (position.x == 0.4 and position.y == 0.4 and position.z == 0.4){
        return true;
    }
    else return false;
}

Eigen::Matrix<std::complex<double>, 6, 6> TestResult::Transfer_function(double omega, const Eigen::Matrix<double, 6, 6>& M, const Eigen::Matrix<double, 6, 6>& D,
    const Eigen::Matrix<double, 6, 6>& K){ //function, which calculates the transferfunction of the system
    std::complex<double> s(0, omega);
    Eigen::MatrixXcd H = (-omega * omega * M.cast<std::complex<double>>() + s*D.cast<std::complex<double>>() + K.cast<std::complex<double>>()).inverse();
    return H;
}

void TestResult::Frequency_Analysis(){
    std::vector<double> omega;
    for (double i = 0.1; i <= 62.83; i *= 1.1) {
        omega.push_back(i);
    }

    std::vector<double> phase_x, phase_y, phase_z;

    for (double w : omega) {
        Eigen::Matrix<std::complex<double>, 6, 6> H = TestResult::Transfer_function(w, M, D, K);
        
        std::complex<double> H_x = H(0, 0);
        std::complex<double> H_y = H(1, 1);
        std::complex<double> H_z = H(2, 2);

        phase_x.push_back(std::arg(H_x) * 180 / M_PI);
        
        phase_y.push_back(std::arg(H_y) * 180 / M_PI);
        
        phase_z.push_back(std::arg(H_z) * 180 / M_PI);
    }
    //find the max phase shift in each direction
    //max_phase_x = 0.0;
    for (long unsigned int i = 0; i < phase_x.size(); ++i) {
        max_phase_x = std::max(max_phase_x, std::abs(phase_x[i]));
    }
    //max_phase_y = 0.0;
    for (long unsigned int i = 0; i < phase_y.size(); ++i) {
        max_phase_y = std::max(max_phase_y, std::abs(phase_y[i]));
    }
    //double max_phase_z = 0.0;
    for (long unsigned int i = 0; i < phase_z.size(); ++i) {
        max_phase_z = std::max(max_phase_z, std::abs(phase_z[i]));
    }
}

bool file_exists(const std::string& file_test_results){
    std::ifstream file(file_test_results); 
    return file.good(); 
    }

void TestResult::save_to_csv(){
    auto position = current_position_.pose.position;
    auto inital_position = inital_position_.pose.position;
    std::ofstream csv_file;

    Eigen::Map<const Eigen::Matrix<double, 7, 1>> inital_dq(inital_joint_state_.velocity.data());
   // std::array<double, 42> jacobian_array =  franka_robot_model_->getZeroJacobian(franka::Frame::kEndEffector); //get Jacobian as an array
   // Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data()); //convert the jacobian array into Jacobian matrix
    Eigen::Matrix<double, 6, 1> inital_velocity = jacobian * inital_dq;
    //RCLCPP_INFO(this->get_logger(), "save to csv started.");

    switch(chosenPrimitive){
        case 1:{ //AVOID
            inside = TestResult::inside_check(position, goal_pose); //calls function to check whether the ee is inside of r_eq

            std::string filename_avoid = "test_results_avoid.csv";
            if (test_end == true){
            bool file_already_exists_avoid = file_exists(filename_avoid); //calls function to check whether the csv file already exists
            std::cout << "the size of path_x is ", path_x.size();
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
                << R << "," << r_eq << "," << inside << "," << "inside_distance" << "," << goal_reached << "," << total_distance << "\n"; 
                csv_file.close();
                RCLCPP_INFO(this->get_logger(), "written to csv file.");
                std::cout << "path_x =";
                for (long unsigned int i = 0;i<path_x.size();++i){
                     std::cout << path_x[i];
                }
            }
            else{
                RCLCPP_ERROR(this->get_logger(), "Failed to open CSV file");
            }
            }
            break;
            
        }
        case 2:{ //FOLLOW
            bool file_already_exists_follow = file_exists("test_results_follow.csv");
            inside = TestResult::inside_check(position, hand_pose);
            TestResult::Frequency_Analysis();
            if (test_end==true){
            if (file_already_exists_follow) {
                csv_file.open("test_results_follow.csv", std::ios::app);

            } 
            else {
                csv_file.open("test_results_follow.csv");
                csv_file << "Chosen Primitive, max_phase_x, max_phase_y, max_phase_z, inital_position_x, initial_position_y, inital_position_z, inital_velocity_x, inital_velocity_y, inital_velocity_z, R, r_eq, outside"; 
            }
            if (csv_file.is_open()){
                csv_file << chosenPrimitive << "," << max_phase_x << "," << max_phase_y << "," << max_phase_z << "," << inital_position.x << "," << inital_position.y 
                << "," << inital_velocity(0) << "," << inital_velocity(1) << "," << inital_velocity(2)
                << "," << inital_position.z << ","
                << R << "," << r_eq << "," << inside << "," << "\n"; 
                //bei Follow hier noch den Parameter für max phase shift einfügen
               csv_file.close();
            }
            else{
                RCLCPP_ERROR(this->get_logger(), "Failed to open CSV file");
            }
            }
            break; 
        }
        case 3:{ //HOLD
            
            bool file_already_exists_hold = file_exists("test_results_hold.csv");
            inside = TestResult::outside_check(position, hand_pose); //eventough bool is called inside, it should still be true for the test to have the desired result
            if (test_end){
            if (file_already_exists_hold) {
                csv_file.open("test_results_hold.csv", std::ios::app);
            } 
            else {
                csv_file.open("test_results_hold.csv");
                csv_file << "Chosen Primitive, force_x, force_y, force_z, inital_position_x, initial_position_y, inital_position_z, final_pos_x, final_pos_y, final_pos_z, r_eq, outside, outside_distance, path_x "; 
            }
            if (csv_file.is_open()){
                csv_file << chosenPrimitive << "," << force_hold_x << "," << force_hold_y << "," << force_hold_z << "," << inital_position.x << "," << inital_position.y
                << "," << inital_position.z << "," << position.x << "," << position.y << "," << position.z <<  ","
                << r_eq << "," << inside << "," << "inside_distance" << ","  << "\n"; 
                csv_file.close();
            }
            else{
                RCLCPP_ERROR(this->get_logger(), "Failed to open CSV file");
            }
            }
            break;
        }
        default: {
            RCLCPP_ERROR(this->get_logger(), "no Primitive Chosen, Test Results are not being saved.");
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