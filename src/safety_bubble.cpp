#include <rclcpp/rclcpp.hpp>
#include "cartesian_impedance_control/safety_bubble.hpp"
#include "cartesian_impedance_control/cartesian_impedance_controller.hpp"
#include <Eigen/Dense>
#include <Eigen/Eigen>
#include <unsupported/Eigen/MatrixFunctions>
#include <cmath>
#include <memory>
#include <functional>

#include <franka/model.h>
#include <franka/robot.h>
#include <franka/robot_state.h>

SafetyBubble::SafetyBubble() : Node("safety_bubble"){
    f_ext_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("external_force_topic", 10);

    hold_stiffness_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("hold_stiffness", 10);

    test_hand_position_subscriber_ = this->create_subscription<std_msgs::msg::Float64MultiArray>("hand_position", 10,
    std::bind(&SafetyBubble::hand_position_callback, this, std::placeholders::_1));

    franka_state_subscriber = this->create_subscription<franka_msgs::msg::FrankaRobotState>("franka_robot_state_broadcaster/robot_state", 10,
    std::bind(&SafetyBubble::StateCallback, this, std::placeholders::_1));

    M_subscriber_ = this->create_subscription<std_msgs::msg::Float64MultiArray>("inertia_matrix_topic", 10,
        std::bind(&SafetyBubble::inertia_callback, this, std::placeholders::_1));

    jacobian_subscriber = this->create_subscription<std_msgs::msg::Float64MultiArray>("jacobian_topic", 10, 
    std::bind(&SafetyBubble::jacobian_callback, this, std::placeholders::_1));

    dq_subscriber = this->create_subscription<std_msgs::msg::Float64MultiArray>("dq_topic", 10, 
    std::bind(&SafetyBubble::dq_callback, this, std::placeholders::_1));

    D_subscriber_ = this->create_subscription<std_msgs::msg::Float64MultiArray>("damping_matrix_topic", 10,
        std::bind(&SafetyBubble::damping_callback, this, std::placeholders::_1));

    D_h_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("D_h_topic", 10);
        
    offset_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("goal_offset_topic", 10);

    hand_goal_publisher = this->create_publisher<std_msgs::msg::Float64MultiArray>("hand_goal_topic", 10);


    RCLCPP_INFO(this->get_logger(), "safety_bubble constructor started");

    timer_ = this->create_wall_timer(
     std::chrono::milliseconds(100), std::bind(&SafetyBubble::external_force, this));
}

void SafetyBubble::StateCallback(const franka_msgs::msg::FrankaRobotState::SharedPtr msg){
    if (inital_position_bool == false){
        inital_position_= msg->o_t_ee;
        inital_position_bool = true;
    }
    current_position_ = msg->o_t_ee;
}

void SafetyBubble::hand_position_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg){
    for (size_t j = 0; j < 3; ++j) {
        current_hand_position_array[j] = msg->data[j];
     }
}

void SafetyBubble::jacobian_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg){
    for (size_t i = 0; i < 6; ++i) {
        for (size_t j = 0; j < 7; ++j) {
            jacobian(i, j) = msg->data[i * 7 + j];
        }
    }
} 

void SafetyBubble::dq_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg){
    for (size_t j = 0; j < 7; ++j) {
        dq(j) = msg->data[j];
     }
} 

void SafetyBubble::inertia_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg){
    for (size_t i = 0; i < 6; ++i) {
        for (size_t j = 0; j < 6; ++j) {
            Lambda(i, j) = msg->data[i * 6 + j];
        }
    }
}

void SafetyBubble::damping_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg){
    for (size_t i = 0; i < 6; ++i) {
        for (size_t j = 0; j < 6; ++j) {
            D(i, j) = msg->data[i * 6 + j];
        }
    }
}

void SafetyBubble::external_force(){
    Eigen::Matrix<double, 6, 6> K =  (Eigen::MatrixXd(6,6) << 50,   0,   0,   0,   0,   0,
                                                                0, 50,   0,   0,   0,   0,
                                                                0,   0, 50,   0,   0,   0,  // impedance stiffness term
                                                                0,   0,   0, 26,   0,   0,
                                                                0,   0,   0,   0, 26,   0,
                                                                0,   0,   0,   0,   0,  2).finished(); 
    velocity = jacobian * dq; //calculates the velocity in operational space 
 
    double current_hand_x = current_hand_position_array[0];
    double current_hand_y = current_hand_position_array[1];
    double current_hand_z = current_hand_position_array[2];
    
    auto position = current_position_.pose.position; 
    auto inital_position = inital_position_.pose.position;
    inital_position_msg.data = {inital_position.x, inital_position.y, inital_position.z};
    
    position_vector = (Eigen::Matrix<double, 6,1>() << position.x,
                                                    position.y, 
                                                    position.z,
                                                    0,
                                                    0,
                                                    0).finished();
    phi = atan2(position.y,position.x-current_hand_x);
    if (!(sqrt(pow(position.x-current_hand_x, 2)+pow(position.y-current_hand_y,2)+pow(position.z-current_hand_z,2))==0)){
            theta = acos((position.z-0.4)/sqrt(pow(position.x-current_hand_x, 2)+pow(position.y-current_hand_y,2)+pow(position.z-current_hand_z,2)));
    }
    R_vector = (Eigen::Matrix<double, 6, 1>() << current_hand_x + R*cos(phi)*sin(theta),
                                                current_hand_y + R*sin(phi)*sin(theta),
                                                current_hand_z + R*cos(theta),
                                                0,
                                                0,
                                                0).finished(); 
    //max_movement as a sphere
    max_movement = (Eigen::Matrix<double, 6, 1>() << inital_position.x + r_eq*cos(phi)*sin(theta),
                                                inital_position.y + r_eq*sin(phi)*sin(theta),
                                                inital_position.z + r_eq*cos(theta),
                                                0,
                                                0,
                                                0).finished();
    //max_movement as a cube
    /*max_movement = (Eigen::Matrix<double, 6, 1>() << r_eq,
                                                     r_eq,
                                                     r_eq,
                                                     0,
                                                     0,
                                                     0).finished(); */

    r0 = sqrt(pow(inital_position.x,2)+pow(inital_position.y,2)+pow(inital_position.z,2)); //absolute value of radial vector to starting position of endeffector
    G = sqrt(pow(0.4,2)+pow(0.4,2)+pow(0.4,2)); //Goal position jeweils hier einsetzen!!!
    if (!(R==0)){
        alpha = G/R; //ratio use for linear combination relating goal position and assumed position upon entering the outer radius
    }
    switch(chosenPrimitive){
        case 1:{ //calculates Bubble Stiffness and Bubble Damping for AVOID
        if (sqrt(pow(position.x-current_hand_x,2)+pow(position.y-current_hand_y,2)+pow(position.z-current_hand_z,2)) < R){    //checks whether the endeffector is inside of the outer Radius
            if (!velocity_upon_entering_bool){ //if the velocity upon entering the safety bubble was not already defined, it is defined
                velocity_upon_entering = velocity;
                velocity_upon_entering_bool = true;
            }
            double zahler = r_eq + G;
            double nenner = R - r_eq;
            double stiffness_factor = zahler/nenner;
            for (int i = 0; i < 6; ++i) {
                for (int j = 0; j < 6; ++j) {
                    safety_bubble_stiffness(i, j) = K(i, j) * stiffness_factor;
                }
            }
            safety_bubble_stiffness_virtual = K*sqrt(pow(r_eq,2)+pow(r0,2)*(2*alpha+pow(alpha,2)))/(R-r_eq); 

            //calculte sqrt of Lambda           
            Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> eigen_solver(Lambda);
            Eigen::MatrixXd U = eigen_solver.eigenvectors();
            Eigen::MatrixXd D = eigen_solver.eigenvalues().cwiseSqrt().asDiagonal(); //ACHTUNG! change the name of this Matrix and also in the following line
            Eigen::MatrixXd Lambda_sqrt = U * D * U.transpose();

            Eigen::MatrixXd identity6x6 = Eigen::MatrixXd::Identity(6,6);           
            safety_bubble_damping_1 = (identity6x6+4*Lambda*(K+safety_bubble_stiffness_virtual)).sqrt() - D;
            C1 = -1*(velocity_upon_entering.norm()*Lambda*identity6x6)+0.5*(safety_bubble_damping_1 + D)*(R-r_eq)*identity6x6 + 0.5*(R - r_eq)*identity6x6;
            if (C1.norm() >= 0){ //Check whether C1 is non-negative, with damping of omega=1
                safety_bubble_damping = safety_bubble_damping_1;
                version = 1;
            }
            else{ //if C1 is negative: adjust the safety bubble damping, to ensure that C1 is positive
                if (velocity_upon_entering.norm() != 0){
                safety_bubble_damping = ((K+safety_bubble_stiffness_virtual)*pow(R+r_eq,2)+Lambda*pow(velocity_upon_entering.norm(),2))/((R-r_eq)*velocity_upon_entering.norm()) - D;
                version = 2;
                }
                else safety_bubble_damping = safety_bubble_damping_1; //dieser Fall wird nie eintretten, da  wenn velocity_upon_entering.norm()==0, dann auch C1 grösser als null wäre
            }
        }
        else{ //as long as the endeffector is outside of the bubble ->bubble is turned off
          safety_bubble_stiffness = safety_bubble_stiffness*0; 
          safety_bubble_damping = safety_bubble_damping*0;
        }
        break;
        }
        case 2:{ //calculates Bubble Stiffness and Bubble Damping for FOLLOW
            safety_bubble_stiffness = safety_bubble_stiffness*0; //Bubble ausgeschalten
            safety_bubble_damping = safety_bubble_damping*0; //Bubble ausgeschalten
            offset_array = {r_eq,r_eq,r_eq,0,0,0}; //cube around the hand
            auto offset_msg = std_msgs::msg::Float64MultiArray();
            for(long unsigned int i = 0; i < offset_array.size(); ++i){
                offset_msg.data.push_back(offset_array[i]);
            }
            offset_publisher_ ->publish(offset_msg); //publishing offset array to controller
            break;
        }
        case 3:{ //calculates Bubble Stiffness and Damping for HOLD
            safety_bubble_stiffness = safety_bubble_stiffness; //Stiffness (Bubble ausgeschalten)
            safety_bubble_damping = safety_bubble_damping; //Damping (Bubble ausgeschalten)
           for(int i = 0; i<6;++i){ //calculating the diagonal entries of the stiffness matrix
            if(max_movement(i) !=0){
                stiffness_hold(i,i)=external_force_assumption(i)/max_movement(i);
            }
            else stiffness_hold(i,i)=0;
           }

           Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> es_s(stiffness_hold);
            Eigen::MatrixXd D_s = es_s.eigenvalues().cwiseSqrt().asDiagonal();
            Eigen::MatrixXd U_s = es_s.eigenvectors();
            Eigen::MatrixXd stiffness_hold_sqrt = U_s * D_s * U_s.transpose();

           safety_bubble_damping = 2*stiffness_hold_sqrt; //adjusting the damping matrix (calling it safety_bubble damping as it is published with the same publisher), Lambda = Physical inertia = identity matrix (else multiply by sqrt of Lambda)

            for (int i = 0; i < 6; ++i) {
                for (int j = 0; j < 6; ++j) {
                    stiffness_hold_array[i * 6 + j] = stiffness_hold(i, j);
                 }
            }
            auto stiffness_hold_msg = std_msgs::msg::Float64MultiArray();
            for(long unsigned int i = 0; i < stiffness_hold_array.size(); ++i){
                stiffness_hold_msg.data.push_back(stiffness_hold_array[i]);
            }
            
            hold_stiffness_publisher_ ->publish(stiffness_hold_msg);
            hand_goal_publisher ->publish(inital_position_msg); //publishing the position, where the end-effector should stay
            break;
        }
        default:{ //default = no influence on the controller
            safety_bubble_stiffness = safety_bubble_stiffness; 
            safety_bubble_damping = safety_bubble_damping;
            break;
        }
    }
    Eigen::Matrix<double, 6, 1> R_minus_pos_vector = R_vector -position_vector; //calculates the "extension of the spring of the saftey bubble"
    Eigen::Matrix<double, 6, 1> F_spring = safety_bubble_stiffness*R_minus_pos_vector; //calculates the spring force
    int index_d_h = 0;
    for(int i = 0; i < 6; i++) {
        for(int j = 0; j < 6; j++) {
            safety_bubble_damping_array[index_d_h++] = safety_bubble_damping(i, j);
        }
    } 
    auto safety_bubble_damping_msg = std_msgs::msg::Float64MultiArray();
    for(long unsigned int i = 0; i < safety_bubble_damping_array.size(); ++i){
        safety_bubble_damping_msg.data.push_back(safety_bubble_damping_array[i]);
    }
    D_h_publisher_->publish(safety_bubble_damping_msg);
    F_ext =  F_spring; //calculates the external force = force of safety bubble, only calculates the safety bubble spring force, as the damping matrix is passed directly to the controller
    MatrixtoArray(F_ext);
}

void SafetyBubble::MatrixtoArray(Eigen::Matrix<double, 6, 1>& F_ext){
    Eigen::Map<Eigen::Matrix<double,6,1>>(F_ext_array.data()) = F_ext;
    auto f_ext_message = std_msgs::msg::Float64MultiArray();
    for(long unsigned int i = 0; i < F_ext_array.size(); ++i){
        f_ext_message.data.push_back(F_ext_array[i]);
    }
    RCLCPP_INFO(this->get_logger(), "f_ext_message values: [%f, %f, %f, %f, %f, %f, %f]",
              f_ext_message.data[0], f_ext_message.data[1], f_ext_message.data[2], f_ext_message.data[3], f_ext_message.data[4], f_ext_message.data[5], version); 
    f_ext_publisher_->publish(f_ext_message);
}

int main(int argc, char *argv[]){
    rclcpp::init(argc, argv);
    auto safety_node = std::make_shared<SafetyBubble>();
    rclcpp::spin(safety_node);
    rclcpp::shutdown();
    return 0;
};