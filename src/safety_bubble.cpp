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

    //test_results_publisher_ = this->create_publisher<std_msgs::msg::Bool>("inside_bool",10);

    test_end_publisher_ = this->create_publisher<std_msgs::msg::Bool>("test_end_bool",10);

    goal_publisher_ = this->create_publisher<messages_fr3::msg::SetPose>("set_new_pose",10);

    hold_stiffness_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("hold_stiffness", 10);

    test_hand_position_subscriber_ = this->create_subscription<messages_fr3::msg::SetPose>("hand_position", 10,
    std::bind(&SafetyBubble::hand_position_callback, this, std::placeholders::_1));

    franka_state_subscriber = this->create_subscription<franka_msgs::msg::FrankaRobotState>("franka_robot_state_broadcaster/robot_state", 10,
    std::bind(&SafetyBubble::StateCallback, this, std::placeholders::_1));

    M_subscriber_ = this->create_subscription<std_msgs::msg::Float64MultiArray>("inertia_matrix_topic", 10,
        std::bind(&SafetyBubble::inertia_callback, this, std::placeholders::_1));

    jacobian_subscriber = this->create_subscription<std_msgs::msg::Float64MultiArray>("jacobian_topic", 10, 
    std::bind(&SafetyBubble::jacobian_callback, this, std::placeholders::_1));

    D_h_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("safety_damping_matrix_topic", 10);
    P_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("safety_stiffness_matrix_topic", 10);

    offset_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("goal_offset_topic", 10);

    RCLCPP_INFO(this->get_logger(), "safety_bubble constructor started");

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100), std::bind(&SafetyBubble::external_force, this));
}

void SafetyBubble::StateCallback(const franka_msgs::msg::FrankaRobotState::SharedPtr msg){
    if (inital_position_bool == false){
        inital_position_= msg->o_t_ee;
        inital_position_bool = true;
    }
    previous_position_ = current_position_;
    current_position_ = msg->o_t_ee;
   // RCLCPP_INFO(this->get_logger(), "state callback in safetybubble has been started.");

    current_joint_state_= msg->measured_joint_state;
}

void SafetyBubble::hand_position_callback(const messages_fr3::msg::SetPose::SharedPtr msg){
    current_hand_position_ = *msg;
}

void SafetyBubble::jacobian_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg){

   /* RCLCPP_INFO(this->get_logger(), "Received message values: [%f, %f, %f, %f, %f, %f]",
               msg->data[0], msg->data[1], msg->data[2], msg->data[3], msg->data[4], msg->data[5]); */
    Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(msg->data.data());
}

void SafetyBubble::inertia_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg){
    Eigen::Map<Eigen::Matrix<double, 6, 6>> Lambda(msg->data.data());
}

void SafetyBubble::external_force(){
    RCLCPP_INFO(this->get_logger(), "external force calculation has been started.");
    Eigen::Matrix<double, 6, 6> K =  (Eigen::MatrixXd(6,6) << 250,   0,   0,   0,   0,   0,
                                                                0, 250,   0,   0,   0,   0,
                                                                0,   0, 250,   0,   0,   0,  // impedance stiffness term
                                                                0,   0,   0, 130,   0,   0,
                                                                0,   0,   0,   0, 130,   0,
                                                                0,   0,   0,   0,   0,  10).finished();

    Eigen::Map<const Eigen::Matrix<double, 7, 1>> dq(current_joint_state_.velocity.data());
    RCLCPP_INFO(this->get_logger(), "dq erreicht.");
    if (!jacobian.isZero()){
        RCLCPP_ERROR(this->get_logger(), "jacobian is not zero");
    }
    velocity = jacobian * dq; //calculates the velocity in operational space
     if (velocity.isZero()){
        RCLCPP_ERROR(this->get_logger(), "velocity is zero");
    }
    if (!velocity.isZero()){
        RCLCPP_INFO(this->get_logger(), "velocity is not zero");
    }

    

    if (!dq.isZero()){
        RCLCPP_ERROR(this->get_logger(), "dq is not zero");
    }
    RCLCPP_INFO(this->get_logger(), "velocity erreicht.");

 
    auto current_hand_x = current_hand_position_.x;
    auto current_hand_y = current_hand_position_.y;
    auto current_hand_z = current_hand_position_.z;
    //std::array<double, 42> jacobian_array =  franka_robot_model_->getZeroJacobian(franka::Frame::kEndEffector); //get Jacobian as an array
    //Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data()); //convert the jacobian array into Jacobian matrix

    //Lambda = (jacobian * M.inverse() * jacobian.transpose()).inverse();
     RCLCPP_INFO(this->get_logger(), "hand erreicht.");

    auto position = current_position_.pose.position;
    auto inital_position = inital_position_.pose.position;
        RCLCPP_INFO(this->get_logger(), "pose erreicht.");

    position_vector = (Eigen::Matrix<double, 6,1>() << position.x,
                                                    position.y, 
                                                    position.z,
                                                    0,
                                                    0,
                                                    0).finished();
   // RCLCPP_INFO(this->get_logger(), "position_vector ready");
    RCLCPP_INFO(this->get_logger(), "posevector erreicht.");


    Eigen::Vector3d position_vector3d = (Eigen::Vector3d() << position.x,
                                            position.y,
                                            position.z).finished();
/*
    if (sqrt(pow(position.x,2)+pow(position.y,2)+pow(position.z,2)) < r_eq ){
        inside = true;
    }

   
    velocity_vector = (Eigen::Matrix<double, 6, 1>() << velocity[0],
                                                        velocity[1],
                                                        velocity[2],
                                                        velocity[3],
                                                        velocity[4],
                                                        velocity[5]).finished();
 */
    if (!inital_velocity_bool and (velocity[0]!=0 or velocity[1]!=0 or velocity[2]!=0)){
        inital_velocity_vector = velocity /*_vector*/;
        inital_velocity_bool = true;
    }

    if (inital_velocity_bool){
        RCLCPP_ERROR(this->get_logger(), "Moved, inital_velocity_bool is true");
    }
   // RCLCPP_INFO(this->get_logger(), "velocity vector ready");

   
    auto phi = atan2(position.y,position.x);
    if (!(sqrt(pow(position.x, 2)+pow(position.y,2)+pow(position.z,2))==0)){
            theta = acos(position_vector3d(2)/sqrt(pow(position_vector3d(0), 2)+pow(position_vector3d(1),2)+pow(position_vector3d(2),2)));
    }
   // RCLCPP_INFO(this->get_logger(), "theta ready");
    R_vector = (Eigen::Matrix<double, 6, 1>() << current_hand_x + R*cos(phi)*sin(theta),
                                                current_hand_y + R*sin(phi)*sin(theta),
                                                current_hand_z + R*cos(theta),
                                                0,
                                                0,
                                                0).finished();
    max_movement = (Eigen::Matrix<double, 6, 1>() << r_eq*cos(phi)*sin(theta),
                                                r_eq*sin(phi)*sin(theta),
                                                r_eq*cos(theta),
                                                0,
                                                0,
                                                0).finished();
   // RCLCPP_INFO(this->get_logger(), "R_vector and max_movement ready");

    r0 = sqrt(pow(inital_position.x,2)+pow(inital_position.y,2)+pow(inital_position.z,2)); //absolute value of radial vector to starting position of endeffector
   // RCLCPP_INFO(this->get_logger(), "r0 ready");
    goal_pose = position_d_;
   // RCLCPP_INFO(this->get_logger(), "goal_pose ready");
    G = sqrt(pow(goal_pose[1],2)+pow(goal_pose[2],2)+pow(goal_pose[3],2)); //absolute value of radial vector to goal position
    //RCLCPP_INFO(this->get_logger(), "G ready");
    if (!(R==0)){
        alpha = G/R; //ratio use for linear combination relating goal position and starting postition
    }
    
    //RCLCPP_INFO(this->get_logger(), "alpha ready");
    //RCLCPP_INFO(this->get_logger(), "precalculations are made");


        switch(chosenPrimitive){
        case 1:{ //calculates Bubble Stiffness and Bubble Damping for AVOID
        if (sqrt(pow(position.x-current_hand_x,2)+pow(position.y-current_hand_y,2)+pow(position.z-current_hand_z,2)) < R){     //checks whether the endeffector is inside of the outer Radius
            safety_bubble_stiffness = K*(r_eq+G)/(R-r_eq);
            RCLCPP_INFO(this->get_logger(), "safety bubble stiffness calculated");

            safety_bubble_stiffness_virtual = K*sqrt(pow(r_eq,2)+pow(r0,2)*(2*alpha+pow(alpha,2)))/(R-r_eq);
            RCLCPP_INFO(this->get_logger(), "virtual safety bubble stiffness calculated");

            safety_bubble_damping = 2*Lambda.sqrt()*((K+safety_bubble_stiffness_virtual).sqrt()-K.sqrt()); //Damping
            RCLCPP_INFO(this->get_logger(), "safety bubble damping calculated");

            RCLCPP_INFO(this->get_logger(), "AVOID Pramameters calculated");
        }
        else{ //as long as the endeffector is outside of the bubble ->bubble is turned off
        safety_bubble_stiffness = safety_bubble_stiffness; 
        safety_bubble_damping = safety_bubble_damping;
        }
        break;
        }
        case 2:{ //calculates Bubble Stiffness and Bubble Damping for FOLLOW
            //Method, where safetybubble is applied and the goal is directly on the hand
            /*
            if (sqrt(pow(position.x-current_hand_x,2)+pow(position.y-current_hand_y,2)+pow(position.z-current_hand_z,2)) < R){ 
            safety_bubble_stiffness = K*(r_eq+G)/(R-r_eq);
            safety_bubble_stiffness_virtual = K*(r_eq*r0*sqrt(pow(alpha,2)+2*alpha))/(R-r_eq);
            safety_bubble_damping = 2*Lambda.sqrt()*((K+safety_bubble_stiffness_virtual).sqrt()-K.sqrt()); //Damping
            }
            */
            //Method, where no safetybubble is applied but the goal is positioned with an offset (achtung, immer eine der beiden methoden ausklammern, goal_publisher braucht es bei beiden)
            offset_array = {r_eq,r_eq,r_eq,0,0,0};
            auto offset_msg = std_msgs::msg::Float64MultiArray();
            for(long unsigned int i = 0; i < offset_array.size(); ++i){
                offset_msg.data.push_back(offset_array[i]);
            }
            RCLCPP_INFO(this->get_logger(), "FOLLOW Pramameters calculated");
            goal_publisher_ ->publish(current_hand_position_);
            offset_publisher_ ->publish(offset_msg);
            break;
        }
        case 3:{ //calculates Bubble Stiffness and Damping for HOLD
            safety_bubble_stiffness = safety_bubble_stiffness; //Stiffness (Bubble wird ausgeschalten)
            safety_bubble_damping = safety_bubble_damping; //Damping
            goal_publisher_ ->publish(current_hand_position_);
            if (position_vector3d == goal_pose){ //if statement, damit zuerst der Ort erreicht wird
            Eigen::Matrix<double, 1, 6> pinv_max_movement =  max_movement.completeOrthogonalDecomposition().pseudoInverse();
            stiffness_hold = external_force_assumption * pinv_max_movement;
            Eigen::Map<Eigen::Matrix<double,6,6>>(stiffness_hold_array.data()) = stiffness_hold;
            auto stiffness_hold_msg = std_msgs::msg::Float64MultiArray();
            for(long unsigned int i = 0; i < stiffness_hold_array.size(); ++i){
                stiffness_hold_msg.data.push_back(stiffness_hold_array[i]);
            }
            //RCLCPP_INFO(this->get_logger(), "HOLD Pramameters calculated");
            hold_stiffness_publisher_ ->publish(stiffness_hold_msg);
            }
            break;
        }
        default:{ //default = Bubble wird ausgeschalten
            safety_bubble_stiffness = safety_bubble_stiffness; 
            safety_bubble_damping = safety_bubble_damping;
            break;
        }
        Eigen::Map<Eigen::Matrix<double,6,6>>(D_h_array.data()) = safety_bubble_damping;
            auto D_h_msg = std_msgs::msg::Float64MultiArray();
            for(long unsigned int i = 0; i < D_h_array.size(); ++i){
                D_h_msg.data.push_back(D_h_array[i]);
            }
            RCLCPP_INFO(this->get_logger(), "safety bubble damping ready to be published");

            D_h_publisher_ ->publish(D_h_msg);
        Eigen::Map<Eigen::Matrix<double,6,6>>(P_array.data()) = safety_bubble_stiffness;
            auto P_msg = std_msgs::msg::Float64MultiArray();
            for(long unsigned int i = 0; i < P_array.size(); ++i){
                P_msg.data.push_back(P_array[i]);
            }
            RCLCPP_INFO(this->get_logger(), "safety bubble stiffness ready to be published");

            P_publisher_ ->publish(P_msg);
    }
   
    /*
    else{ //as long as the endeffector is outside of the bubble ->bubble is turned off
        safety_bubble_stiffness = safety_bubble_stiffness; 
        safety_bubble_damping = safety_bubble_damping;
    }*/

    Eigen::Matrix<double, 6, 1> R_minus_pos_vector = R_vector -position_vector;
    Eigen::Matrix<double, 6, 1> F_spring = safety_bubble_stiffness*R_minus_pos_vector;
    Eigen::Matrix<double, 6, 1> F_damper = safety_bubble_damping*velocity /*_vector*/;
   // RCLCPP_INFO(this->get_logger(), "ready to calculate F_ext");

    F_ext =  F_spring - F_damper; //calculates the external force = force of safety bubble
   // RCLCPP_INFO(this->get_logger(), "F_ext calculated");
   bool vel0_bool;
    if ((abs(previous_position_.pose.position.x - position.x) < 0.00002) and (abs(previous_position_.pose.position.y - position.y) < 0.00002) and (abs(previous_position_.pose.position.z - position.z) < 0.00002)){
        vel0_bool = true;
    }
    else vel0_bool = false;

    if ((goal_pose == position_vector3d or vel0_bool) /*and inital_velocity_bool*/){
        test_end = true;
     }
     std::cout<< "test_end is " << test_end;


    MatrixtoArray(F_ext);
    /*
    auto inside_msg = std_msgs::msg::Bool();
    inside_msg.data = inside;
    test_results_publisher_ ->publish(inside_msg);
    */
    auto test_end_msg = std_msgs::msg::Bool();
    test_end_msg.data = test_end;
    test_end_publisher_->publish(test_end_msg);

}

void SafetyBubble::MatrixtoArray(Eigen::Matrix<double, 6, 1>& F_ext){
    //RCLCPP_INFO(this->get_logger(), "Matrix to Array called");

    Eigen::Map<Eigen::Matrix<double,6,1>>(F_ext_array.data()) = F_ext;
    auto f_ext_message = std_msgs::msg::Float64MultiArray();
    for (const auto& value : F_ext_array){
        f_ext_message.data.push_back(value);
    }
    RCLCPP_INFO(this->get_logger(), "f_ext_message values: [%f, %f, %f, %f, %f, %f]",
                f_ext_message.data[0], f_ext_message.data[1], f_ext_message.data[2],
                f_ext_message.data[3], f_ext_message.data[4], f_ext_message.data[5]);

    /*
    for(long unsigned int i = 0; i < F_ext_array.size(); ++i){
        f_ext_message.data.push_back(F_ext_array[i]);
    }*/
   // RCLCPP_INFO(this->get_logger(), "Matrix to Array calculated");

    f_ext_publisher_->publish(f_ext_message);
}

int main(int argc, char *argv[]){
    rclcpp::init(argc, argv);
    auto safety_node = std::make_shared<SafetyBubble>();
    RCLCPP_INFO(safety_node->get_logger(), "SafetyBubble is spinning");
    rclcpp::spin(safety_node);
    RCLCPP_INFO(safety_node->get_logger(), "SafetyBubble has shutdown");
    rclcpp::shutdown();

    return 0;
};