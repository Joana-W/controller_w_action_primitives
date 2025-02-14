// Copyright (c) 2021 Franka Emika GmbH
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <array>
#include <cassert>
#include <cmath>
#include <cstring>
#include <exception>
#include <memory>
#include <mutex>
#include <string>
#include <unistd.h>
#include <thread>
#include <chrono>         

#include <cartesian_impedance_control/user_input_server.hpp>

#include <rclcpp/rclcpp.hpp>
#include "rclcpp/subscription.hpp"

#include <Eigen/Dense>
#include <Eigen/Eigen>
#include <Eigen/Eigenvalues>

#include <controller_interface/controller_interface.hpp>

#include <franka/model.h>
#include <franka/robot.h>
#include <franka/robot_state.h>

#include "franka_hardware/franka_hardware_interface.hpp"
#include <franka_hardware/model.hpp>

#include "franka_msgs/msg/franka_robot_state.hpp"
#include "franka_msgs/msg/errors.hpp"
#include "messages_fr3/srv/set_pose.hpp"
#include "messages_fr3/msg/set_pose.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include "franka_semantic_components/franka_robot_model.hpp"
#include "franka_semantic_components/franka_robot_state.hpp"

//#include "cartesian_impedance_control/safety_bubble.hpp"

#define IDENTITY Eigen::MatrixXd::Identity(6, 6)

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using Vector7d = Eigen::Matrix<double, 7, 1>;

namespace cartesian_impedance_control {

class CartesianImpedanceController : public controller_interface::ControllerInterface {
public:
  [[nodiscard]] controller_interface::InterfaceConfiguration command_interface_configuration()
      const override;

  [[nodiscard]] controller_interface::InterfaceConfiguration state_interface_configuration()
      const override;

  controller_interface::return_type update(const rclcpp::Time& time,
                                           const rclcpp::Duration& period) override;
  controller_interface::CallbackReturn on_init() override;

  controller_interface::CallbackReturn on_configure(
      const rclcpp_lifecycle::State& previous_state) override;

  controller_interface::CallbackReturn on_activate(
      const rclcpp_lifecycle::State& previous_state) override;

  controller_interface::CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State& previous_state) override;

    void setPose(const std::shared_ptr<messages_fr3::srv::SetPose::Request> request, 
    std::shared_ptr<messages_fr3::srv::SetPose::Response> response);
      
  Eigen::Matrix<double, 6, 6> Lambda = IDENTITY;                                           // operational space mass matrix
  Eigen::Vector3d position_d_;

 private:
    //Nodes
    rclcpp::Subscription<franka_msgs::msg::FrankaRobotState>::SharedPtr franka_state_subscriber = nullptr;
    rclcpp::Service<messages_fr3::srv::SetPose>::SharedPtr pose_srv_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr safety_bubble_subscriber; //subscribes to safety bubble spring force
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr hold_stiffness_subscriber_; //subscribes to the stiffness matrix for the primitive hold
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr safety_bubble_damping_subscriber; //subscribes to the safety bubble damping matrix/ damping matrix (for primitive hold)
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr hold_force_subscriber_; //subscribes to external test force for the primitive hold
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr goal_offset_subscriber_; //subscribes to offset for the primitive follow
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr goal_hand_subscriber_; //in the case of follow and hold, is new pose (position of the hand) directly received through this subscriber

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr lambda_publisher;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr K_publisher;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr D_publisher;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr jacobian_publisher;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr dq_publisher;


    std::array<double, 36> lambda_array;
    std::array<double, 36> K_array;
    std::array<double, 36> D_array;
    messages_fr3::msg::SetPose position_d_target_msg;


    //Functions
    void topic_callback(const std::shared_ptr<franka_msgs::msg::FrankaRobotState> msg);
    void f_safety_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    std_msgs::msg::Float64MultiArray f_safety_;
    void new_pose_callback(const messages_fr3::msg::SetPose::SharedPtr msg);
    void hold_stiffness_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    void safety_bubble_damping_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    void hold_force_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    void offset_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    void hand_goal_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);


    void updateJointStates();
    void update_stiffness_and_references();
    void arrayToMatrix(const std::array<double, 6>& inputArray, Eigen::Matrix<double, 6, 1>& resultMatrix);
    void arrayToMatrix(const std::array<double, 7>& inputArray, Eigen::Matrix<double, 7, 1>& resultMatrix);
    Eigen::Matrix<double, 7, 1> saturateTorqueRate(const Eigen::Matrix<double, 7, 1>& tau_d_calculated, const Eigen::Matrix<double, 7, 1>& tau_J_d);  
    Eigen::Matrix<double, 7, 1> saturateTorque(const Eigen::Matrix<double, 7, 1>& tau_d_calculated);  

    std::array<double, 6> convertToStdArray(const geometry_msgs::msg::WrenchStamped& wrench);
    //State vectors and matrices
    std::array<double, 7> q_subscribed;
    std::array<double, 7> tau_J_d = {0,0,0,0,0,0,0};
    std::array<double, 6> O_F_ext_hat_K = {0,0,0,0,0,0};
    Eigen::Matrix<double, 7, 1> q_subscribed_M;
    Eigen::Matrix<double, 7, 1> tau_J_d_M = Eigen::MatrixXd::Zero(7, 1);
    Eigen::Matrix<double, 6, 1> O_F_ext_hat_K_M = Eigen::MatrixXd::Zero(6,1);
    Eigen::Matrix<double, 7, 1> q_;
    Eigen::Matrix<double, 7, 1> dq_;
    Eigen::MatrixXd jacobian_transpose_pinv;  

    //Robot parameters
    const int num_joints = 7;
    const std::string state_interface_name_{"robot_state"};
    const std::string robot_name_{"panda"};
    const std::string k_robot_state_interface_name{"robot_state"};
    const std::string k_robot_model_interface_name{"robot_model"};
    franka_hardware::FrankaHardwareInterface interfaceClass;
    std::unique_ptr<franka_semantic_components::FrankaRobotModel> franka_robot_model_;
    const double delta_tau_max_{1.0};
    const double dt = 0.001;
                
    //Impedance control variables              
    Eigen::Matrix<double, 6, 6> Sm = IDENTITY;                                               // task space selection matrix for positions and rotation
    Eigen::Matrix<double, 6, 6> Sf = Eigen::MatrixXd::Zero(6, 6);                            // task space selection matrix for forces
   Eigen::Matrix<double, 6, 6> K =  (Eigen::MatrixXd(6,6) << 50,   0,   0,   0,   0,   0,
                                                                0, 50,   0,   0,   0,   0,
                                                                0,   0, 50,   0,   0,   0,  // impedance stiffness term
                                                                0,   0,   0, 26,   0,   0,
                                                                0,   0,   0,   0, 26,   0,
                                                                0,   0,   0,   0,   0,  2).finished(); 
    Eigen::Matrix<double, 6, 6> D =  (Eigen::MatrixXd(6,6) <<  14.2,   0,   0,   0,   0,   0,
                                                                0,  14.2,   0,   0,   0,   0,
                                                                0,   0,  14.2,   0,   0,   0,  // impedance damping term
                                                                0,   0,   0,   10.1,   0,   0,
                                                                0,   0,   0,   0,   10.1,   0,
                                                                0,   0,   0,   0,   0,      2.8).finished();  

    /* Eigen::Matrix<double, 6, 6> K =  (Eigen::MatrixXd(6,6) << 250,   0,   0,   0,   0,   0,
                                                                 0, 250,   0,   0,   0,   0,
                                                                0,   0, 250,   0,   0,   0,  // impedance stiffness term
                                                                0,   0,   0,  130,   0,   0,
                                                                0,   0,   0,   0,  130,   0,
                                                                0,   0,   0,   0,   0,  10).finished(); 

    Eigen::Matrix<double, 6, 6> D =  (Eigen::MatrixXd(6,6) <<  35,   0,   0,   0,   0,   0,
                                                                0,  35,   0,   0,   0,   0,
                                                                0,   0,  35,   0,   0,   0,  // impedance damping term
                                                                0,   0,   0,  25,   0,   0,
                                                                0,   0,   0,   0,  25,   0,
                                                                0,   0,   0,   0,   0,   6).finished(); */
    Eigen::Matrix<double, 6, 6> Theta = IDENTITY;
    Eigen::Matrix<double, 6, 6> T = (Eigen::MatrixXd(6,6) <<       1,   0,   0,   0,   0,   0,
                                                                   0,   1,   0,   0,   0,   0,
                                                                   0,   0,   2.5,   0,   0,   0,  // Inertia term
                                                                   0,   0,   0,   1,   0,   0,
                                                                   0,   0,   0,   0,   1,   0,
                                                                   0,   0,   0,   0,   0,   2.5).finished();                                               // impedance inertia term

    Eigen::Matrix<double, 6, 6> cartesian_stiffness_target_;                                 // impedance damping term
    Eigen::Matrix<double, 6, 6> cartesian_damping_target_;                                   // impedance damping term
    Eigen::Matrix<double, 6, 6> cartesian_inertia_target_;                                   // impedance damping term
    Eigen::Vector3d position_d_target_ = {0.5, 0.0, 0.5}; 
    Eigen::Vector3d rotation_d_target_ = {M_PI, 0.0, 0.0};
    Eigen::Quaterniond orientation_d_target_;
    Eigen::Quaterniond orientation_d_; 
    Eigen::Matrix<double, 6, 1> F_impedance;  
    Eigen::Matrix<double, 6, 1> F_contact_des = Eigen::MatrixXd::Zero(6, 1);                 // desired contact force
    Eigen::Matrix<double, 6, 1> F_contact_target = Eigen::MatrixXd::Zero(6, 1);              // desired contact force used for filtering
    Eigen::Matrix<double, 6, 1> F_ext = Eigen::MatrixXd::Zero(6, 1);                         // external forces
    Eigen::Matrix<double, 6, 1> F_cmd = Eigen::MatrixXd::Zero(6, 1);                         // commanded contact force
    Eigen::Matrix<double, 7, 1> q_d_nullspace_;
    Eigen::Matrix<double, 6, 1> error;                                                       // pose error (6d)
    double nullspace_stiffness_{0.001};
    double nullspace_stiffness_target_{0.001};

    bool maximum_error_necessary = false;
    double error_factor = 1;
    double max_error_factor = 1;
    double max_error = 0.2;
    Eigen::Matrix<double, 6, 1> scaled_error;                                                // scaled pose error (6d)

    Eigen::Matrix<double, 6, 1> F_safety; //safety bubble spring force
    Eigen::Matrix<double, 6, 1> F_hold; //external test force for the primitive hold
    Eigen::Matrix<double, 6, 6> K_hold; //contains the adjusted stiffness for the case of hold
    Eigen::Matrix<double, 6, 6> D_h; //safety bubble damping matrix (for the primitive follow) / damping matrix (for the primitive hold)
    Eigen::Vector3d Offset; //offset vector for the primitive follow
    
    //Logging
    int outcounter = 0;
    const int update_frequency = 2; //frequency for update outputs

    //Integrator
    Eigen::Matrix<double, 6, 1> I_error = Eigen::MatrixXd::Zero(6, 1);                      // pose error (6d)
    Eigen::Matrix<double, 6, 1> I_F_error = Eigen::MatrixXd::Zero(6, 1);                    // force error integral
    Eigen::Matrix<double, 6, 1> integrator_weights = 
      (Eigen::MatrixXd(6,1) << 75.0, 75.0, 75.0, 75.0, 75.0, 4.0).finished();
    Eigen::Matrix<double, 6, 1> max_I = 
      (Eigen::MatrixXd(6,1) << 30.0, 30.0, 30.0, 50.0, 50.0, 2.0).finished();

   
  
    std::mutex position_and_orientation_d_target_mutex_;

    //Flags
    bool config_control = false;           // sets if we want to control the configuration of the robot in nullspace
    bool do_logging = false;               // set if we do log values

    //Filter-parameters
    double filter_params_{0.001};
    int mode_ = 1;
};
}  // namespace cartesian_impedance_control