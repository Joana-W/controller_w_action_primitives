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

#include <cartesian_impedance_control/cartesian_impedance_controller.hpp>

#include <cassert>
#include <cmath>
#include <exception>
#include <string>

#include <Eigen/Eigen>

namespace {

template <class T, size_t N>
std::ostream& operator<<(std::ostream& ostream, const std::array<T, N>& array) {
  ostream << "[";
  std::copy(array.cbegin(), array.cend() - 1, std::ostream_iterator<T>(ostream, ","));
  std::copy(array.cend() - 1, array.cend(), std::ostream_iterator<T>(ostream));
  ostream << "]";
  return ostream;
}
}

namespace cartesian_impedance_control {

void CartesianImpedanceController::update_stiffness_and_references(){
  //target by filtering
  /** at the moment we do not use dynamic reconfigure and control the robot via D, K and T **/
  //K = filter_params_ * cartesian_stiffness_target_ + (1.0 - filter_params_) * K;
  //D = filter_params_ * cartesian_damping_target_ + (1.0 - filter_params_) * D;
  nullspace_stiffness_ = filter_params_ * nullspace_stiffness_target_ + (1.0 - filter_params_) * nullspace_stiffness_;
  //std::lock_guard<std::mutex> position_d_target_mutex_lock(position_and_orientation_d_target_mutex_);
  position_d_ = filter_params_ * position_d_target_ + (1.0 - filter_params_) * position_d_;
  orientation_d_ = orientation_d_.slerp(filter_params_, orientation_d_target_);
  F_contact_des = 0.05 * F_contact_target + 0.95 * F_contact_des;
}


void CartesianImpedanceController::arrayToMatrix(const std::array<double,7>& inputArray, Eigen::Matrix<double,7,1>& resultMatrix)
{
 for(long unsigned int i = 0; i < 7; ++i){
     resultMatrix(i,0) = inputArray[i];
   }
}

void CartesianImpedanceController::arrayToMatrix(const std::array<double,6>& inputArray, Eigen::Matrix<double,6,1>& resultMatrix)
{
 for(long unsigned int i = 0; i < 6; ++i){
     resultMatrix(i,0) = inputArray[i];
   }
}

Eigen::Matrix<double, 7, 1> CartesianImpedanceController::saturateTorqueRate(
  const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
  const Eigen::Matrix<double, 7, 1>& tau_J_d_M) {  
  Eigen::Matrix<double, 7, 1> tau_d_saturated{};
  for (size_t i = 0; i < 7; i++) {
  double difference = tau_d_calculated[i] - tau_J_d_M[i];
  tau_d_saturated[i] =
         tau_J_d_M[i] + std::max(std::min(difference, delta_tau_max_), -delta_tau_max_);
  }
  return tau_d_saturated;
}

Eigen::Matrix<double, 7, 1> CartesianImpedanceController::saturateTorque(
  const Eigen::Matrix<double, 7, 1>& tau_d) {  
  Eigen::Matrix<double, 7, 1> tau_d_calculated = tau_d;
  Eigen::Matrix<double, 7, 1> tau_d_saturated{};
  Eigen::Matrix<double, 7, 1> tau_max;
  tau_max << 87, 87, 87, 87, 12, 12, 12;  
  bool need_to_saturate = false;
  double max_factor = 1;
  double factor;
  for (size_t i = 0; i < 7; i++) {
    if (abs(tau_d_calculated(i)) > abs(tau_max(i))){
      need_to_saturate = true;
      factor = abs(tau_max(i))/abs(tau_d_calculated(i));
      if (factor < max_factor){
        max_factor = factor;
      }
    }
  }
 // RCLCPP_INFO(get_node()->get_logger(), "tau_d_Calculated vorher: [%f, %f, %f, %f, %f, %f, %f, %f]",
   //     tau_d_calculated[0], tau_d_calculated[1], tau_d_calculated[2], tau_d_calculated[3], tau_d_calculated[4], tau_d_calculated[5], tau_d_calculated[6], tau_d_calculated[7]);
  if (need_to_saturate){
    for (size_t i = 0; i < 7; i++){
      tau_d_saturated(i) = tau_d_calculated(i)*max_factor;
    }
    RCLCPP_INFO(get_node()->get_logger(), "tau_d saturated"); 
  }
  else tau_d_saturated = tau_d_calculated;
 // RCLCPP_INFO(get_node()->get_logger(), "tau_d_saturated nachher: [%f, %f, %f, %f, %f, %f, %f, %f]",
   //     tau_d_saturated[0], tau_d_saturated[1], tau_d_saturated[2], tau_d_saturated[3], tau_d_saturated[4], tau_d_saturated[5], tau_d_saturated[6], tau_d_saturated[7]);
  return tau_d_saturated;
}

inline void pseudoInverse(const Eigen::MatrixXd& M_, Eigen::MatrixXd& M_pinv_, bool damped = true) {
  double lambda_ = damped ? 0.2 : 0.0;
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(M_, Eigen::ComputeFullU | Eigen::ComputeFullV);   
  Eigen::JacobiSVD<Eigen::MatrixXd>::SingularValuesType sing_vals_ = svd.singularValues();
  Eigen::MatrixXd S_ = M_;  // copying the dimensions of M_, its content is not needed.
  S_.setZero();

  for (int i = 0; i < sing_vals_.size(); i++)
     S_(i, i) = (sing_vals_(i)) / (sing_vals_(i) * sing_vals_(i) + lambda_ * lambda_);

  M_pinv_ = Eigen::MatrixXd(svd.matrixV() * S_.transpose() * svd.matrixU().transpose());
}


controller_interface::InterfaceConfiguration
CartesianImpedanceController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (int i = 1; i <= num_joints; ++i) {
    config.names.push_back(robot_name_ + "_joint" + std::to_string(i) + "/effort");
  }
  return config;
}


controller_interface::InterfaceConfiguration CartesianImpedanceController::state_interface_configuration()
  const {
  controller_interface::InterfaceConfiguration state_interfaces_config;
  state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (int i = 1; i <= num_joints; ++i) {
    state_interfaces_config.names.push_back(robot_name_ + "_joint" + std::to_string(i) + "/position");
    state_interfaces_config.names.push_back(robot_name_ + "_joint" + std::to_string(i) + "/velocity");
  }

  for (const auto& franka_robot_model_name : franka_robot_model_->get_state_interface_names()) {
    state_interfaces_config.names.push_back(franka_robot_model_name);
    std::cout << franka_robot_model_name << std::endl;
  }

  const std::string full_interface_name = robot_name_ + "/" + state_interface_name_;

  return state_interfaces_config;
}


CallbackReturn CartesianImpedanceController::on_init() {
   UserInputServer input_server_obj(&position_d_target_, &rotation_d_target_, &K, &D, &T);
   std::thread input_thread(&UserInputServer::main, input_server_obj, 0, nullptr);
   input_thread.detach();
   return CallbackReturn::SUCCESS;
}


CallbackReturn CartesianImpedanceController::on_configure(const rclcpp_lifecycle::State& /*previous_state*/) {
  franka_robot_model_ = std::make_unique<franka_semantic_components::FrankaRobotModel>(
  franka_semantic_components::FrankaRobotModel(robot_name_ + "/" + k_robot_model_interface_name,
                                               robot_name_ + "/" + k_robot_state_interface_name));
                        
  try {
    rclcpp::QoS qos_profile(1); // Depth of the message queue
    qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    franka_state_subscriber = get_node()->create_subscription<franka_msgs::msg::FrankaRobotState>(
    "franka_robot_state_broadcaster/robot_state", qos_profile, 
    std::bind(&CartesianImpedanceController::topic_callback, this, std::placeholders::_1));
    std::cout << "Succesfully subscribed to robot_state_broadcaster" << std::endl;

    safety_bubble_subscriber = get_node()->create_subscription<std_msgs::msg::Float64MultiArray>(
      "external_force_topic", 10, std::bind(&CartesianImpedanceController::f_safety_callback, this, std::placeholders::_1));
    std::cout << "Successfully subscribed to safety_bubble" << std::endl;

   // safety_bubble_new_position_subscriber = get_node()->create_subscription<messages_fr3::msg::SetPose>("set_new_pose",10,
   //   std::bind(&CartesianImpedanceController::new_pose_callback, this, std::placeholders::_1));

    hold_stiffness_subscriber_ = get_node()->create_subscription<std_msgs::msg::Float64MultiArray>(
      "hold_stiffness", 10, std::bind(&CartesianImpedanceController::hold_stiffness_callback, this, std::placeholders::_1));

    safety_bubble_damping_subscriber = get_node()->create_subscription<std_msgs::msg::Float64MultiArray>(
      "D_h_topic", 10, std::bind(&CartesianImpedanceController::safety_bubble_damping_callback, this, std::placeholders::_1));

    hold_force_subscriber_ = get_node()->create_subscription<std_msgs::msg::Float64MultiArray>("hold_test_force", 10,
      std::bind(&CartesianImpedanceController::hold_force_callback, this, std::placeholders::_1));
    
    goal_offset_subscriber_ = get_node()->create_subscription<std_msgs::msg::Float64MultiArray>("goal_offset_topic", 10,
      std::bind(&CartesianImpedanceController::offset_callback, this, std::placeholders::_1));

    goal_hand_subscriber_ = get_node()->create_subscription<std_msgs::msg::Float64MultiArray>("hand_goal_topic", 10,
      std::bind(&CartesianImpedanceController::hand_goal_callback, this, std::placeholders::_1));

    lambda_publisher = get_node()->create_publisher<std_msgs::msg::Float64MultiArray>("inertia_matrix_topic", 10);
    D_publisher = get_node()->create_publisher<std_msgs::msg::Float64MultiArray>("damping_matrix_topic", 10);
    K_publisher = get_node()->create_publisher<std_msgs::msg::Float64MultiArray>("stiffness_matrix_topic", 10); 
    avoid_goal_publisher = get_node()->create_publisher<messages_fr3::msg::SetPose>("original_pose", 10);
    jacobian_publisher = get_node()->create_publisher<std_msgs::msg::Float64MultiArray>("jacobian_topic", 10); 
    dq_publisher = get_node()->create_publisher<std_msgs::msg::Float64MultiArray>("dq_topic", 10);
  }

  catch (const std::exception& e) {
    fprintf(stderr,  "Exception thrown during publisher creation at configure stage with message : %s \n",e.what());
    return CallbackReturn::ERROR;
    }


  RCLCPP_DEBUG(get_node()->get_logger(), "configured successfully");
  return CallbackReturn::SUCCESS;
}


CallbackReturn CartesianImpedanceController::on_activate(
  const rclcpp_lifecycle::State& /*previous_state*/) {
  franka_robot_model_->assign_loaned_state_interfaces(state_interfaces_);

  std::array<double, 16> initial_pose = franka_robot_model_->getPoseMatrix(franka::Frame::kEndEffector);
  Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_pose.data()));
  position_d_ = initial_transform.translation();
  orientation_d_ = Eigen::Quaterniond(initial_transform.rotation());
  std::cout << "Completed Activation process" << std::endl;
  return CallbackReturn::SUCCESS;
}


controller_interface::CallbackReturn CartesianImpedanceController::on_deactivate(
  const rclcpp_lifecycle::State& /*previous_state*/) {
  franka_robot_model_->release_interfaces();
  return CallbackReturn::SUCCESS;
}

std::array<double, 6> CartesianImpedanceController::convertToStdArray(const geometry_msgs::msg::WrenchStamped& wrench) {
    std::array<double, 6> result;
    result[0] = wrench.wrench.force.x;
    result[1] = wrench.wrench.force.y;
    result[2] = wrench.wrench.force.z;
    result[3] = wrench.wrench.torque.x;
    result[4] = wrench.wrench.torque.y;
    result[5] = wrench.wrench.torque.z;
    return result;
}

void CartesianImpedanceController::topic_callback(const std::shared_ptr<franka_msgs::msg::FrankaRobotState> msg) {
  O_F_ext_hat_K = convertToStdArray(msg->o_f_ext_hat_k);
  arrayToMatrix(O_F_ext_hat_K, O_F_ext_hat_K_M);
}

void CartesianImpedanceController::updateJointStates() {
  for (auto i = 0; i < num_joints; ++i) {
    const auto& position_interface = state_interfaces_.at(2 * i);
    const auto& velocity_interface = state_interfaces_.at(2 * i + 1);
    assert(position_interface.get_interface_name() == "position");
    assert(velocity_interface.get_interface_name() == "velocity");
    q_(i) = position_interface.get_value();
    dq_(i) = velocity_interface.get_value();
  }
}

void CartesianImpedanceController::f_safety_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg){
  //RCLCPP_INFO(get_node()->get_logger(), "Received message values: [%f, %f, %f, %f, %f, %f]",
    //msg->data[0], msg->data[1], msg->data[2], msg->data[3], msg->data[4], msg->data[5]);
  for (size_t j = 0; j < 6; ++j) {
        F_safety(j) = msg->data[j];
     }
}

void CartesianImpedanceController::hand_goal_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg){
  for (size_t j = 0; j < 3; ++j) {
        position_d_target_(j) = msg->data[j];
     }
  RCLCPP_INFO(get_node()->get_logger(), "hold goal msg values: [%f, %f, %f]",
                msg->data[0], msg->data[1], msg->data[2]); 
}

void CartesianImpedanceController::hold_stiffness_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg){
  for (size_t i = 0; i < 6; ++i) {
        for (size_t j = 0; j < 6; ++j) {
            K_hold(i, j) = msg->data[i * 6 + j];
        }
    }
}

void CartesianImpedanceController::safety_bubble_damping_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg){
  for (size_t i = 0; i < 6; ++i) {
        for (size_t j = 0; j < 6; ++j) {
            D_h(i, j) = msg->data[i * 6 + j];
        }
    }
}

void CartesianImpedanceController::hold_force_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg){
 RCLCPP_INFO(get_node()->get_logger(), "hold force msg values: [%f, %f, %f, %f, %f, %f]",
                msg->data[0], msg->data[1], msg->data[2],
                msg->data[3], msg->data[4], msg->data[5]); 
  for (size_t j = 0; j < 6; ++j) {
        F_hold(j) = msg->data[j];
     }
}

void CartesianImpedanceController::offset_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg){
  for (size_t j = 0; j < 3; ++j) {
        Offset(j) = msg->data[j];
     }
}

controller_interface::return_type CartesianImpedanceController::update(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {  
  // if (outcounter == 0){
  // std::cout << "Enter 1 if you want to track a desired position or 2 if you want to use free floating with optionally shaped inertia" << std::endl;
  // std::cin >> mode_;
  // std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
  // std::cout << "Mode selected" << std::endl;
  // while (mode_ != 1 && mode_ != 2){
  //   std::cout << "Invalid mode, try again" << std::endl;
  //   std::cin >> mode_;
  // }
  // }
  std::array<double, 49> mass = franka_robot_model_->getMassMatrix();
  std::array<double, 7> coriolis_array = franka_robot_model_->getCoriolisForceVector();
  std::array<double, 42> jacobian_array =  franka_robot_model_->getZeroJacobian(franka::Frame::kEndEffector);
  std::array<double, 16> pose = franka_robot_model_->getPoseMatrix(franka::Frame::kEndEffector);
  Eigen::Map<Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
  Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
  
  auto jacobian_msg = std_msgs::msg::Float64MultiArray();
  for(long unsigned int i = 0; i < jacobian_array.size(); ++i){
    jacobian_msg.data.push_back(jacobian_array[i]);
  }
  auto dq_msg = std_msgs::msg::Float64MultiArray();
  for(size_t i = 0; i < 7; ++i){
    dq_msg.data.push_back(dq_(i));
  }
  jacobian_publisher->publish(jacobian_msg);
  dq_publisher->publish(dq_msg);

  Eigen::Map<Eigen::Matrix<double, 7, 7>> M(mass.data());
  Eigen::Affine3d transform(Eigen::Matrix4d::Map(pose.data()));
  Eigen::Vector3d position(transform.translation());
  Eigen::Quaterniond orientation(transform.rotation());
  orientation_d_target_ = Eigen::AngleAxisd(rotation_d_target_[0], Eigen::Vector3d::UnitX())
                        * Eigen::AngleAxisd(rotation_d_target_[1], Eigen::Vector3d::UnitY())
                        * Eigen::AngleAxisd(rotation_d_target_[2], Eigen::Vector3d::UnitZ());
  updateJointStates(); 

  
  error.head(3) << position - position_d_ /*- Offset*/; 
  if (orientation_d_.coeffs().dot(orientation.coeffs()) < 0.0) {
    orientation.coeffs() << -orientation.coeffs();
  }
  Eigen::Quaterniond error_quaternion(orientation.inverse() * orientation_d_);
  error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
  error.tail(3) << -transform.rotation() * error.tail(3);
  I_error += Sm * dt * integrator_weights.cwiseProduct(error);
  for (int i = 0; i < 6; i++){
    I_error(i,0) = std::min(std::max(-max_I(i,0),  I_error(i,0)), max_I(i,0)); 
  }
/* maximum_error_necessary = false;
      error_factor = 1;
      max_error_factor = 1;
      
      for (size_t i = 0; i < 3; i++){
        if (error(i) > max_error){
          maximum_error_necessary = true;
          error_factor = max_error/error(i);
          if (max_error_factor > error_factor){
            max_error_factor = error_factor;
          }
        }
      }
      if (maximum_error_necessary){
        for (size_t i = 0; i < 3; i++){
          scaled_error(i) = error(i) * max_error_factor;
        }
        for (size_t i = 3; i < 6; i++){
          scaled_error(i) = error(i);
        }
       }
      else*/ scaled_error = error;

  avoid_goal_publisher->publish(position_d_target_msg);
  Lambda = (jacobian * M.inverse() * jacobian.transpose()).inverse();
  // Theta = T*Lambda;
  // F_impedance = -1*(Lambda * Theta.inverse() - IDENTITY) * F_ext;
  //Inertia of the robot
  
  //publishes lambda Matrix
  int index_lambda = 0;
  for(int i = 0; i < 6; i++) {
    for(int j = 0; j < 6; j++) {
      lambda_array[index_lambda++] = Lambda(i, j);
    }
  } 
  auto lambda_msg = std_msgs::msg::Float64MultiArray();
  for(long unsigned int i = 0; i < lambda_array.size(); ++i){
    lambda_msg.data.push_back(lambda_array[i]);
  }
  lambda_publisher->publish(lambda_msg);
  
  //publishes damping Matrix
  int index_D = 0;
  for(int i = 0; i < 6; i++) {
    for(int j = 0; j < 6; j++) {
      D_array[index_D++] = D(i, j);
    }
  } 
  auto D_msg = std_msgs::msg::Float64MultiArray();
  for(long unsigned int i = 0; i < D_array.size(); ++i){
    D_msg.data.push_back(D_array[i]);
  }
  D_publisher->publish(D_msg);
  
  switch (mode_)
  {
  case 1:
    Theta = Lambda;
    if (!K_hold.isZero()){
      F_impedance = -1 * (D_h * (jacobian * dq_) + K_hold * error  /*+ I_error*/) + F_hold; 
    }
    else{
      F_impedance = -1 * ((D+D_h) * (jacobian * dq_) + K * scaled_error /*+ I_error*/) + F_safety;
      }
    break;

  case 2:
    Theta = T*Lambda;
    F_impedance = -1*(Lambda * Theta.inverse() - IDENTITY) * F_ext;
    break;
  
  default:
    break;
  }
 // RCLCPP_INFO(get_node()->get_logger(), "F_impedance: [%f, %f, %f, %f, %f, %f]",
 //   F_impedance[0], F_impedance[1], F_impedance[2], F_impedance[3], F_impedance[4], F_impedance[5]);
  F_ext = 0.9 * F_ext + 0.1 * O_F_ext_hat_K_M; //Filtering 
  I_F_error += dt * Sf* (F_contact_des - F_ext);
  F_cmd = Sf*(0.4 * (F_contact_des - F_ext) + 0.9 * I_F_error + 0.9 * F_contact_des);

  Eigen::VectorXd tau_task(7), tau_nullspace(7), tau_d(7), tau_impedance(7);
  pseudoInverse(jacobian.transpose(), jacobian_transpose_pinv);

  tau_nullspace << (Eigen::MatrixXd::Identity(7, 7) -
                    jacobian.transpose() * jacobian_transpose_pinv) *
                    (nullspace_stiffness_ * config_control * (q_d_nullspace_ - q_) - //if config_control = true we control the whole robot configuration
                    (2.0 * sqrt(nullspace_stiffness_)) * dq_);  // if config control ) false we don't care about the joint position

  tau_impedance = jacobian.transpose() * Sm * (F_impedance /*+ F_repulsion + F_potential*/) + jacobian.transpose() * Sf * F_cmd;
  auto tau_d_placeholder = tau_impedance + tau_nullspace + coriolis; //add nullspace and coriolis components to desired torque
  tau_d << tau_d_placeholder;
  tau_d << saturateTorqueRate(tau_d, tau_J_d_M);  // Saturate torque rate to avoid discontinuities
  tau_d << saturateTorque(tau_d);
  tau_J_d_M = tau_d;

  for (size_t i = 0; i < 7; ++i) {
    command_interfaces_[i].set_value(tau_d(i));
  }
  
  if (outcounter % 1000/update_frequency == 0){
    std::cout << "F_ext_robot [N]" << std::endl;
    std::cout << O_F_ext_hat_K << std::endl; 
    std::cout << O_F_ext_hat_K_M << std::endl;
    std::cout << "Lambda  Thetha.inv(): " << std::endl;
    std::cout << Lambda*Theta.inverse() << std::endl;
    std::cout << "tau_d" << std::endl;
    std::cout << tau_d << std::endl;
    std::cout << "--------" << std::endl;
    std::cout << tau_nullspace << std::endl;
    std::cout << "--------" << std::endl;
    std::cout << tau_impedance << std::endl;
    std::cout << "--------" << std::endl;
    std::cout << coriolis << std::endl;
    std::cout << "Inertia scaling [m]: " << std::endl;
    std::cout << T << std::endl;
  } 
  outcounter++;
  update_stiffness_and_references();
  return controller_interface::return_type::OK;
}
}

// namespace cartesian_impedance_control
#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(cartesian_impedance_control::CartesianImpedanceController,
                       controller_interface::ControllerInterface)