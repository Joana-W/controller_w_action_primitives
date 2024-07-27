
#include "cartesian_impedance_control/cartesian_impedance_controller.hpp"

#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/bool.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include "franka_msgs/msg/franka_robot_state.hpp"
#include "messages_fr3/srv/set_pose.hpp"
#include "messages_fr3/msg/set_pose.hpp"
#include <franka_hardware/franka_hardware_interface.hpp>
#include <franka_hardware/model.hpp>
#include "franka_semantic_components/franka_robot_model.hpp"
#include "franka_semantic_components/franka_robot_state.hpp"


#include <franka/model.h>
#include <franka/robot.h>
#include <franka/robot_state.h>

#include <rclcpp/rclcpp.hpp>

#include <cmath>
#include <exception>
#include <string>

#include <Eigen/Eigen>



class SafetyBubble : public rclcpp::Node, public cartesian_impedance_control::CartesianImpedanceController {
    public:
    SafetyBubble(); //Konstruktor der Klasse   

    void external_force();
    void MatrixtoArray(Eigen::Matrix<double, 6, 1>& F_ext); //Initalizes the function which makes an array from the external force-matrix

    void hand_position_callback(const messages_fr3::msg::SetPose::SharedPtr msg); //Initalize Callback function for current hand Position    

    int chosenPrimitive = 1; //[1]=Avoid, [2]=Follow, [3]=Hold
    double R = 0.15; //Outer Radius of the Safety Bubble [m]
    double r_eq = 0.05; //Inner Radius of the Safety Bubble [m]

    private:
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Subscription<franka_msgs::msg::FrankaRobotState>::SharedPtr franka_state_subscriber = nullptr;

    void StateCallback(const franka_msgs::msg::FrankaRobotState::SharedPtr msg);

    void jacobian_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    void inertia_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg); //callback function for the inertia matrix

    //cartesian_impedance_control::CartesianImpedanceController controller_;

    std::unique_ptr<franka_semantic_components::FrankaRobotModel> franka_robot_model_; //needed to get the jacobian

    geometry_msgs::msg::PoseStamped current_position_; //variable to store the most recent position
    geometry_msgs::msg::PoseStamped inital_position_;
    geometry_msgs::msg::PoseStamped previous_position_;

    bool inital_position_bool;
    geometry_msgs::msg::TwistStamped current_velocity_; //variable to store the most recent velocity
    sensor_msgs::msg::JointState current_joint_state_;

    messages_fr3::msg::SetPose current_hand_position_; //variable to store the most recent hand position

    Eigen::Matrix<double, 7, 1> dq; //vector containing the current velocity in joint space

    Eigen::Matrix<double, 6, 1> position_vector; //vector containing the current position
    Eigen::Matrix<double, 6, 1> velocity_vector; //vector containing the current velocity
    Eigen::Matrix<double, 6, 1> inital_velocity_vector; //vector containing the inital velocity
    Eigen::Matrix<double, 6, 1> velocity;


    Eigen::Matrix<double, 6, 7> jacobian;

    std::array<double, 6> offset_array = {0,0,0,0,0,0};

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr f_ext_publisher_ = nullptr; //initalized the publisher for the external force
    rclcpp::Publisher<messages_fr3::msg::SetPose>::SharedPtr goal_publisher_ = nullptr; //initalized the publisher for the new goal in case of follow and hold
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr hold_stiffness_publisher_ = nullptr;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr P_publisher_ = nullptr;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr D_h_publisher_ = nullptr;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr offset_publisher_ = nullptr;

    rclcpp::Subscription<messages_fr3::msg::SetPose>::SharedPtr test_hand_position_subscriber_ = nullptr; //initalized the subscriber used to include the test hand position    
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr jacobian_subscriber; 
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr dq_subscriber; 
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr M_subscriber_; //subscribes to the inertia matrix/msg/array



    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr test_results_publisher_ = nullptr;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr test_end_publisher_ = nullptr;


    //Initalize the Safety_Bubble Stiffness and Damping Matrix:
    Eigen::Matrix<double, 6, 6> safety_bubble_stiffness =  (Eigen::Matrix<double, 6,6>() << 0,   0,   0,   0,   0,   0,
                                                                0, 0,   0,   0,   0,   0,
                                                                0,   0, 0,   0,   0,   0,  // impedance stiffness term
                                                                0,   0,   0, 0,   0,   0,
                                                                0,   0,   0,   0, 0,   0,
                                                                0,   0,   0,   0,   0,  0).finished();
    Eigen::Matrix<double, 6, 6> safety_bubble_stiffness_virtual =  (Eigen::Matrix<double, 6,6>() << 0,   0,   0,   0,   0,   0,
                                                                0, 0,   0,   0,   0,   0,
                                                                0,   0, 0,   0,   0,   0,  // impedance virtual stiffness term
                                                                0,   0,   0, 0,   0,   0,
                                                                0,   0,   0,   0, 0,   0,
                                                                0,   0,   0,   0,   0,  0).finished();

    Eigen::Matrix<double, 6, 6> safety_bubble_damping =  (Eigen::Matrix<double, 6,6>() <<  0,   0,   0,   0,   0,   0,
                                                                0,  0,   0,   0,   0,   0,
                                                                0,   0,  0,   0,   0,   0,  // impedance damping term
                                                                0,   0,   0,   0,   0,   0,
                                                                0,   0,   0,   0,   0,   0,
                                                                0,   0,   0,   0,   0,   0).finished();
    Eigen::Matrix<double, 6, 1> F_ext =  (Eigen::Matrix<double, 6,1>() <<  0,   0,   0,   0,   0,   0).finished(); //Initalize the Matrix with the external force
    Eigen::Matrix<double, 6, 6> Lambda;

    bool test_end = false;
    bool inital_velocity_bool = false;

    std::array<double, 6> F_ext_array; //array containing the external force, since we can't publish a matrix
    std::array<double, 36> stiffness_hold_array;
    std::array<double, 36> P_array;
    std::array<double, 36> D_h_array;
    Eigen::Vector3d goal_pose;

    double theta = 0;


    int r0; //absolute value of the starting position of the endeffector
    int alpha = 0; //ration between starting position and goal position
    int G; //absolute value of the goal position

    bool inside; //boolean, which is used for testing purposes, true = endeffector is inside of the inner radius

    Eigen::Matrix<double, 6, 1> R_vector; //Parametrization of the outer radius of the safety_bubble

    Eigen::Matrix<double, 6, 1> external_force_assumption = (Eigen::Matrix<double, 6, 1>() <<  25,   25,   25,   25,   25,   25).finished(); //In the case of hold, we assume a force of 25N in each direction
    Eigen::Matrix<double, 6, 1> max_movement = (Eigen::Matrix<double, 1, 6>() << 0.05, 0.05, 0.05, 0, 0, 0).finished(); //in the case of hold, the ee should move maximal this distance
    Eigen::Matrix<double, 6, 6> stiffness_hold;

};


class TestInput : public rclcpp::Node{
    public:
    TestInput(); //Constructor of the class Test Input

    void test_input(); //void function which defines/contains the test hand positions

    private:
    rclcpp::Publisher<messages_fr3::msg::SetPose>::SharedPtr test_hand_position_publisher_; //initalized the publisher used to include the test hand position
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr hold_force_publisher_; //initalized the publisher used to publish the force acting when the ActionPrimitive Hold is chosen
    bool force_time = true;

    double force_hold_x = 25;
    double force_hold_y = 25;
    double force_hold_z = 25;

    int chosenPrimitive = 1;

};

class TestResult : public rclcpp::Node {
    public:
    TestResult();

    private:
    void results_callback(const std_msgs::msg::Bool::SharedPtr msg); //callback for inside parameter
    void test_end_callback(const std_msgs::msg::Bool::SharedPtr msg); //callback for test_end parameter
    void new_pose_callback(const messages_fr3::msg::SetPose::SharedPtr msg); //callback for new goal position
    void goal_callback(const messages_fr3::msg::SetPose::SharedPtr msg);

    void K_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg); //callback function for the stiffness matrix
    void inertia_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg); //callback function for the inertia matrix
    void D_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg); //callback function for the damping matrix
    void D_h_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    void P_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    void jacobian_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);


    void state_callback(const franka_msgs::msg::FrankaRobotState::SharedPtr msg);

    void store_positions();

    void Frequency_Analysis();
    void save_to_csv();

    bool inside_check(geometry_msgs::msg::Point& position, std::array<double, 3>& hand_pose);
    bool reached_goal_check(geometry_msgs::msg::Point& position, std::array<double, 3>& goal_pose);
    bool outside_check(geometry_msgs::msg::Point& position, std::array<double, 3>& hand_pose);


    Eigen::Matrix<std::complex<double>, 6, 6> Transfer_function(double omega, const Eigen::Matrix<double, 6, 6>& M, const Eigen::Matrix<double, 6, 6>& D,
    const Eigen::Matrix<double, 6, 6>& K); //function which calculates the transferfunction of the systems

    //rclcpp::Subscription<messages_fr3::msg::SetPose>::SharedPtr safety_bubble_new_position_subscriber;
    rclcpp::Subscription<messages_fr3::msg::SetPose>::SharedPtr goal_subscriber;
    rclcpp::Subscription<messages_fr3::msg::SetPose>::SharedPtr hand_position_subscriber_; 
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr test_end_subscriber_;


    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr K_subscriber_; //subscribes to the stiffness matrix/msg/array
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr M_subscriber_; //subscribes to the inertia matrix/msg/array
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr D_subscriber_; //subscribes to the damping matrix/msg/array
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr D_h_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr P_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr jacobian_subscriber; 


    rclcpp::Subscription<franka_msgs::msg::FrankaRobotState>::SharedPtr franka_state_subscriber = nullptr; //subscribes to robot states
    std::unique_ptr<franka_semantic_components::FrankaRobotModel> franka_robot_model_; //needed to get jacobian


    Eigen::Matrix<double, 6, 6> D;
    Eigen::Matrix<double, 6, 6> M;
    Eigen::Matrix<double, 6, 6> K;
    Eigen::Matrix<double, 6, 6> P;
    Eigen::Matrix<double, 6, 6> D_h;

    std::array<double, 3> goal_pose;
    std::array<double, 3> hand_pose;

    std::vector<double> path_x;
    std::vector<double> path_y;
    std::vector<double> path_z;

    Eigen::Matrix<double, 6, 7> jacobian;


    //SafetyBubble bubble_;
    geometry_msgs::msg::PoseStamped inital_position_;
    geometry_msgs::msg::PoseStamped current_position_;
    sensor_msgs::msg::JointState current_joint_state_;
    sensor_msgs::msg::JointState inital_joint_state_ ;

    bool inital_position_bool = false;
    bool inside; //boolean, which is used for testing purposes, true = endeffector is inside of the inner radius
    bool test_end; //boolean, which tells whether the endeffector is at the end (either goal or no more movement)
    bool goal_reached;
    bool previous_test_bool;

    double max_phase_x;
    double max_phase_y;
    double max_phase_z;

    geometry_msgs::msg::PoseStamped last_position_;
    bool initialized_ = false;
    double total_distance;

    int chosenPrimitive = 1; //[1]=Avoid, [2]=Follow, [3]=Hold
    double R = 0.15; //Outer Radius of the Safety Bubble [m]
    double r_eq = 0.05; //Inner Radius of the Safety Bubble [m]

    double force_hold_x = 25;
    double force_hold_y = 25;
    double force_hold_z = 25;

};