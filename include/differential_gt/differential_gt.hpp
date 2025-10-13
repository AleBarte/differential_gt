#ifndef DIFFERENTIAL_GT_HPP
#define DIFFERENTIAL_GT_HPP

#include "rclcpp/rclcpp.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <filesystem>
#include "../include/differential_gt/cgt.hpp"
#include "../include/differential_gt/ncgt.hpp"
#include "../include/differential_gt/arbitration.hpp"
#include "../include/differential_gt/utils.hpp"
#include "../include/differential_gt/moe_goal_inference.hpp"
#include <eigen3/Eigen/Dense>
#include <chrono>
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"                 // For Twist messages
#include "std_msgs/msg/float64_multi_array.hpp"                // For Float64MultiArray messages
#include "std_msgs/msg/float32_multi_array.hpp"                // For Float32MultiArray messages
#include "std_msgs/msg/int32.hpp"                              // For Int32 messages
#include "sensor_msgs/msg/joy.hpp"                             // For joystick input
#include <tf2_ros/transform_listener.h>                        // TF2 Transform listener
#include <tf2_ros/buffer.h>                                    // TF2 Buffer
#include <tf2/LinearMath/Quaternion.h>                         // TF2 Quaternion math
#include <tf2/LinearMath/Transform.h>                          // TF2 Transform math

#define GRIPPER_OFFSET 0.1629 // Offset from the end effector to the tool tip in meters

namespace fs = std::filesystem;

class DifferentialGT : public rclcpp::Node
{
public:
    DifferentialGT(const std::string &node_name);

private:

    bool Startup();

    // Callbacks
    void WrenchFromHOCallback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg);          // Takes the force from the joystick and stores it for later use
    void PoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);                    // Takes the pose of the robot End-Effector and stores it for later use
    void TwistCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
    void TwistFromSafetyFilterCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg);  // Takes a twist from a Safety Filter and stores it for later use (Marco you don't care about this)
    void ButtonsCallback(const sensor_msgs::msg::Joy::SharedPtr msg);                           // Takes the values of the buttons on the joystick and stores them for later use
    void TargetCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);                 // Takes the target position and stores it for later use
    void ObstaclesCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);              // Takes the obstacles positions and radii and stores them for later use
    void CylindricalObstaclesCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);  // Takes the cylindrical obstacles data and stores it for later use
    void CylinderBaseCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);          // Takes the cylinder base obstacle data and stores it for later use

    // Functions
    void SetSystemMatrices();   // Sets matrices for the Mass-Spring-Damper (MSD) system. Needed to compute the CGT and NCGT gains
    void SetCostMatrices();     // Sets the various cost matrices for CGT and NCGT
    void Publish();             // Calls all the publishers to publish respective messages
    void ComputeACSAction();    // Computes the Automatic Control System Action

    
    void ComputeReferences(Eigen::VectorXd &ref_h, Eigen::VectorXd &ref_r); //Computes references for HO and ACS (Marco you can change this function)
    Eigen::VectorXd ComputeRepulsiveForce(const Eigen::VectorXd &obsatcle, const double &radius);
    Eigen::VectorXd ComputeRepulsiveForce(const std::vector<Eigen::VectorXd> &obstacles, const std::vector<double> &radii);
    Eigen::VectorXd SelectGoal(const std::vector<Eigen::VectorXd> &goals, const Eigen::VectorXd &uh, double &scaling);
    Eigen::VectorXd ComputeRepulsiveForce(const Eigen::VectorXd &cylinder_center, const double &radius, const double &height);
    Eigen::VectorXd ComputeCylindricalRepulsiveForce(const std::vector<Eigen::VectorXd> &cylinder_centers, 
                                                 const std::vector<double> &radii, 
                                                 const std::vector<double> &heights);


    // Publishers
    rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_from_acs_pub_;               // Publisher for the ACS wrench action
    rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_from_ho_pub_;                // Publisher fot the HO wrench action (force from joystick)
    rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr feedback_wrench_pub_;               // Publisher for the feedback wrench (force feedback to joystick)
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr selected_goal_pub_;                              // Publisher for the selected goal index
    

    // Subscribers
    rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_from_ho_sub_;             // Subscribes to the force commanded by the joystick
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;                         // Subscribes to the robot end effector pose
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_sub_;                       // Subscribes to the robot end effector twist
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_from_safety_filter_sub_;    // Subscribes to twist from safety filter (Marco you can delete this or discard it)
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr buttons_sub_;                                // Subcribes to the buttons of the joystick
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr target_sub_;                      // Subscribes to the topic where the target positions are published
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr obstacles_sub_;                   // Subscribes to the topic where the obstacles positions are published    
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr cylindrical_obstacles_sub_;      // Subscribes to the topic where the cylindrical obstacles data is published
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr cylinder_base_sub_;              // Subscribes to the topic where the cylinder base obstacle data is published

    // Messages to save data from subscribers
    geometry_msgs::msg::WrenchStamped wrench_from_ho_msg_;                                              // HO force in [N]
    Eigen::VectorXd position_;                                                                          // End Effector (EE) Position [m]
    Eigen::Matrix3d orientation_;                                                                       // End Effector (EE) Orientation (Rotation Matrix)
    Eigen::VectorXd linear_velocity_ = Eigen::VectorXd::Zero(3);                                        // Initialize EE linear velocity to zero [m/s]
    Eigen::Vector3d twist_from_safety_filter_ = Eigen::Vector3d::Zero();                                // Initialize twist from safety filter to zero [m/s] (Marco can discard)

    // Messages to publish
    geometry_msgs::msg::WrenchStamped wrench_from_acs_msg_;                                             // ACS wrench to be applied to robot EE [N] and [Nm]                                             
    geometry_msgs::msg::WrenchStamped wrench_ho_topub_msg_;                                             // HO wrench to be aplied to robot EE [N] and [Nm]
    geometry_msgs::msg::WrenchStamped feedback_wrench_msg_;                                             // Feedback wrench to be applied to joystick [N] and [Nm]
    std_msgs::msg::Int32                selected_goal_msg_;                                             // Selected goal index
    

    // Game Theory Objects
    CoopGT coop_gt_;        // CGT
    NonCoopGT noncoop_gt_;  // NCGT

    // Arbitration Object
    Arbitration arbitration_;

    // Arbitration level and other variables (Note this code is WIP, these may or may not be used)
    double alpha_;
    int cosine_similarity_counter_ = 0;
    int decision_ = 0;                      // Decision made by the arbitration
    int prev_decision_ = 0;
    double cos_theta_ = 0.0;                // Cosine similarity value
    double cos_theta_coop_ = 0.0;           // Cosine similarity value for cooperative action
    double cos_theta_nc_ = 0.0;             // Cosine similarity value for non-cooperative action

    // Matrices for game theory calculations
    Eigen::MatrixXd Qhh_;
    Eigen::MatrixXd Qhr_;
    Eigen::MatrixXd Qrr_;
    Eigen::MatrixXd Qrh_;
    Eigen::MatrixXd Qh_;
    Eigen::MatrixXd Qr_;
    Eigen::MatrixXd Rh_ ;
    Eigen::MatrixXd Rr_ ;
    Eigen::MatrixXd Rrr_;
    Eigen::MatrixXd Rhh_;
    Eigen::MatrixXd Rhr_;
    Eigen::MatrixXd Rrh_;


    // For Mixing
    Eigen::MatrixXd default_Qr_;
    Eigen::MatrixXd K_ncgt_default_a_;
    Eigen::MatrixXd K_ncgt_default_h_;
    Eigen::MatrixXd K_ncgt_ped_a_;
    Eigen::MatrixXd K_ncgt_ped_h_;

    // System Matrices
    Eigen::MatrixXd A_; // System matrix 
    Eigen::MatrixXd B_; // Input matrix
    
    // Safety Filter (Marco you can delete or ignore)
    Eigen::MatrixXd F_; // System matrix
    Eigen::MatrixXd G_; // Input Matrix
    Eigen::Vector3d z_; // State of the filter

    // Gains of the controllers (CGT & NCGT)
    Eigen::MatrixXd K_cgt_;
    Eigen::MatrixXd K_ncgt_a_;
    Eigen::MatrixXd K_ncgt_h_;

    // Goal
    Eigen::VectorXd goal_position_ = Eigen::VectorXd::Zero(3); // Goal position [m]

    // Vector Containing the Obstacles Positions and Obstacles radii
    std::vector<Eigen::VectorXd> obstacle_vec_;
    std::vector<double> obstacle_radius_vec_;
    std::vector<Eigen::VectorXd> cylindrical_obstacle_centers_;
    std::vector<double> cylindrical_obstacle_radii_;
    std::vector<double> cylindrical_obstacle_heights_;

    // Cylinder base obstacle
    Eigen::VectorXd cylinder_base_center_;
    double cylinder_base_radius_;
    double cylinder_base_height_;
    bool has_cylinder_base_ = false;


    // Parameters
    std::string ho_wrench_topic_;           // Topic from which the HO wrench is read
    std::string acs_wrench_pub_topic_;      // Topic on which the ACS wrench is published
    std::string pose_topic_;                // Topic from which EE pose is read
    std::string twist_topic_;               // Topic from which EE twist is read
    std::string ho_wrench_pub_topic_;       // Topic on which the HO wrench is published
    std::string base_frame_;                // Base frame for the robot, can be set as a parameter
    std::string end_effector_;              // End-effector frame for the robot, can be set as a paraeter
    double switch_on_point_;                // Switch on point for Cosine Similarity Hysteresis
    double switch_off_point_;               // Switch off point for Cosien Similarity Hysteresis
    double publishing_rate_;                // Default publishing rate in seconds
    bool override_ho_wrench_;               // Flag to override the HO wrench with ACS action
    std::string save_matrices_;             // Flag to save matrices to a file
    std::string load_matrices_;             // Flag to load previously saved configurations
    bool mix_;                              // Mix Pedrocchi control action with default


    // TF2
    tf2_ros::Buffer tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // Common timer object
    rclcpp::TimerBase::SharedPtr timer_;

    // Flags
    bool is_initialized_ = false; // Flag to check if the node is initialized
    bool button_pressed_ = false; // Flag to check whether a button is pressed or not
    bool potential_active_ = false; // Flag to check if the potential is active

    double stored_energy_ = 0.0;

    // Debugging
    Eigen::Vector3d initial_position_; // Initial position of the end effector
    int previous_index_ = -1;
    double d0_;


    Eigen::VectorXd acs_ref_; // Reference trajectory for the ACS
    Eigen::VectorXd ho_ref_; // Reference trajectory for the HO 
    Eigen::VectorXd filtered_posterior_ = Eigen::VectorXd::Zero(NUM_TARGETS); // Filtered posterior for the MoE //! Targets must be known a priori
    Eigen::VectorXd filtered_acs_wrench_ = Eigen::VectorXd::Zero(3); // Filtered ACS wrench

    //? Debugging ------------------------------------------------------------------------
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr ref_ho_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr ref_acs_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr cos_theta_pub_;
    rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr ho_ncgt_wrench_pub_;
    rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr acs_ncgt_wrench_pub_;
    rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr ho_cgt_wrench_pub_;
    rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr acs_cgt_wrench_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr decision_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr tank_level_pub_;

    geometry_msgs::msg::PoseStamped ref_ho_msg_;
    geometry_msgs::msg::PoseStamped ref_acs_msg_;
    geometry_msgs::msg::WrenchStamped ho_ncgt_wrench_msg_;
    geometry_msgs::msg::WrenchStamped acs_ncgt_wrench_msg_;
    geometry_msgs::msg::WrenchStamped ho_cgt_wrench_msg_;
    geometry_msgs::msg::WrenchStamped acs_cgt_wrench_msg_;
    std_msgs::msg::Float64MultiArray cos_theta_msg_;
    std_msgs::msg::Int32 decision_msg_;
    std_msgs::msg::Float64MultiArray tank_level_msg_;
    //?------------------------------------------------------------------------------------

    static constexpr int NUM_TARGETS = 1;

};
#endif // DIFFERENTIAL_GT_HPP
