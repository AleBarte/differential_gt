#ifndef DIFFERENTIAL_GT_HPP
#define DIFFERENTIAL_GT_HPP

#include "rclcpp/rclcpp.hpp"
#include "../include/differential_gt/cgt.hpp"
#include "../include/differential_gt/ncgt.hpp"
#include "../include/differential_gt/arbitration.hpp"
#include <eigen3/Eigen/Dense>
#include <chrono>
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"                 // For Twist messages
#include "std_msgs/msg/float64_multi_array.hpp"                // For Float64MultiArray messages
#include "std_msgs/msg/int32.hpp"                            // For Int32 messages
#include "sensor_msgs/msg/joy.hpp"                             // For joystick input
#include <tf2_ros/transform_listener.h>                        // TF2 Transform listener
#include <tf2_ros/buffer.h>                                    // TF2 Buffer
#include <tf2/LinearMath/Quaternion.h>                         // TF2 Quaternion math
#include <tf2/LinearMath/Transform.h>                          // TF2 Transform math

class DifferentialGT : public rclcpp::Node
{
public:
    DifferentialGT(const std::string &node_name);

private:

    bool Startup();

    // Callbacks
    void WrenchFromHOCallback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg);
    
    void PoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void TwistCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
    void DesiredEEVelCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg); //TODO: Remove this
    void TwistFromSafetyFilterCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
    void ButtonsCallback(const sensor_msgs::msg::Joy::SharedPtr msg); // For joystick input, if needed
    
    // Functions
    void SetSystemMatrices();
    void SetCostMatrices();
    void Publish();
    void ComputeACSAction();

    //TODO Maybe reomove -----------------
    // void ComputeTrajectories();
    // void ComputeLinearTrajectory();
    //TODO---------------------------------
    
    void ComputeReferences(Eigen::VectorXd &ref_h, Eigen::VectorXd &ref_r);


    // Publishers
    rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_from_acs_pub_;
    rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_from_ho_pub_;
    

    // Subscribers
    rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_from_ho_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_sub_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr desired_ee_vel_sub_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_from_safety_filter_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr buttons_sub_; // For joystick input, if needed

    // Messages to save data from subscribers
    geometry_msgs::msg::WrenchStamped wrench_from_ho_msg_;
    Eigen::Vector3d position_;
    Eigen::Matrix3d orientation_;
    Eigen::Vector3d linear_velocity_ = Eigen::Vector3d::Zero(); // Initialize linear velocity to zero
    Eigen::Vector3d desired_ee_vel_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d twist_from_safety_filter_ = Eigen::Vector3d::Zero(); // Initialize twist from safety filter to zero

    // Messages to publish
    geometry_msgs::msg::WrenchStamped wrench_from_acs_msg_;
    geometry_msgs::msg::WrenchStamped wrench_ho_topub_msg_;
    

    // Game Theory Objects
    CoopGT coop_gt_; 
    NonCoopGT noncoop_gt_;

    // Arbitration Object
    Arbitration arbitration_;

    // Arbitration level
    double alpha_;
    int cosine_similarity_counter_ = 0;
    int decision_ = 0; // Decision made by the arbitration
    double cos_theta_ = 0.0; // Cosine similarity value
    double cos_theta_coop_ = 0.0; // Cosine similarity value for cooperative action
    double cos_theta_nc_ = 0.0; // Cosine similarity value for non

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

    // System Matrices
    Eigen::MatrixXd A_; // System matrix for cooperative GT
    Eigen::MatrixXd B_; // Input matrix for cooperative GT
    Eigen::MatrixXd F_;
    Eigen::MatrixXd G_; 

    Eigen::Vector3d z_;

    // Gains of the controllers
    Eigen::MatrixXd K_cgt_;
    Eigen::MatrixXd K_ncgt_a_;
    Eigen::MatrixXd K_ncgt_h_;


    // Parameters
    std::string ho_wrench_topic_;
    std::string acs_wrench_pub_topic_;
    std::string pose_topic_;
    std::string twist_topic_;
    std::string ho_wrench_pub_topic_;
    std::string base_frame_; // Base frame for the robot, can be set as a parameter
    std::string end_effector_;
    double switch_on_point_;
    double switch_off_point_;
    double publishing_rate_; // Default publishing rate in seconds
    bool override_ho_wrench_; // Flag to override the HO wrench with ACS action


    // TF2
    tf2_ros::Buffer tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // Common timer object
    rclcpp::TimerBase::SharedPtr timer_;

    // Flags
    bool is_initialized_ = false; // Flag to check if the node is initialized
    bool button_pressed_ = false; 

    // Debugging
    Eigen::Vector3d initial_position_; // Initial position of the end effector

    //TODO: Remove. These are to test marco experiment
    // int traj_index_ = 0; // Index of the current trajectory
    // Eigen::MatrixXd ho_ref_;
    // Eigen::MatrixXd acs_ref_;
    //TODO --------------------------------------------------

    Eigen::VectorXd acs_ref_; // Reference trajectory for the ACS
    Eigen::VectorXd ho_ref_; // Reference trajectory for the HO

    //TODO Remove debugging
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr ref_ho_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr ref_acs_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr cos_theta_pub_;
    rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr ho_ncgt_wrench_pub_;
    rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr acs_ncgt_wrench_pub_;
    rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr ho_cgt_wrench_pub_;
    rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr acs_cgt_wrench_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr decision_pub_;

    geometry_msgs::msg::PoseStamped ref_ho_msg_;
    geometry_msgs::msg::PoseStamped ref_acs_msg_;
    geometry_msgs::msg::WrenchStamped ho_ncgt_wrench_msg_;
    geometry_msgs::msg::WrenchStamped acs_ncgt_wrench_msg_;
    geometry_msgs::msg::WrenchStamped ho_cgt_wrench_msg_;
    geometry_msgs::msg::WrenchStamped acs_cgt_wrench_msg_;
    std_msgs::msg::Float64MultiArray cos_theta_msg_;
    std_msgs::msg::Int32 decision_msg_;


};
#endif // DIFFERENTIAL_GT_HPP
