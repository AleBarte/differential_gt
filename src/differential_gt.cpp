#include "../include/differential_gt/differential_gt.hpp"

//*----------------------------*//
//*------- Constructor --------*//
//*----------------------------*//

DifferentialGT::DifferentialGT(const std::string &node_name)
    : Node(node_name),
      tf_buffer_(this->get_clock()),
      tf_listener_(std::make_shared<tf2_ros::TransformListener>(tf_buffer_)),
      coop_gt_(3, 0.01), // Initialize CoopGT with 3 DoFs and 0.01 seconds time step
      noncoop_gt_(3, 0.01) // Initialize NonCoopGT with 3 DoFs and 0.01 seconds time step
{
    // Declare parameters
    this->declare_parameter<std::string>("ho_wrench_topic", "/falcon_joystick/joystick_wrench");
    this->declare_parameter<std::string>("acs_wrench_pub_topic", "/differential_gt/wrench_from_acs");
    this->declare_parameter<std::string>("ho_wrench_pub_topic", "/differential_gt/wrench_from_ho");
    this->declare_parameter<std::string>("pose_topic", "/admittance_controller/pose_debug");
    this->declare_parameter<std::string>("twist_topic", "/admittance_controller/end_effector_twist");
    this->declare_parameter<std::string>("base_frame", "base_link");
    this->declare_parameter<std::string>("end_effector", "tool0");
    this->declare_parameter<std::string>("twist_from_safety_filter_topic", "/safety_filter/twist");
    this->declare_parameter<std::string>("buttons_topic", "/falcon0/buttons");
    this->declare_parameter<std::string>("acs_reference_point_topic", "/ACS_reference_point");
    this->declare_parameter<std::string>("safety_coefficient_topic", "/safety_coefficient");
    this->declare_parameter<std::string>("override_topic", "joystick/override");
    this->declare_parameter<double>("switch_on_point", 0.71);
    this->declare_parameter<double>("switch_off_point", 0.0);
    this->declare_parameter<double>("publishing_rate", 500.0);
    this->declare_parameter<bool>("override_ho_wrench", false);
    this->declare_parameter<double>("feedback_scaling_factor", 0.5); 

    // Get parameters
    this->ho_wrench_topic_ = this->get_parameter("ho_wrench_topic").as_string();
    this->acs_wrench_pub_topic_ = this->get_parameter("acs_wrench_pub_topic").as_string();
    this->ho_wrench_pub_topic_ = this->get_parameter("ho_wrench_pub_topic").as_string();
    this->pose_topic_ = this->get_parameter("pose_topic").as_string();
    this->twist_topic_ = this->get_parameter("twist_topic").as_string();
    this->base_frame_ = this->get_parameter("base_frame").as_string();
    this->end_effector_ = this->get_parameter("end_effector").as_string();
    this->twist_from_safety_filter_topic_ = this->get_parameter("twist_from_safety_filter_topic").as_string();
    this->buttons_topic_ = this->get_parameter("buttons_topic").as_string();
    this->acs_reference_point_topic_ = this->get_parameter("acs_reference_point_topic").as_string();
    this->safety_coefficient_topic_ = this->get_parameter("safety_coefficient_topic").as_string();
    this->override_topic_ = this->get_parameter("override_topic").as_string();
    this->switch_on_point_ = this->get_parameter("switch_on_point").as_double();
    this->switch_off_point_ = this->get_parameter("switch_off_point").as_double();
    this->publishing_rate_ = this->get_parameter("publishing_rate").as_double();
    this->override_ho_wrench_ = this->get_parameter("override_ho_wrench").as_bool();
    this->feedback_scaling_factor_ = this->get_parameter("feedback_scaling_factor").as_double();

    // Initialize publishers
    this->wrench_from_acs_pub_ = this->create_publisher<geometry_msgs::msg::WrenchStamped>(this->acs_wrench_pub_topic_, 10);
    this->wrench_from_ho_pub_ = this->create_publisher<geometry_msgs::msg::WrenchStamped>(this->ho_wrench_pub_topic_, 10);
    this->feedback_force_pub_ = this->create_publisher<geometry_msgs::msg::WrenchStamped>("/admittance_controller/force_measurements", 10);

    // Initialize subscribers
    this->wrench_from_ho_sub_ = this->create_subscription<geometry_msgs::msg::WrenchStamped>(this->ho_wrench_topic_, 10, std::bind(&DifferentialGT::WrenchFromHOCallback, this, std::placeholders::_1));
    this->pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(this->pose_topic_, 10, std::bind(&DifferentialGT::PoseCallback, this, std::placeholders::_1));
    this->twist_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(this->twist_topic_, 10, std::bind(&DifferentialGT::TwistCallback, this, std::placeholders::_1));
    this->twist_from_safety_filter_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(this->twist_from_safety_filter_topic_, 10, std::bind(&DifferentialGT::TwistFromSafetyFilterCallback, this, std::placeholders::_1));
    this->buttons_sub_ = this->create_subscription<sensor_msgs::msg::Joy>( this->buttons_topic_, 10, std::bind(&DifferentialGT::ButtonsCallback, this, std::placeholders::_1));
    this->acs_reference_point_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(this->acs_reference_point_topic_, 10, std::bind(&DifferentialGT::ACSReferencePointCallback, this, std::placeholders::_1));
    this->safety_coefficient_sub_ = this->create_subscription<std_msgs::msg::Float32>(this->safety_coefficient_topic_, 10, std::bind(&DifferentialGT::SafetyCoefficientCallback, this, std::placeholders::_1));
    this->override_sub_ = this->create_subscription<std_msgs::msg::Int32>(this->override_topic_, 10, std::bind(&DifferentialGT::OverrideCallback, this, std::placeholders::_1));

    // Arbitration initialization
    this->arbitration_ = Arbitration(0.5);

    // Matrices for game theory calculations
    // TODO Parametrize these matrices
    this->SetCostMatrices();

    //Feedback scaling factor
    this->feedback_scaling_factor_ = 1;     // Factor to scale the feedback force sent to the master device
    this->assistance_factor_ = 5.0;         // Factor to increase assistance in non-cooperative GT

    //* Complete game theory initialization
    
    // System Matrices
    // TODO: Compute these from the parameters passed to the admittance controller
    this->SetSystemMatrices();

    // Give system matrices to game theory objects
    this->coop_gt_.setSysParams(this->A_, this->B_);
    this->noncoop_gt_.setSysParams(this->A_, this->B_);

    //* Set initial value of alpha for arbitration
    this->alpha_ = 0.01; 
    this->coop_gt_.setAlpha(this->alpha_);
    
    // Setup game theory objects with cost matrices
    this->coop_gt_.setCostsParams(this->Qhh_, this->Qhr_, this->Qrh_, this->Qrr_, this->Rh_, this->Rr_);

    //! As in Pedrocchi script we set Qh_ and Qr_ for the non-cooperative GT as follows
    this->coop_gt_.getCostMatrices(this->Qh_, this->Qr_, this->Rh_, this->Rr_);

    this->noncoop_gt_.setCostsParams(this->Qh_, this->Qr_*this->assistance_factor_, this->Rh_, this->Rr_);
    
    // Precompute the non-cooperative gains (as long as matrices are constant)
    this->noncoop_gt_.computeNonCooperativeGains();
    this->noncoop_gt_.getNonCooperativeGains(this->K_ncgt_h_, this->K_ncgt_a_);
    
    // Initialize timer
    this->timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000.0 / this->publishing_rate_)),
        std::bind(&DifferentialGT::Publish, this));

    // Startup method to get the correct initial position through tf
    if (!this->Startup())
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to start DifferentialGT node.");
        rclcpp::shutdown();
    }

    // Initialization concluded
    this->is_initialized_ = true;

    //TODO Remove (Debugging)
    this->ref_ho_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/differential_gt/ho_ref", 10);
    this->ref_acs_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/differential_gt/acs_ref", 10);
    this->cos_theta_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/differential_gt/cos_theta", 10);
    this->decision_pub_ = this->create_publisher<std_msgs::msg::Int32>("/differential_gt/decision", 10);
    this->ho_ncgt_wrench_pub_ = this->create_publisher<geometry_msgs::msg::WrenchStamped>("/differential_gt/ho_nc_wrench", 10);
    this->acs_ncgt_wrench_pub_ = this->create_publisher<geometry_msgs::msg::WrenchStamped>("/differential_gt/acs_nc_wrench", 10);
    this->ho_cgt_wrench_pub_ = this->create_publisher<geometry_msgs::msg::WrenchStamped>("/differential_gt/ho_coop_wrench", 10);
    this->acs_cgt_wrench_pub_ = this->create_publisher<geometry_msgs::msg::WrenchStamped>("/differential_gt/acs_coop_wrench", 10);

    this->cos_theta_msg_.data.resize(2);

    this->cosine_similarity_counter_ = 0;

}

//----------------------------------------------------
// Startup
bool DifferentialGT::Startup()
{
    try {
        geometry_msgs::msg::TransformStamped start_transform = this->tf_buffer_.lookupTransform(this->base_frame_, this->end_effector_, 
        tf2::TimePointZero, tf2::durationFromSec(5.0));

        // Set initial position
        this->position_[0] = start_transform.transform.translation.x;
        this->position_[1] = start_transform.transform.translation.y;
        this->position_[2] = start_transform.transform.translation.z;


        // Record the initial position
        this->initial_position_ = this->position_;
        this->acs_ref_.resize(3);
        this->acs_ref_ = this->initial_position_;
        this->ho_ref_.resize(3);
        this->ho_ref_ = this->initial_position_;

        // Set initial orientation
        Eigen::Quaterniond q(
            start_transform.transform.rotation.w,
            start_transform.transform.rotation.x,
            start_transform.transform.rotation.y,
            start_transform.transform.rotation.z);
        this->orientation_ = q.toRotationMatrix();

    } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to get transform: %s", e.what());
        return false;
    }

    return true;
}

//----------------------------------------------------
// ButtonsCallback - Used only with falcon joystick
void DifferentialGT::ButtonsCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
    if (!this->is_initialized_)
        return;

    // Check if the button is pressed
    if (msg->buttons[0] > 0) // Assuming button 0 is the one to toggle
    {
        this->button_pressed_ = true;
    }else{
        this->button_pressed_ = false;
    }
}


//----------------------------------------------------
// WrenchFromHOCallback
void DifferentialGT::WrenchFromHOCallback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
{
    if (!this->is_initialized_)
    {
        return;
    }

    this->wrench_from_ho_msg_.wrench = msg->wrench;
}

//----------------------------------------------------
// PoseCallback
void DifferentialGT::PoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    if (!this->is_initialized_)
    {
        return;
    }

    this->position_[0] = msg->pose.position.x;
    this->position_[1] = msg->pose.position.y;
    this->position_[2] = msg->pose.position.z;
}

//----------------------------------------------------
// TwistCallback
void DifferentialGT::TwistCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
    if (!this->is_initialized_)
    {
        return;
    }

    this->linear_velocity_[0] = msg->twist.linear.x;
    this->linear_velocity_[1] = msg->twist.linear.y;
    this->linear_velocity_[2] = msg->twist.linear.z;
}

//----------------------------------------------------
// TwistFromSafetyFilterCallback
void DifferentialGT::TwistFromSafetyFilterCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
    if (!this->is_initialized_)
    {
        return;
    }

    this->twist_from_safety_filter_[0] = msg->twist.linear.x;
    this->twist_from_safety_filter_[1] = msg->twist.linear.y;
    this->twist_from_safety_filter_[2] = msg->twist.linear.z;
}

//----------------------------------------------------
// ACSReferencePointCallback
void DifferentialGT::ACSReferencePointCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    if (!this->is_initialized_)
    {
        return;
    }

    this->acs_ref_[0] = msg->pose.position.x;
    this->acs_ref_[1] = msg->pose.position.y;
    this->acs_ref_[2] = msg->pose.position.z;
}

//----------------------------------------------------
// SafetyCoefficientCallback
void DifferentialGT::SafetyCoefficientCallback(const std_msgs::msg::Float32::SharedPtr msg)
{
    this->alpha_ = msg->data;
    this->coop_gt_.setAlpha(this->alpha_); // Update the alpha value in the cooperative game theory object
}

//----------------------------------------------------
// OverrideCallback - Used only with wii remote
void DifferentialGT::OverrideCallback(const std_msgs::msg::Int32::SharedPtr msg) 
{
    if (!this->is_initialized_)
    {
        return;
    }
    
    this->override_value_ = msg->data;

    // Set button_pressed_ based on the override value (to emulate falcon joystick)
    if (this->override_value_ == 2)
    {
        this->button_pressed_ = true;
    }
    else
    {
        this->button_pressed_ = false;
    }
}

//----------------------------------------------------
// ComputeFeedbackForce
void DifferentialGT::ComputeFeedbackForce(const Eigen::VectorXd &ho_action, const Eigen::VectorXd &acs_action)
{
    // Compute the magnitudes of the human and ACS wrenches
    double human_magnitude = ho_action.norm();
    double acs_magnitude = acs_action.norm();

    // If wrenches are too small, do not compute feedback force
    if (human_magnitude < 1e-6 || acs_magnitude < 1e-6)
    {
        return;
    }

    // Compute the misalignment and feedback force
    Eigen::Vector3d acs_direction = acs_action / acs_magnitude;
    double projection = ho_action.dot(acs_direction);
    Eigen::Vector3d misalignment = ho_action - projection * acs_direction;
    Eigen::Vector3d feedback_force = -this->feedback_scaling_factor_ * misalignment;

    // Apply a low-pass filter to smooth the feedback force
    static Eigen::Vector3d filtered_feedback_force = Eigen::Vector3d::Zero(); 
    double smoothing_factor = 0.01; // Adjust for more or less smoothing
    filtered_feedback_force = smoothing_factor * feedback_force + (1.0 - smoothing_factor) * filtered_feedback_force;

    // Populate the feedback force message
    this->feedback_force_msg_.header.stamp = this->now();
    this->feedback_force_msg_.header.frame_id = this->base_frame_;
    this->feedback_force_msg_.wrench.force.x = filtered_feedback_force[0];
    this->feedback_force_msg_.wrench.force.y = filtered_feedback_force[1];
    this->feedback_force_msg_.wrench.force.z = filtered_feedback_force[2];
    this->feedback_force_msg_.wrench.torque.x = 0.0;
    this->feedback_force_msg_.wrench.torque.y = 0.0;
    this->feedback_force_msg_.wrench.torque.z = 0.0;

    // Publish the feedback force
    this->feedback_force_pub_->publish(this->feedback_force_msg_);
}

//----------------------------------------------------
// ComputeACSAction
void DifferentialGT::ComputeACSAction()
{
    // Set the reference trajectory for the HO and ACS
    Eigen::VectorXd ref_h;
    Eigen::VectorXd ref_r;
    ref_h.resize(3);
    ref_r.resize(3);

    this->ComputeReferences(ref_h, ref_r);

    //! These lines are for debugging --------------------
    this->ref_ho_msg_.header.stamp = this->now();
    this->ref_ho_msg_.header.frame_id = this->base_frame_;
    this->ref_ho_msg_.pose.position.x = ref_h[0];
    this->ref_ho_msg_.pose.position.y = ref_h[1];
    this->ref_ho_msg_.pose.position.z = ref_h[2];
    this->ref_ho_msg_.pose.orientation.w = 1.0; // Assuming no rotation for the reference

    this->ref_acs_msg_.header.stamp = this->now();
    this->ref_acs_msg_.header.frame_id = this->base_frame_;
    this->ref_acs_msg_.pose.position.x = ref_r[0];
    this->ref_acs_msg_.pose.position.y = ref_r[1];
    this->ref_acs_msg_.pose.position.z = ref_r[2];
    this->ref_acs_msg_.pose.orientation.w = 1.0; // Assuming no rotation for the reference
    //!----------------------------------------------------

    // Compute Cooperative control gains
    this->coop_gt_.computeCooperativeGains(this->alpha_); //! This line here also sets alpha
    this->K_cgt_ = this->coop_gt_.getCooperativeGains();

    this->coop_gt_.setPosReference(ref_h, ref_r); //TODO Change with a complete reference
    this->noncoop_gt_.setPosReference(ref_h, ref_r); //TODO Change with a complete reference

    // Get the references for the cooperative and non-cooperative game
    Eigen::VectorXd ref_cgt = this->coop_gt_.getReference();
    Eigen::VectorXd ref_ncgt_h, ref_ncgt_a;
    this->noncoop_gt_.getReference(ref_ncgt_h, ref_ncgt_a);

    // Compute the control actions
    Eigen::VectorXd current_state(6);
    current_state << this->position_, this->linear_velocity_;
    Eigen::VectorXd u_cgt = -this->K_cgt_ * (current_state - ref_cgt);

    Eigen::VectorXd u_cgt_h = u_cgt.segment(0, 3);
    Eigen::VectorXd u_cgt_a = u_cgt.segment(3, 3);

    Eigen::VectorXd u_ncgt_h = -this->K_ncgt_h_ * (current_state - ref_ncgt_h);
    Eigen::VectorXd u_ncgt_a = -this->K_ncgt_a_ * (current_state - ref_ncgt_a);

    // Get the real wrench applied by the HO
    Eigen::VectorXd uh_real(3);
    uh_real << this->wrench_from_ho_msg_.wrench.force.x,
                this->wrench_from_ho_msg_.wrench.force.y,
                this->wrench_from_ho_msg_.wrench.force.z;

    //! These lines are for debugging --------------------
    this->cos_theta_msg_.data.clear();
    this->cos_theta_msg_.data.push_back(this->cos_theta_);
    this->cos_theta_msg_.data.push_back(this->cos_theta_coop_);
    this->cos_theta_msg_.data.push_back(this->cos_theta_nc_);
    this->cos_theta_msg_.data.push_back(this->alpha_);
    this->decision_msg_.data = this->decision_;

    this->acs_ncgt_wrench_msg_.header.stamp = this->now();
    this->acs_ncgt_wrench_msg_.header.frame_id = this->base_frame_;
    this->acs_ncgt_wrench_msg_.wrench.force.x = u_ncgt_a[0];
    this->acs_ncgt_wrench_msg_.wrench.force.y = u_ncgt_a[1];
    this->acs_ncgt_wrench_msg_.wrench.force.z = u_ncgt_a[2];

    this->ho_ncgt_wrench_msg_.header.stamp = this->now();
    this->ho_ncgt_wrench_msg_.header.frame_id = this->base_frame_;
    this->ho_ncgt_wrench_msg_.wrench.force.x = u_ncgt_h[0];
    this->ho_ncgt_wrench_msg_.wrench.force.y = u_ncgt_h[1];
    this->ho_ncgt_wrench_msg_.wrench.force.z = u_ncgt_h[2];

    this->ho_cgt_wrench_msg_.header.stamp = this->now();
    this->ho_cgt_wrench_msg_.header.frame_id = this->base_frame_;
    this->ho_cgt_wrench_msg_.wrench.force.x = u_cgt_h[0];
    this->ho_cgt_wrench_msg_.wrench.force.y = u_cgt_h[1];
    this->ho_cgt_wrench_msg_.wrench.force.z = u_cgt_h[2];

    this->acs_cgt_wrench_msg_.header.stamp = this->now();
    this->acs_cgt_wrench_msg_.header.frame_id = this->base_frame_;
    this->acs_cgt_wrench_msg_.wrench.force.x = u_cgt_a[0];
    this->acs_cgt_wrench_msg_.wrench.force.y = u_cgt_a[1];
    this->acs_cgt_wrench_msg_.wrench.force.z = u_cgt_a[2];
    //!---------------------------------------------------

    Eigen::VectorXd acs_action(3); // Action to be published
    Eigen::VectorXd ho_action(3); // Action from the HO
    Eigen::VectorXd feedback_force(3); // Feedback force to be sent to the master device

    // Check the manual override value
    if (this->override_value_ == 1) 
    {
        acs_action.setZero(); // Set acs_action to 0 and skip the following block
    } 
    else 
    {
        // ARBITRATION ------------------------------------------

        this->arbitration_.CosineSimilarityHysteresis(
            uh_real, u_ncgt_a, this->cos_theta_, this->decision_,
            this->switch_on_point_, this->switch_off_point_
        );

        // Add a blending factor for smooth transitions between modes
        double blending_factor_ = 0.0; // Starts at 0 (NC) and transitions to 1 (C)
        double blending_rate_ = 0.1;   // Rate of blending

        if (this->decision_ == 0) // Cooperative mode
        {
            // Gradually increase the blending factor
            blending_factor_ = std::min(1.0, blending_factor_ + blending_rate_);

            // Blend the ACS force between NC and C
            acs_action = blending_factor_ * (1 - this->alpha_) * u_cgt_a + (1 - blending_factor_) * ((1 - this->alpha_) * u_ncgt_a);

            ho_action = u_cgt_h;

            if (this->override_ho_wrench_)
            {
                acs_action += u_cgt_h;
            }

            // Compute and publish feedback force
            // this->ComputeFeedbackForce(ho_action, u_ncgt_a); // using u_ncgt_a as the ACS action for feedback
        }
        else // Non-cooperative mode
        {
            // Gradually decrease the blending factor
            blending_factor_ = std::max(0.0, blending_factor_ - blending_rate_);

            // Blend the ACS force between NC and C
            acs_action = blending_factor_ * (1 - this->alpha_) * u_cgt_a + (1 - blending_factor_) * ((1 - this->alpha_) * u_ncgt_a);

            ho_action = u_ncgt_h;

            if (this->override_ho_wrench_)
            {
                acs_action += u_ncgt_h;
            }
        }
    }

    // Create the WrenchStamped message to publish
    this->wrench_from_acs_msg_.header.stamp = this->now();
    this->wrench_from_acs_msg_.header.frame_id = this->base_frame_;
    this->wrench_from_acs_msg_.wrench.force.x = acs_action[0];
    this->wrench_from_acs_msg_.wrench.force.y = acs_action[1];
    this->wrench_from_acs_msg_.wrench.force.z = acs_action[2];
    this->wrench_from_acs_msg_.wrench.torque.x = 0.0;
    this->wrench_from_acs_msg_.wrench.torque.y = 0.0;
    this->wrench_from_acs_msg_.wrench.torque.z = 0.0;

    this->wrench_ho_topub_msg_.header.stamp = this->now();
    this->wrench_ho_topub_msg_.header.frame_id = this->base_frame_;
    this->wrench_ho_topub_msg_.wrench.force.x = this->wrench_from_ho_msg_.wrench.force.x;
    this->wrench_ho_topub_msg_.wrench.force.y = this->wrench_from_ho_msg_.wrench.force.y;
    this->wrench_ho_topub_msg_.wrench.force.z = this->wrench_from_ho_msg_.wrench.force.z;
    this->wrench_ho_topub_msg_.wrench.torque.x = this->wrench_from_ho_msg_.wrench.torque.x;
    this->wrench_ho_topub_msg_.wrench.torque.y = this->wrench_from_ho_msg_.wrench.torque.y;
    this->wrench_ho_topub_msg_.wrench.torque.z = this->wrench_from_ho_msg_.wrench.torque.z;
}

//----------------------------------------------------
// Set System Matrices
void DifferentialGT::SetSystemMatrices()
{
    //! Hardcoded value for n_dofs
    int n_dofs = 3; // Number of degrees of freedom, can be set as a parameter
    this->A_.resize(2 * n_dofs,2 * n_dofs);
    this->B_.resize(2 * n_dofs, n_dofs);

    //! Hardcoded values for system matrices
    double m,c,k;
    m=10;
    k=0;
    c=100;
    
    Eigen::MatrixXd M = m * Eigen::MatrixXd::Identity(n_dofs,n_dofs);
    Eigen::MatrixXd C = c * Eigen::MatrixXd::Identity(n_dofs,n_dofs);
    Eigen::MatrixXd K = k * Eigen::MatrixXd::Identity(n_dofs,n_dofs);
    this->A_.setZero();
    this->B_.setZero();
    this->A_.block(0, n_dofs, n_dofs, n_dofs) = Eigen::MatrixXd::Identity(n_dofs,n_dofs);
    this->A_.block(n_dofs, 0, n_dofs, n_dofs) = -M.inverse() * K;
    this->A_.block(n_dofs, n_dofs, n_dofs, n_dofs) = -M.inverse() * C;
    this->B_.block(n_dofs, 0, n_dofs, n_dofs) = M.inverse();

    double Ts         = 1.0 / this->publishing_rate_;                     // Sampling Time
    double eig        = -c / m;                                           // Eigenvalue for the system (Continuous Time)
    double diag_a     = std::exp(eig * Ts);                               // Discrete Time Eigenvalue
    double diag_b     = 1.0 / eig * (diag_a - 1.0) / m;                   // Discrete Time Input Gain
    this->F_     = Eigen::Matrix3d::Identity() * diag_a;
    this->G_     = Eigen::Matrix3d::Identity() * diag_b;
}

//----------------------------------------------------
// Set Cost Matrices
void DifferentialGT::SetCostMatrices()
{

    // TODO Write better this method. Provide clear division between cooperative and non-cooperative GT cost matrices.
    this->Qhh_.resize(6, 6);
    this->Qhr_.resize(6, 6);
    this->Qrr_.resize(6, 6);
    this->Qrh_.resize(6, 6);
    this->Rh_.resize(3, 3);
    this->Rr_.resize(3, 3);
    this->Rrr_.resize(3, 3);
    this->Rhh_.resize(3, 3);
    this->Rhr_.resize(3, 3);
    this->Rrh_.resize(3, 3);
    this->Qh_.resize(6, 6);
    this->Qr_.resize(6, 6);

    this->Qhh_.setIdentity();
    this->Qhr_.setIdentity();
    this->Qrr_.setIdentity();
    this->Qrh_.setIdentity();
    this->Rh_.setIdentity();
    this->Rr_.setIdentity();
    this->Rrr_.setIdentity();
    this->Rhh_.setIdentity();
    


    this->Qhh_.block(3, 3, 3, 3) = 1e-4 * Eigen::Matrix3d::Identity();

    this->Qrh_.block(0, 0, 3, 3) = Eigen::Matrix3d::Zero();
    this->Qrh_.block(3, 3, 3, 3) = 1e-4 * Eigen::Matrix3d::Identity();

    this->Qrr_.block(3, 3, 3, 3) = 1e-4 * Eigen::Matrix3d::Identity();

    this->Qhr_.block(0, 0, 3, 3) = 1e-4 * Eigen::Matrix3d::Identity();
    this->Qhr_.block(3, 3, 3, 3) = Eigen::Matrix3d::Zero();

    this->Rh_ = 5e-4 * Eigen::Matrix3d::Identity();
    this->Rr_ = 1e-4 * Eigen::Matrix3d::Identity();

    this->Rrr_ = this->Rr_; //! Attention here
    this->Rhh_ = this->Rh_; //! Attention here
}

//----------------------------------------------------
// Publish
void DifferentialGT::Publish()
{
    if (!this->is_initialized_)
    {
        return;
    }

    // Compute the ACS action
    this->ComputeACSAction();
    this->wrench_from_acs_pub_->publish(this->wrench_from_acs_msg_);
    this->wrench_from_ho_pub_->publish(this->wrench_ho_topub_msg_);

    //? Lines are for debugging------------------------------------
    this->ref_ho_pub_         ->publish(this->ref_ho_msg_);
    this->ref_acs_pub_        ->publish(this->ref_acs_msg_);
    this->cos_theta_pub_      ->publish(this->cos_theta_msg_);
    this->decision_pub_       ->publish(this->decision_msg_);
    this->acs_cgt_wrench_pub_ ->publish(this->acs_cgt_wrench_msg_);
    this->ho_cgt_wrench_pub_  ->publish(this->ho_cgt_wrench_msg_);
    this->acs_ncgt_wrench_pub_->publish(this->acs_ncgt_wrench_msg_);
    this->ho_ncgt_wrench_pub_ ->publish(this->ho_ncgt_wrench_msg_);
    //?--------------------------------------------------------------
}

               
void DifferentialGT::ComputeReferences(Eigen::VectorXd &ref_h, Eigen::VectorXd &ref_r)
{
    // Compute the reference for the HO and ACS
    Eigen::VectorXd current_state = Eigen::VectorXd::Zero(6);
    Eigen::VectorXd ref_ho = Eigen::VectorXd::Zero(6);
    Eigen::VectorXd ref_acs = Eigen::VectorXd::Zero(6);
    ref_h.resize(3);
    ref_r.resize(3);

    current_state.segment(0,3) = this->position_;
    ref_ho.segment(0, 3) = this->ho_ref_;
    ref_acs.segment(0, 3) = this->acs_ref_;
 
    if (!this->button_pressed_)
    {
        this->decision_ = 0;
        this->coop_gt_.setAlpha(this->alpha_);
        this->coop_gt_.setPosReference(this->ho_ref_, this->acs_ref_);

        Eigen::VectorXd ref_cgt = this->coop_gt_.getReference();
        this->ho_ref_ = this->position_; // When button is not pressed HO & ACS reference is the current position
        this->acs_ref_ = this->ho_ref_;

        ref_h << this->ho_ref_[0],
                 this->ho_ref_[1],
                 this->ho_ref_[2];

        ref_r << this->acs_ref_[0],
                 this->acs_ref_[1],
                 this->acs_ref_[2];

        
    } else {

        // Compute the reference for the HO based on the admittance model
        double dt = 1.0 / this->publishing_rate_;
        double gamma = 1e-2;
        Eigen::Vector3d uh(
            this->wrench_from_ho_msg_.wrench.force.x,
            this->wrench_from_ho_msg_.wrench.force.y,
            this->wrench_from_ho_msg_.wrench.force.z);

        this->z_ = this->F_ * this->z_ + this->G_ * uh; // Update the state z
        
        this->ho_ref_ = this->ho_ref_ + dt * this->z_;  // Update the reference for the HO
        
        ref_r << this->acs_ref_[0],
                 this->acs_ref_[1],
                 this->acs_ref_[2];

        
        ref_h << this->ho_ref_[0],
                 this->ho_ref_[1],
                 this->ho_ref_[2];

        this->prev_decision_ = this->decision_;
    }
}