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
    this->declare_parameter<double>("switch_on_point", 0.71);
    this->declare_parameter<double>("switch_off_point", 0.0);
    this->declare_parameter<double>("publishing_rate", 500.0);
    this->declare_parameter<bool>("override_ho_wrench", false); 
    this->declare_parameter<std::string>("save_matrices", ""); 
    this->declare_parameter<std::string>("load_matrices", "");

    // Get parameters
    this->ho_wrench_topic_ = this->get_parameter("ho_wrench_topic").as_string();
    this->acs_wrench_pub_topic_ = this->get_parameter("acs_wrench_pub_topic").as_string();
    this->ho_wrench_pub_topic_ = this->get_parameter("ho_wrench_pub_topic").as_string();
    this->pose_topic_ = this->get_parameter("pose_topic").as_string();
    this->twist_topic_ = this->get_parameter("twist_topic").as_string();
    this->base_frame_ = this->get_parameter("base_frame").as_string();
    this->end_effector_ = this->get_parameter("end_effector").as_string();
    this->switch_on_point_ = this->get_parameter("switch_on_point").as_double();
    this->switch_off_point_ = this->get_parameter("switch_off_point").as_double();
    this->publishing_rate_ = this->get_parameter("publishing_rate").as_double();
    this->override_ho_wrench_ = this->get_parameter("override_ho_wrench").as_bool();
    this->save_matrices_ = this->get_parameter("save_matrices").as_string();
    this->load_matrices_ = this->get_parameter("load_matrices").as_string();

    // Initialize publishers and subscribers
    this->wrench_from_acs_pub_ = this->create_publisher<geometry_msgs::msg::WrenchStamped>(this->acs_wrench_pub_topic_, 10);
    this->wrench_from_ho_pub_ = this->create_publisher<geometry_msgs::msg::WrenchStamped>(this->ho_wrench_pub_topic_, 10);
    this->feedback_wrench_pub_ = this->create_publisher<geometry_msgs::msg::WrenchStamped>("/differential_gt/force_feedback", 10);

    this->wrench_from_ho_sub_ = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
        this->ho_wrench_topic_, 10, std::bind(&DifferentialGT::WrenchFromHOCallback, this, std::placeholders::_1));

    this->pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        this->pose_topic_, 10, std::bind(&DifferentialGT::PoseCallback, this, std::placeholders::_1));
    
    this->twist_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
        this->twist_topic_, 10, std::bind(&DifferentialGT::TwistCallback, this, std::placeholders::_1));

    this->twist_from_safety_filter_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
        "/safety_filter/twist", 10, std::bind(&DifferentialGT::TwistFromSafetyFilterCallback, this, std::placeholders::_1));

    this->buttons_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "/falcon0/buttons", 10, std::bind(&DifferentialGT::ButtonsCallback, this, std::placeholders::_1));
     

    // Arbitration
    this->arbitration_ = Arbitration(0.5);


    //* Complete game theory initialization

    
    // System Matrices
    // TODO: Compute these from the parameters passed to the admittance controller
    this->SetSystemMatrices();

    // Give system matrices to game theory objects
    this->coop_gt_.setSysParams(this->A_, this->B_);
    this->noncoop_gt_.setSysParams(this->A_, this->B_);

    //* Set initial value of alpha for arbitration
    this->alpha_ = 0.9; // Default value, can be changed later
    this->coop_gt_.setAlpha(this->alpha_);

    // Matrices for game theory calculations
    // TODO Parametrize these matrices
    this->SetCostMatrices();
    

    if (this->save_matrices_ != "" && this->load_matrices_ == "")
    {
        // Save matrices in a file
        std::vector<Eigen::MatrixXd> matrices = {this->Qhh_, this->Qhr_, this->Qrh_, this->Qrr_,
                                            this->Rh_, this->Rr_, this->Qh_, this->Qr_, this->Rhh_, this->Rhr_, this->Rrh_, this->Rrr_};

        std::string save_path = "/home/alebarte/ur5e_ws/src/differential_gt/saved_configs/" + this->save_matrices_ + ".txt";
        Utils::saveMultipleMatrices(save_path, matrices);
    }

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
    this->tank_level_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/differential_gt/tank_level", 10);

    this->cos_theta_msg_.data.resize(2);

    this->cosine_similarity_counter_ = 0;
    this->prev_decision_ = 0;

}

//----------------------------------------------------
// Startup
bool DifferentialGT::Startup()
{
    try {
        geometry_msgs::msg::TransformStamped start_transform = this->tf_buffer_.lookupTransform(this->base_frame_, this->end_effector_, 
        tf2::TimePointZero, tf2::durationFromSec(5.0));
        this->position_.resize(3);
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
// ButtonsCallback
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
    if (this->decision_ == 0) // Use cooperative action
    {
        acs_action = u_cgt_a;
        ho_action = u_cgt_h; // Action from the HO in cooperative game
        if (this->override_ho_wrench_)
        {
            acs_action += u_cgt_h; // Add the cooperative action for the first agent
        }
    }
    else // Use non-cooperative action
    {
        acs_action = u_ncgt_a;
        ho_action = u_ncgt_h; // Action from the HO in non-cooperative game
        if (this->override_ho_wrench_)
        {
            acs_action += u_ncgt_h; // Add the non-cooperative action for the first agent
        }
    }

    //TODO: Matrix tuning
    // Eigen::MatrixXd damping = Eigen::MatrixXd::Identity(3, 3) * 50.0; // Damping matrix  //TODO Find matrix with equivalent effect for NCGT
    // Eigen::VectorXd damping_action = - damping * this->linear_velocity_; // Damping action

    // if (this->decision_ == 0)
    // {
    //     damping_action = Eigen::VectorXd::Zero(3); // No damping action in cooperative game
    //     // this->stored_energy_ = 0.7 * this->stored_energy_; // Reduce the stored energy in cooperative game
    // }

    double ct = std::max(0.0, std::min(1.0, this->cos_theta_));
    double neg_cosine = std::min(0.0, this->cos_theta_);
    this->stored_energy_ = (1.0 - ct) * this->stored_energy_ - neg_cosine; // Energy dissipated by the damping action
    this->stored_energy_ = std::max(0.0, std::min(10.0, this->stored_energy_)); // Ensure stored energy is non-negative


    // ARBITRATION -------------------
    // Select Game
    // Select alpha
    
    //________________________________

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

    if (!this->potential_active_)
    {
        this->feedback_wrench_msg_.wrench.force.x = acs_action[0];
        this->feedback_wrench_msg_.wrench.force.y = acs_action[1];
        this->feedback_wrench_msg_.wrench.force.z = acs_action[2];
        this->feedback_wrench_msg_.wrench.torque.x = 0.0;
        this->feedback_wrench_msg_.wrench.torque.y = 0.0;
        this->feedback_wrench_msg_.wrench.torque.z = 0.0;
    } else {
        this->feedback_wrench_msg_.wrench.force.x = 0.0;
        this->feedback_wrench_msg_.wrench.force.y = 0.0;
        this->feedback_wrench_msg_.wrench.force.z = 0.0;
        this->feedback_wrench_msg_.wrench.torque.x = 0.0;
        this->feedback_wrench_msg_.wrench.torque.y = 0.0;
        this->feedback_wrench_msg_.wrench.torque.z = 0.0;
    }


    //? Debugging --------------------------

    this->tank_level_msg_.data.clear();
    this->tank_level_msg_.data.push_back(this->stored_energy_);

    //? ------------------------------------

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
    this->Rhr_.setIdentity();
    this->Rrh_.setIdentity();
    

    if (this->load_matrices_ == "")
    {
        this->Qhh_.block(3, 3, 3, 3) = 1e-4 * Eigen::Matrix3d::Identity();

        this->Qrh_.block(0, 0, 3, 3) = Eigen::Matrix3d::Zero();
        this->Qrh_.block(3, 3, 3, 3) = 1e-4 * Eigen::Matrix3d::Identity();

        this->Qrr_.block(3, 3, 3, 3) = 1e-4 * Eigen::Matrix3d::Identity();

        this->Qhr_.block(0, 0, 3, 3) = 1e-1 * Eigen::Matrix3d::Identity();
        this->Qhr_.block(3, 3, 3, 3) = Eigen::Matrix3d::Zero();

        this->Rh_ = 5e-4 * Eigen::Matrix3d::Identity();
        this->Rr_ = 1e-4 * Eigen::Matrix3d::Identity();

        this->Rrr_ = this->Rr_; //! Attention here
        this->Rhh_ = this->Rh_; //! Attention here
        this->Rhr_ = Eigen::Matrix3d::Zero();
        this->Rrh_ = Eigen::Matrix3d::Zero();


        // Setup game theory objects with cost matrices
        this->coop_gt_.setCostsParams(this->Qhh_, this->Qhr_, this->Qrh_, this->Qrr_, this->Rh_, this->Rr_);
        std::cout << "Setting up Cooperative Game Theory with matrices:" << std::endl;

        //! As in Pedrocchi script we set Qh_ and Qr_ for the non-cooperative GT as follows
        this->coop_gt_.getCostMatrices(this->Qh_, this->Qr_, this->Rh_, this->Rr_);
        std::cout << "Here"<<std::endl;
        // this->Qr_ = 30.0 * this->Qr_; // Scale the Qr matrix for non-cooperative game theory
        this->noncoop_gt_.setCostsParams(this->Qh_, this->Qr_, this->Rhh_, this->Rrr_, this->Rhr_, this->Rrh_);
        std::cout << "Setting up Non-Cooperative Game Theory with matrices:" << std::endl;
        //!--------------------------------------------------------------------------------
    } else {
        // Load matrices from a file
        std::string load_path = "/home/alebarte/ur5e_ws/src/differential_gt/saved_configs/" + this->load_matrices_ + ".txt";
        std::vector<Eigen::MatrixXd> matrices;
        matrices = Utils::loadMultipleMatrices(load_path);
        
        this->Qhh_ = matrices[0];
        this->Qhr_ = matrices[1];
        this->Qrh_ = matrices[2];
        this->Qrr_ = matrices[3];
        this->Rh_ = matrices[4];
        this->Rr_ = matrices[5];
        this->Qh_ = matrices[6];
        this->Qr_ = matrices[7];
        this->Rhh_ = matrices[8];
        this->Rhr_ = matrices[9];
        this->Rrh_ = matrices[10];
        this->Rrr_ = matrices[11];

        std::cout << "Loading Cooperative Game Theory matrices from file: " << load_path << std::endl;
        std::cout << "Qhh_:\n" << this->Qhh_ << std::endl;
        std::cout << "Qhr_:\n" << this->Qhr_ << std::endl;
        std::cout << "Qrh_:\n" << this->Qrh_ << std::endl;
        std::cout << "Qrr_:\n" << this->Qrr_ << std::endl;
        std::cout << "Rh_:\n" << this->Rh_ << std::endl;
        std::cout << "Rr_:\n" << this->Rr_ << std::endl;
        std::cout << "Qh_:\n" << this->Qh_ << std::endl;
        std::cout << "Qr_:\n" << this->Qr_ << std::endl;
        std::cout << "Rhh_:\n" << this->Rhh_ << std::endl;
        std::cout << "Rhr_:\n" << this->Rhr_ << std::endl;
        std::cout << "Rrh_:\n" << this->Rrh_ << std::endl;
        std::cout << "Rrr_:\n" << this->Rrr_ << std::endl;

        this->coop_gt_.setCostsParams(this->Qhh_, this->Qhr_, this->Qrh_, this->Qrr_, this->Rh_, this->Rr_);

        
        this->noncoop_gt_.setCostsParams(this->Qh_, this->Qr_, this->Rhh_, this->Rrr_, this->Rhr_, this->Rrh_);
    }
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
    this->wrench_from_ho_pub_ ->publish(this->wrench_ho_topub_msg_);
    this->feedback_wrench_pub_->publish(this->feedback_wrench_msg_);

    //? Lines are for debugging------------------------------------
    this->ref_ho_pub_         ->publish(this->ref_ho_msg_);
    this->ref_acs_pub_        ->publish(this->ref_acs_msg_);
    this->cos_theta_pub_      ->publish(this->cos_theta_msg_);
    this->decision_pub_       ->publish(this->decision_msg_);
    this->acs_cgt_wrench_pub_ ->publish(this->acs_cgt_wrench_msg_);
    this->ho_cgt_wrench_pub_  ->publish(this->ho_cgt_wrench_msg_);
    this->acs_ncgt_wrench_pub_->publish(this->acs_ncgt_wrench_msg_);
    this->ho_ncgt_wrench_pub_ ->publish(this->ho_ncgt_wrench_msg_);
    this->tank_level_pub_     ->publish(this->tank_level_msg_);
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
    ref_ho.segment(0, 3)       = this->ho_ref_;
    ref_acs.segment(0, 3)      = this->acs_ref_;

    // Introduce a Goal and an Obstacle
    //! Warning: Hardcoded values -----------------------------------------------------------------
    Eigen::VectorXd goal(3);
    Eigen::VectorXd goal1(3);
    Eigen::VectorXd goal2(3);
    Eigen::VectorXd obstacle(3);
    Eigen::VectorXd obstacle2(3);

    std::vector<Eigen::VectorXd> obstacle_vec;
    std::vector<double> obstacle_radius_vec;
    std::vector<Eigen::VectorXd> goal_vec;
    goal1 << 0.40, 0.49, 0.3;
    goal2 << 0.11, 0.64, 0.1; // This is the second goal position, can be set as a parameter

    obstacle << 0.11, 0.49, 0.1; // This is the obstacle position, can be set as a parameter
    obstacle2 << 0.11, 0.79, 0.1; // This is the second obstacle position, can be set as a parameter

    obstacle_vec = {obstacle, obstacle2}; // Vector of obstacles, can be extended with more obstacles
    goal_vec = {goal1, goal2};

    double obstacle_radius = 0.2; // Radius of the obstacle, can be set as a parameter
    double obstacle_radius2 = 0.2; // Radius of the second obstacle, can be set as a parameter
    obstacle_radius_vec = {obstacle_radius, obstacle_radius2};
    //!--------------------------------------------------------------------------------------------

    //! Added line for faking the button press
    // this->button_pressed_ = true; //! Remove absolutely--------------------------------------

    if (!this->button_pressed_)
    {
        this->decision_ = 0;
        // this->alpha_ = this->coop_gt_.getAlphaFromCurrentState(current_state, ref_ho, ref_acs);
        this->alpha_ = 0.9; 
        this->coop_gt_.setAlpha(this->alpha_);
        this->coop_gt_.setPosReference(this->ho_ref_, this->acs_ref_);

        Eigen::VectorXd ref_cgt = this->coop_gt_.getReference();
        this->ho_ref_ = this->position_;
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
        double gamma = 0.01;
        Eigen::Vector3d uh(
            this->wrench_from_ho_msg_.wrench.force.x,
            this->wrench_from_ho_msg_.wrench.force.y,
            this->wrench_from_ho_msg_.wrench.force.z);

        this->z_ = this->F_ * this->z_ + this->G_ * uh; // Update the state z

        // Compute the reference for the ACS based on the goal position
        Eigen::MatrixXd K = Eigen::MatrixXd::Identity(3, 3) * gamma;
        Eigen::MatrixXd K_obstacle = Eigen::MatrixXd::Identity(3, 3) * 0.01; // Repulsive force gain
        Eigen::VectorXd repulsive_force = this->ComputeRepulsiveForce(obstacle_vec, obstacle_radius_vec);
        // Select most likely goal
        goal = this->SelectGoal(goal_vec, uh);
        Eigen::VectorXd goal_diff = goal - this->position_;
        Eigen::VectorXd attractive_force = K * goal_diff; 
        this->acs_ref_ = this->position_ +  attractive_force + K_obstacle * repulsive_force; // Update the ACS reference (Here I will need to sum the effect of the backoff caused by the potential)
        
        if (goal_diff.norm() < 0.05) // If close to the goal, stop
        {
            this->acs_ref_ = goal;
        }
        this->ho_ref_ = this->position_ + dt * this->z_; //!Modified Here

        // Compute the energy dissipated
        double delta_energy = repulsive_force.dot(this->linear_velocity_) * dt;
        double Ug = 0.5 * goal_diff.dot(attractive_force);

        if (delta_energy > 1e-6)
            this->stored_energy_ = this->stored_energy_ + delta_energy;

        // Arbitrate (Could write better)
        Eigen::VectorXd u_h(3);
        u_h << this->wrench_from_ho_msg_.wrench.force.x,
                this->wrench_from_ho_msg_.wrench.force.y,
                this->wrench_from_ho_msg_.wrench.force.z;
        
        int placeholder;
        Eigen::VectorXd delta = this->acs_ref_ - this->position_;
        this->arbitration_.CosineSimilarity(u_h, delta, this->cos_theta_, placeholder);
        this->alpha_ = std::max(0.01, std::min(0.99, this->cos_theta_));

        Eigen::VectorXd goal_dir = goal - this->position_;
        double goal_theta;
        this->arbitration_.CosineSimilarity(u_h, goal_dir, goal_theta, placeholder);
        
        if (goal_theta < 0.0) {
            this->stored_energy_ = this->stored_energy_ + 1e-5 * Ug; // Ensure stored energy is non-negative
        }

        if (this->stored_energy_ > 0.05)
        {
            this->decision_ = 1;
        }else{
            this->decision_ = 0;
        }

        ref_r << this->acs_ref_[0],
                 this->acs_ref_[1],
                 this->acs_ref_[2];

        
        ref_h << this->ho_ref_[0],
                 this->ho_ref_[1],
                 this->ho_ref_[2];
    }
}

Eigen::VectorXd DifferentialGT::ComputeRepulsiveForce(const Eigen::VectorXd &obstacle, const double &radius)
{
    Eigen::VectorXd force(3);
    force.setZero();

    double eta = 1.0;

    // Compute the distance to the obstacle
    Eigen::VectorXd diff = this->position_ - obstacle;
    double distance = diff.norm();

    // If within the radius, compute the repulsive force
    if (distance < radius)
    {
        force =  eta * (1.0 / distance - 1.0 / radius) * diff / std::pow(distance, 3);
    }

    return force;
}

Eigen::VectorXd DifferentialGT::ComputeRepulsiveForce(const std::vector<Eigen::VectorXd> &obstacles, const std::vector<double> &radii)
{
    Eigen::VectorXd total_force(3);
    Eigen::MatrixXd D = 5.0 * Eigen::MatrixXd::Identity(3,3);
    total_force.setZero();

    for (size_t i = 0; i < obstacles.size(); ++i)
    {
        Eigen::VectorXd force = this->ComputeRepulsiveForce(obstacles[i], radii[i]);
        total_force += force;
    }

    if (total_force.norm() > 0.0)
    {
        this->potential_active_ = true; // Set the potential active flag if any force is computed
    } else {
        this->potential_active_ = false; // Reset the potential active flag if no force is computed
    }
    total_force = total_force - D* this->linear_velocity_; // Damping effect on the repulsive force
    return total_force;
}

//----------------------------------------------------
// Select Goal

Eigen::VectorXd DifferentialGT::SelectGoal(const std::vector<Eigen::VectorXd> &goals, const Eigen::VectorXd& uh)
{
    double sum = 0.0;
    Eigen::VectorXd w(goals.size());
    Eigen::VectorXd goal_dist(3);

    if (uh.norm() < 1e-6)
    {
        return this->position_;
    }

    for (int i = 0; i < goals.size(); i++)
    {
        goal_dist = goals[i] - this->position_;
        sum += std::exp(-goal_dist.norm());
        w[i] = std::exp(-goal_dist.norm());
    }
    w /= sum;

    int max_index;
    w.maxCoeff(&max_index); // Get the index of the maximum value in w

    return goals[max_index]; // Return the goal with the maximum weight
}