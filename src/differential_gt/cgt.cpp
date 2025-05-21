#include <differential_gt/cgt.h>

//*------------------------------------------------------------*//
//*----------------------- Constructor ------------------------*//
//*------------------------------------------------------------*//
CoopGT::CoopGT(const int& n_dofs, const double& dt): n_dofs_(n_dofs), dt_(dt)
{
  // System Matrices (State Space representation)
  this->A_.resize(2*n_dofs_,2*n_dofs_);
  this->B_.resize(2*n_dofs_,n_dofs_);
  this->C_.resize(n_dofs_,2*n_dofs_);
  this->X_.resize(2*n_dofs_);
  this->dX_.resize(2*n_dofs_);
  
  this->Q11_.resize(2*n_dofs_,2*n_dofs_); this->Q11_.setZero();
  this->Q12_.resize(2*n_dofs_,2*n_dofs_); this->Q12_.setZero();
  this->Q21_.resize(2*n_dofs_,2*n_dofs_); this->Q21_.setZero();
  this->Q22_.resize(2*n_dofs_,2*n_dofs_); this->Q22_.setZero();
  
  // State weight matrices for cooperative GT
  this->Q1_  .resize(2*n_dofs_,2*n_dofs_);this->Q1_  .setZero(); 
  this->Q2_  .resize(2*n_dofs_,2*n_dofs_);this->Q2_  .setZero(); 
  this->Q_gt_.resize(2*n_dofs_,2*n_dofs_);this->Q_gt_.setZero(); 
  
  // Control weight matrices for cooperative GT
  this->R1_.resize(n_dofs_,n_dofs_); this->R1_.setZero();
  this->R2_.resize(n_dofs_,n_dofs_); this->R2_.setZero();
  this->R_gt_.resize(2*n_dofs_,2*n_dofs_); this->R_gt_.setZero(); 
  
  this->reference_.resize(n_dofs_); this->reference_.setZero();
  
  // Cooperative GT gain
  K_cgt_.resize(2*n_dofs_,2*n_dofs_); K_cgt_.setZero();
  
  // Flags
  this->state_ok_       = false;
  this->reference_ok_   = false;
  this->sys_params_set_ = false;
  this->cost_params_set_= false;
  this->gains_set_      = false;
  this->alpha_set_      = false;
}

//*-----------------------------------------------------------*//
//*---------------------- Public Methods ---------------------*//
//*-----------------------------------------------------------*//

//-----------------------------------------------------------
// Set System Parameters (C not specified)
void CoopGT::setSysParams( const Eigen::MatrixXd& A,
                        const Eigen::MatrixXd& B)
{
  Eigen::MatrixXd C; 
  C.resize(this->n_dofs_, 2 * this->n_dofs_); 
  C.setZero();
  C.block(0, 0, this->n_dofs_, this->n_dofs_) = Eigen::MatrixXd::Identity(this->n_dofs_, this->n_dofs_);
  setSysParams(A,B,C);
}

//-----------------------------------------------------------
// Set System Parameters (C specified)
void CoopGT::setSysParams( const Eigen::MatrixXd& A,
                        const Eigen::MatrixXd& B,
                        const Eigen::MatrixXd& C)
{
  this->A_ = A;
  this->B_ = B;
  this->C_ = C;
  
  std::cout << "A:\n" << this->A_ << std::endl;
  std::cout << "B:\n" << this->B_ << std::endl;
  std::cout << "C:\n" << this->C_ << std::endl;


  // ROS_DEBUG_STREAM("A:\n"<<A_);
  // ROS_DEBUG_STREAM("B:\n"<<B_);
  // ROS_DEBUG_STREAM("C:\n"<<C_);
    
  this->sys_params_set_ = true;
}

//-----------------------------------------------------------
// Get System Parameters
bool CoopGT::getSysParams(Eigen::MatrixXd& A,
                       Eigen::MatrixXd& B,
                       Eigen::MatrixXd& C)
{
  if(!this->sys_params_set_)
  {

    std::cerr << "System parameters not set. return." << std::endl;
    // ROS_ERROR("system params not yet set. return");
    return false;
  }
  
  A = this->A_;
  B = this->B_;
  C = this->C_;
  
  return true;
}

//-----------------------------------------------------------
// Set Cost Parameters
void CoopGT::setCostsParams(const Eigen::MatrixXd& Q11,
                            const Eigen::MatrixXd& Q12,
                            const Eigen::MatrixXd& Q21,
                            const Eigen::MatrixXd& Q22,
                            const Eigen::MatrixXd& R1,
                            const Eigen::MatrixXd& R2)
{
  this->Q11_ = Q11;
  this->Q12_ = Q12;
  this->Q21_ = Q21;
  this->Q22_ = Q22;
  this->R1_  = R1;
  this->R2_  = R2;

  this->updateGTMatrices();
  
  this->cost_params_set_ = true;
}

//-----------------------------------------------------------
// Get Cost Matrices
bool CoopGT::getCostMatrices(Eigen::MatrixXd& Q1,
                             Eigen::MatrixXd& Q2,
                             Eigen::MatrixXd& R1,
                             Eigen::MatrixXd& R2)
{
  if (!this->cost_params_set_)
  {
    std::cerr << "Cost parameters not yet set" << std::endl;
    // ROS_ERROR("Cost params not yet set");
    return false;
  }
  
  Q1 = this->Q1_;
  Q2 = this->Q2_;
  R1 = this->R1_;
  R2 = this->R2_;
  
  return true;
}

//-----------------------------------------------------------
// Set Alpha
bool CoopGT::setAlpha(const double& alpha)
{
  if(alpha>1 || alpha <0)
  {
    //TODO: Introduce a better logic for this
    std::cerr << "weight alpha must be 0 < alpha < 1 . Current value of alpha: "<<alpha << std::endl;
    // ROS_ERROR_STREAM("weight alpha must be 0 < alpha < 1 . Current value of alpha: "<<alpha);
    return false;
  }
  this->alpha_ = alpha;
  this->alpha_set_ = true;
  return true;
}

//-----------------------------------------------------------
// Set Current State
bool CoopGT::setCurrentState(const Eigen::VectorXd& x)
{  
  if (x.size() != 2 * this->n_dofs_)
  {
    std::cerr << "State size is not correct. got: "<< x.size()<<", required: "<< 2 * this->n_dofs_<<std::endl;
    // ROS_ERROR_STREAM("State size is not correct. got: "<< x.size()<<", required: "<< 2*n_dofs_);
    return false;
  }

  this->X_ = x;
  this->state_ok_ = true;
  return true;
}

//-----------------------------------------------------------
// Update GT Matrices (No alpha)
bool CoopGT::updateGTMatrices()
{
  if(!this->alpha_set_)
  {
    std::cerr << "Parameter alpha not yet set!" << std::endl;
    // ROS_ERROR("parameter alpha not yet set! ");
    return false;
  }
    
  this->updateGTMatrices(this->alpha_);
  return true;
}

//-----------------------------------------------------------
// Update GT Matrices (With alpha)
void CoopGT::updateGTMatrices(const double& alpha )
{

  //* For a better understanding check the paper:
  //* Human-Robot Role Arbitration via Differential Game Theory (Franceschi et al.)
  this->Q1_ = alpha * this->Q11_ + (1-alpha) * this->Q21_;
  this->Q2_ = alpha * this->Q12_ + (1-alpha) * this->Q22_;
  
  this->Q_gt_ = this->Q1_ + this->Q2_;

  this->R_gt_.topLeftCorner(this->R1_.rows(), this->R1_.cols()) = alpha * this->R1_;
  this->R_gt_.bottomRightCorner(this->R1_.rows(), this->R1_.cols()) = (1-alpha) * this->R2_;
}

//-----------------------------------------------------------
// Get Current State
Eigen::VectorXd CoopGT::getCurrentState(){return this->X_;};

//-----------------------------------------------------------
// Compute Cooperative Gains
void CoopGT::computeCooperativeGains(const double& alpha)
{
  this->setAlpha(alpha);
  if(!this->updateGTMatrices())
    std::cerr << "Something wrong in updating matrices" << std::endl;
    // ROS_ERROR("comething wrong in updating matrices");
  this->computeCooperativeGains(this->Q_gt_, this->R_gt_);
}

//-----------------------------------------------------------
// Compute Cooperative Gains (No Inputs)
void CoopGT::computeCooperativeGains()
{
  computeCooperativeGains(this->Q_gt_, this->R_gt_);
}

//-----------------------------------------------------------
// Compute Cooperative Gains (With Inputs Q, R)
void CoopGT::computeCooperativeGains(const Eigen::MatrixXd& Q, const Eigen::MatrixXd& R)
{
  Eigen::MatrixXd B_gt; B_gt.resize(this->B_.rows(), 2 * this->B_.cols());
  B_gt << this->B_, this->B_;
  Eigen::MatrixXd P_cgt;
  Eigen::MatrixXd kk = this->solveRiccati(this->A_, B_gt, Q, R, P_cgt);
//   K_cgt_ = kk.diagonal().asDiagonal();
  this->K_cgt_.topLeftCorner    (this->n_dofs_, this->n_dofs_) = kk.topLeftCorner(this->n_dofs_, this->n_dofs_).diagonal().asDiagonal();
  this->K_cgt_.topRightCorner   (this->n_dofs_, this->n_dofs_) = kk.topRightCorner(this->n_dofs_, this->n_dofs_).diagonal().asDiagonal();
  this->K_cgt_.bottomRightCorner(this->n_dofs_, this->n_dofs_) = kk.bottomRightCorner(this->n_dofs_, this->n_dofs_).diagonal().asDiagonal();
  this->K_cgt_.bottomLeftCorner (this->n_dofs_, this->n_dofs_) = kk.bottomLeftCorner (this->n_dofs_, this->n_dofs_).diagonal().asDiagonal();
  // ROS_DEBUG_STREAM("K_cgt\n: "<<K_cgt_);
  std::cout << "K_cgt:\n" << this->K_cgt_ << std::endl;
  this->gains_set_ = true;
}

//-----------------------------------------------------------
// Get Cooperative Gains
Eigen::MatrixXd CoopGT::getCooperativeGains()
{
  if(!this->gains_set_)
    std::cerr << "Gains have not yet been computed!" << std::endl;
    // ROS_WARN_STREAM("gains have not yet been computed ! ");
  return this->K_cgt_;
}

//-----------------------------------------------------------
// Set Position Reference
bool CoopGT::setPosReference(const Eigen::VectorXd& ref_1, const Eigen::VectorXd& ref_2)
{
  if(ref_1.size() < this->n_dofs_ || ref_2.size() < this->n_dofs_)
  {
    std::cerr << "reference vectors have wrong length. Expected: "<< this->n_dofs_ <<", got ref_1: "<<ref_1.size()<<" and ref_2: "<<ref_2.size() << std::endl;
    // ROS_ERROR_STREAM("reference vectors have wrong length. Expected: "<<n_dofs_<<", got ref_1: "<<ref_1.size()<<" and ref_2: "<<ref_2.size() );
    return false;
  }
  
  Eigen::VectorXd r1; r1.resize(2 * this->n_dofs_); r1.setZero();
  Eigen::VectorXd r2; r2.resize(2 * this->n_dofs_); r2.setZero();
  
  r1.segment(0, this->n_dofs_) = ref_1;
  r2.segment(0, this->n_dofs_) = ref_2;
  if(!this->setReference(r1,r2))
    return false;
    
  return true;
}

//-----------------------------------------------------------
// Set Reference
bool CoopGT::setReference(const Eigen::VectorXd& ref_1, const Eigen::VectorXd& ref_2)
{
  if(ref_1.size()< 2 * this->n_dofs_ || ref_2.size()< 2 * this->n_dofs_)
  {
    std::cerr << "reference vectors have wrong length. Expected: "<<2 * this->n_dofs_<<", got ref_1: "<<ref_1.size()<<" and ref_2: "<<ref_2.size() << std::endl;
    // ROS_ERROR_STREAM("reference vectors have wrong length. Expected: "<<2*n_dofs_<<", got ref_1: "<<ref_1.size()<<" and ref_2: "<<ref_2.size() );
    return false;
  }
  
  // Shared Reference computation
  this->reference_ = this->Q_gt_.inverse() * (this->Q1_ * ref_1 + this->Q2_ * ref_2);
  
  this->reference_ok_ = true;
  return true;
}

//-----------------------------------------------------------
// Get Reference
Eigen::VectorXd CoopGT::getReference()
{
  return this->reference_;
}

//-----------------------------------------------------------
// Compute Control Inputs
Eigen::VectorXd CoopGT::computeControlInputs()
{
  if (!this->state_ok_)
  {
    std::cerr << "State is not updated. Computing gains on the last state received: " << this->X_.transpose() << std::endl;
    // ROS_WARN_STREAM("State is not updated. computing gains on the last state received: " << X_.transpose());
  }
  if (!this->reference_ok_)
  {
    std::cerr << "Reference is not updated. Computing gains on the last reference received: " << this->reference_.transpose() << std::endl;
    // ROS_WARN_STREAM("Reference is not updated. computing gains on the last reference received: " << reference_.transpose());
  }
  
  this->state_ok_     = false;
  this->reference_ok_ = false;
  
  Eigen::VectorXd control = -this->K_cgt_ * this->X_ + this->K_cgt_ * this->reference_;
  // TODO: Adjust these. Fixes angular velocity commands to 0. It is good for now though
  // TODO: Check conisistency
  if(this->n_dofs_ > 3)
  {
    control(3) = 0;
    control(4)= 0;
    
    control(9) = 0;
    control(10)= 0;
  }
  return control;
}


//-----------------------------------------------------------
// Step the system
Eigen::VectorXd CoopGT::step(const Eigen::VectorXd& x, const Eigen::VectorXd& ref_1, const Eigen::VectorXd& ref_2)
{
  if (x.size() != 2 * this->n_dofs_)
  {
    std::cerr << "State size is not correct. got: "<< x.size()<<", required: "<< 2 * this->n_dofs_<<std::endl;
    // ROS_ERROR_STREAM("State size is not correct. got: "<< x.size()<<", required: "<< 2*n_dofs_);
  }
  if (!this->sys_params_set_)
  {
    std::cerr << "System parameters not set. use setC2DSysParams or setSysParams to set the parameters before !" << std::endl;
    // ROS_ERROR_STREAM("System parameters not set. use setC2DSysParams or setSysParams to set the parameters before !");
  }

  //? The function setC2DSysParams is not defined in the class
  
  if(ref_1.size()!=ref_2.size())
    std::cerr << "references are not the same size ! " << std::endl;
    // ROS_ERROR_STREAM("references are not the same size ! ");
  if(ref_1.size() == this->n_dofs_ && ref_2.size() == this->n_dofs_)
    this->setPosReference(ref_1,ref_2);
  else if(ref_1.size() == 2 * this->n_dofs_ && ref_2.size() == 2 * this->n_dofs_)
    this->setReference(ref_1,ref_2);
  else
    std::cerr << "references have an incorrect length ."<<std::endl;
    // ROS_ERROR("references have an incorrect length .");
  
  Eigen::VectorXd u = this->computeControlInputs();
  
  
  this->setCurrentState(x);
  this->dX_ = this->A_ * this->X_ + this->B_ * u.segment(0, this->n_dofs_) + this->B_ * u.segment(this->n_dofs_, this->n_dofs_);
  this->X_ = this->X_ + this->dX_*dt_; //TODO Forward Euler. Modify. Could use exact discretization
  
  return this->X_;
}

Eigen::VectorXd CoopGT::step(const Eigen::VectorXd& ref_1, const Eigen::VectorXd& ref_2)
{
  return this->step(this->X_, ref_1, ref_2);
}

//*-------------------------*//
//*--- Protected Methods ---*//
//*-------------------------*//

//-----------------------------------------------------------
// Solve Riccati Equation
Eigen::MatrixXd CoopGT::solveRiccati(const Eigen::MatrixXd &A,
                                  const Eigen::MatrixXd &B,
                                  const Eigen::MatrixXd &Q,
                                  const Eigen::MatrixXd &R, Eigen::MatrixXd &P)
{
  const uint dim_x = A.rows();
  const uint dim_u = B.cols();

  Eigen::MatrixXd Ham = Eigen::MatrixXd::Zero(2 * dim_x, 2 * dim_x);
  Ham << A, -B * R.inverse() * B.transpose(), -Q, -A.transpose();

  Eigen::EigenSolver<Eigen::MatrixXd> Eigs(Ham);

  Eigen::MatrixXcd eigvec = Eigen::MatrixXcd::Zero(2 * dim_x, dim_x);
  int j = 0;
  for (int i = 0; i < 2 * dim_x; ++i) {
    if (Eigs.eigenvalues()[i].real() < 0.) {
      eigvec.col(j) = Eigs.eigenvectors().block(0, i, 2 * dim_x, 1);
      ++j;
    }
  }

  Eigen::MatrixXcd Vs_1, Vs_2;
  Vs_1 = eigvec.block(0, 0, dim_x, dim_x);
  Vs_2 = eigvec.block(dim_x, 0, dim_x, dim_x);
  P = (Vs_2 * Vs_1.inverse()).real();
  
  return R.inverse()*B.transpose()*P;
}

//-----------------------------------------------------------
// Solve Nash Equilibrium
void CoopGT::solveNashEquilibrium( const Eigen::MatrixXd &A,
                                const Eigen::MatrixXd &B1,
                                const Eigen::MatrixXd &B2,
                                const Eigen::MatrixXd &Q1,
                                const Eigen::MatrixXd &Q2,
                                const Eigen::MatrixXd &R1,
                                const Eigen::MatrixXd &R2, 
                                const Eigen::MatrixXd &R12,
                                const Eigen::MatrixXd &R21, 
                                Eigen::MatrixXd &P1,Eigen::MatrixXd &P2)
{
  Eigen::MatrixXd S1  = B1 * R1.inverse() * B1.transpose();
  Eigen::MatrixXd S2  = B2 * R2.inverse() * B2.transpose();
  Eigen::MatrixXd S12 = B1 * R1.inverse() * R21 * R1.inverse() * B1.transpose();
  Eigen::MatrixXd S21 = B2 * R2.inverse() * R12 * R2.inverse()* B2.transpose();

  this->solveRiccati(A,B1,Q1,R1,P1);
  this->solveRiccati(A,B2,Q2,R2,P2);
  
  Eigen::MatrixXd P1_prev = P1;
  Eigen::MatrixXd P2_prev = P2;
  double err_1 = 1;
  double err_2 = 1;
  double toll = 0.00001;

  //! Exit condition has been changed from original code
  while (err_1 > toll || err_2 > toll)
  {    
    Eigen::MatrixXd A1 = A - S2*P2;
    Eigen::MatrixXd A2 = A - S1*P1;
    
    Eigen::MatrixXd Q_1 = Q1 + P1*S21*P1;
    this->solveRiccati(A1,B1,Q_1,R1,P1);
    Eigen::MatrixXd Q_2 = Q2 + P2*S12*P2;
    this->solveRiccati(A2,B2,Q_2,R2,P2);
  
    err_1 = (P1-P1_prev).norm();
    err_2 = (P2-P2_prev).norm();
    
    P1_prev = P1;
    P2_prev = P2;
  }
  
  return;
}




