#include "../include/differential_gt/arbitration.hpp"

//*--------------------------*//
//*----- Constructor(s) -----*//
//*--------------------------*//

Arbitration::Arbitration(double cosine_similarity_threshold, double epsilon) :
    cosine_similarity_threshold_(cosine_similarity_threshold),
    epsilon_(epsilon) // Initialize the threshold and epsilon
{
    if (this->cosine_similarity_threshold_ < 0.0)
    {
        std::cerr << "Cosine similarity threshold must be non-negative. Setting to default value of 0.5." << std::endl;
        this->cosine_similarity_threshold_ = 0.5; // Default value
    } else if (std::abs(this->cosine_similarity_threshold_) > 1.0)
    {
        std::cerr << "Cosine similarity threshold must be between -1 and 1. Setting to default value of 0.5." << std::endl;
        this->cosine_similarity_threshold_ = 0.5; // Default value
    }
}

Arbitration::Arbitration() :
    cosine_similarity_threshold_(0.5), // Default value
    epsilon_(1e-5) // Default small value to avoid division by zero
{

}

Arbitration::Arbitration(double cosine_similarity_threshold)
    : cosine_similarity_threshold_(cosine_similarity_threshold),
      epsilon_(1e-5) // Default small value to avoid division by zero
{}

//*---------------------------*//
//*----- Public Methods ------*//
//*---------------------------*//

//------------------------------------------------------------
// Cosine Similarity
void Arbitration::CosineSimilarity(Eigen::VectorXd& v1, Eigen::VectorXd& v2, double& cos_theta, int& decision)
{
    // Calculate the cosine of the angle between the two vectors
    cos_theta = v1.dot(v2) / (v1.norm() * v2.norm() + this->epsilon_);

    // Check if the cosine value is below a certain threshold
    if (cos_theta > this->cosine_similarity_threshold_)
    {
        decision = 0; // Decision 1
    }
    else
    {
        decision = 1; // Decision 0
    }
}

//------------------------------------------------------------
// Cosine Similarity with Hysteresis
void Arbitration::CosineSimilarityHysteresis(Eigen::VectorXd& v1, Eigen::VectorXd& v2, double& cos_theta, int& decision, double switch_on_point, double switch_off_point)
{
    // Calculate the cosine of the angle between the two vectors
    cos_theta = v1.dot(v2) / (v1.norm() * v2.norm() + this->epsilon_);

    if (cos_theta > switch_on_point && this->hysteresis_decision_ == 1)
    {
        this->hysteresis_decision_ = 0;
    }else if (cos_theta < switch_off_point && this->hysteresis_decision_ == 0)
    {
        this->hysteresis_decision_ = 1;
    }

    decision = this->hysteresis_decision_;

}

//---------------------------------------------------------------
// Cosine similarity nearest vector
void Arbitration::CosineSimilarityNearestVector(Eigen::VectorXd& v1, Eigen::VectorXd& v2, Eigen::VectorXd& v3, double& cos_theta12, double& cos_theta13, int& decision)
{
    cos_theta12 = v1.dot(v2) / (v1.norm() * v2.norm() + this->epsilon_);
    cos_theta13 = v1.dot(v3) / (v1.norm() * v3.norm() + this->epsilon_);

    if (cos_theta12 >= cos_theta13)
    {
        decision = 0;
    } else {
        decision = 1;
    }
}
