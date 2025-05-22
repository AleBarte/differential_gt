#include "../include/differential_gt/arbitration.hpp"

//*--------------------------*//
//*----- Constructor(s) -----*//
//*--------------------------*//

Arbitration::Arbitration(double cosine_similarity_threshold, double epsilon) :
    cosine_similarity_threshold_(cosine_similarity_threshold),
    epsilon_(epsilon) // Initialize the threshold and epsilon
{
    // Constructor implementation (For now, it's empty)
}

Arbitration::Arbitration() :
    cosine_similarity_threshold_(0.5), // Default value
    epsilon_(1e-5) // Default small value to avoid division by zero
{

}


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

