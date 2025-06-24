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

//---------------------------------------------------------------
// Cosine Similarity Filtered
void Arbitration::CosineSimilarityFiltered(Eigen::VectorXd& v1, Eigen::VectorXd& v2, double& cos_theta, int& decision, double alpha)
{
    // Calculate the cosine of the angle between the two vectors
    cos_theta = v1.dot(v2) / (v1.norm() * v2.norm() + this->epsilon_);

    // Apply the filter to the cosine value
    cos_theta = alpha * cos_theta + (1 - alpha) * this->cos_theta_prev_;

    // Check if the cosine value is below a certain threshold
    if (cos_theta > this->cosine_similarity_threshold_)
    {
        decision = 0; // Decision 1
    }
    else
    {
        decision = 1; // Decision 0
    }
    this->cos_theta_prev_ = cos_theta; // Update the previous cosine value
}

//---------------------------------------------------------------
// Second Level Arbitration ACS Override
double Arbitration::SecondLevelArbitrationACSOverride(Eigen::VectorXd& v1, Eigen::VectorXd& v2)
{
    // Compute the projection of v1 onto v2
    Eigen::VectorXd v1_proj = (v1.dot(v2) / (v2.dot(v2) + this->epsilon_)) * v2;
    
    // Compute the orthogonal projection of v1 onto v2
    Eigen::VectorXd v1_oproj = v1 - v1_proj;
    
    double chi = v1_proj.norm() / v2.norm();
    double psi = std::pow(v1_oproj.norm() / v1.norm(), 2);

    double alpha = std::max(0.001, std::min(0.999, chi) - psi);

    return alpha;
}

//---------------------------------------------------------------
// Second Level Arbitration Split
double Arbitration::SecondLevelArbitrationSplit(Eigen::VectorXd& v1, Eigen::VectorXd& v2)
{
    // Compute the projection of v1 onto v2
    Eigen::VectorXd v1_proj = (v1.dot(v2) / (v2.dot(v2) + this->epsilon_)) * v2;
    
    // Compute the orthogonal projection of v1 onto v2
    Eigen::VectorXd v1_oproj = v1 - v1_proj;
    
    double chi = v1_proj.norm() / v2.norm();

    double alpha_prime = std::min(0.5, 0.5 * chi);
    double psi = std::pow(v1_oproj.norm() / v1.norm(), 2);

    double alpha = std::min(1.0, alpha_prime + psi);

    return alpha;
}

//---------------------------------------------------------------
// Second Level Arbitration ACS Override Filtered
double Arbitration::SecondLevelArbitrationACSOverrideFiltered(Eigen::VectorXd& v1, Eigen::VectorXd& v2, double alpha)
{
    // Compute the projection of v1 onto v2
    Eigen::VectorXd v1_proj = (v1.dot(v2) / (v2.dot(v2) + this->epsilon_)) * v2;
    
    // Compute the orthogonal projection of v1 onto v2
    Eigen::VectorXd v1_oproj = v1 - v1_proj;
    
    double chi = v1_proj.norm() / v2.norm();
    double psi = std::pow(v1_oproj.norm() / v1.norm(), 2);

    double alpha_prime = std::max(0.5, std::min(0.999, chi) - psi);
    double alpha_filtered = alpha * alpha_prime + (1 - alpha) * this->arbitration_acs_override_;

    this->arbitration_acs_override_ = alpha_filtered;

    return alpha_filtered;
}

//---------------------------------------------------------------
// Cone Similarity
void Arbitration::ConeSimilarity(Eigen::VectorXd& v1, Eigen::VectorXd& a1, Eigen::VectorXd& b1, int& decision, double& alpha)
{
    // Calculate the cross product between a and b

    Eigen::Vector3d a = a1.head<3>();
    Eigen::Vector3d b = b1.head<3>();
    Eigen::Vector3d v = v1.head<3>();

    Eigen::VectorXd n = a.cross(b);
    Eigen::VectorXd n_normalized = n.normalized();

    // Compute the cross product between v and a and v and b
    Eigen::VectorXd cross_v_a = v.cross(a);
    Eigen::VectorXd cross_v_b = v.cross(b);

    double cos_gamma = a.dot(b) / (a.norm() * b.norm() + this->epsilon_); // Cosine of the angle between a and b
    double cos_delta = v.dot(a) / (v.norm() * a.norm() + this->epsilon_); // Cosine of the angle between v and a
    cos_gamma = std::max(-1.0, std::min(1.0, cos_gamma)); // Clamp the value to [-1, 1]
    cos_delta = std::max(-1.0, std::min(1.0, cos_delta)); // Clamp the value to [-1, 1]
    double gamma = std::acos(cos_gamma); // Angle between a and b
    double delta = std::acos(cos_delta); // Angle between v and a



    // Check how large the cone is
    if (gamma < 0.08)
    {
        std::cout << "Cone is very small." << std::endl;
        decision = 0; // Non-Cooperation
        alpha = 0.5; // No cooperation
    } else if (cross_v_a.dot(n) >= 0 && cross_v_b.dot(n) >= 0 && v.dot(a) >= 0 && v.dot(b) >= 0) {
            decision = 0; // Inside the cone
            Eigen::VectorXd projection = v - (v.dot(n_normalized) * n_normalized); // Computing projection onto the plane defined by the a and b
            alpha = std::max(0.001, std::min(0.999, delta / gamma));
            std::cout << "Inside the cone: alpha = " << alpha << ", cos_gamma = " << cos_gamma << ", cos_delta = " << cos_delta << std::endl;           
    } else {
        decision = 1; // Outside the cone
        alpha = 0.001; // No cooperation
    }
    

}

    
// Cone Similarity Filtered
void Arbitration::ConeSimilarityFiltered(Eigen::VectorXd& v1, Eigen::VectorXd& a1, Eigen::VectorXd& b1, int& decision, double& alpha)
{
    double alpha1 = 0.01;
    double alpha2 = 0.01;

    this->ConeSimilarity(v1, a1, b1, decision, alpha); // First level cone similarity

    double dec = static_cast<double>(decision); // Convert decision to double for filtering
    // Apply the filter to the decision value
    this->filtered_decision_ = alpha1 * dec + (1 - alpha1) * this->filtered_decision_;

    if (this->filtered_decision_ >= 0.5) {
        decision = 1; // Cooperation
    } else {
        decision = 0; // Non-Cooperation
    }

    // Apply the filter to the alpha value
    this->score_ = alpha2 * alpha + (1 - alpha2) * this->score_;
    alpha = std::max(0.001, std::min(0.999, this->score_)); // Clamp the value to [0.001, 0.999]

}



void Arbitration::FirstLevelSimilarity(Eigen::VectorXd& v1, Eigen::VectorXd& v2, double& dec, int& decision, double forgetting_factor)
{
    // Calculate the cosine of the angle between the two vectors
    double cos_theta = v1.dot(v2) / (v1.norm() * v2.norm() + this->epsilon_);

    this->score_ = forgetting_factor * this->score_ + cos_theta;

    // Pass the score through a nonlinear map
    dec = 1 / (1 + std::exp(-this->score_)); // Sigmoid function

    if (dec >= 0.5) {
        decision = 0; // Cooperation
    } else {
        decision = 1; // Non-Cooperation
    }
}