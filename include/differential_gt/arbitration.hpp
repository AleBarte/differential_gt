#ifndef ARBITRATION_HPP
#define ARBITRATION_HPP

#include <iostream>
#include <string>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <cmath>



class Arbitration 
{
public:
    // Constructors
    Arbitration();
    Arbitration(double cosine_similarity_threshold, double epsilon);
    Arbitration(double cosine_similarity_threshold);
    
    // Method to perform I Level arbitration
    void CosineSimilarity(Eigen::VectorXd& v1, Eigen::VectorXd& v2, double& cos_theta, int& decision);
    void CosineSimilarity(Eigen::Vector3d& v1, Eigen::Vector3d& v2, double& cos_theta, int& decision);

    // Method to perform I Level arbitration with hysteresis
    void CosineSimilarityHysteresis(Eigen::VectorXd& v1, Eigen::VectorXd& v2, double& cos_theta, int& decision, 
                                    double switch_on_point, double switch_off_point);
    void CosineSimilarityHysteresis(Eigen::Vector3d& v1, Eigen::Vector3d& v2, double& cos_theta, int& decision, 
                                    double switch_on_point, double switch_off_point);

    
    // Other methods 
    void CosineSimilarityNearestVector(Eigen::VectorXd& v1, Eigen::VectorXd& v2, Eigen::VectorXd& v3, double& cos_theta12, double& cos_theta13, int& decision);
    void CosineSimilarityFiltered(Eigen::VectorXd& v1, Eigen::VectorXd& v2, double& cos_theta, int& decision, double alpha);
    void FirstLevelSimilarity(Eigen::VectorXd& v1, Eigen::VectorXd& v2, double& dec, int& decision, double forgetting_factor);
    void ConeSimilarity(Eigen::VectorXd& v1, Eigen::VectorXd& a1, Eigen::VectorXd& b1, int& decision, double& alpha);
    void ConeSimilarityFiltered(Eigen::VectorXd& v1, Eigen::VectorXd& a1, Eigen::VectorXd& b1, int& decision, double& alpha);
    double SecondLevelArbitrationACSOverride(Eigen::VectorXd& v1, Eigen::VectorXd& v2);
    double SecondLevelArbitrationSplit(Eigen::VectorXd& v1, Eigen::VectorXd& v2);
    double SecondLevelArbitrationACSOverrideFiltered(Eigen::VectorXd& v1, Eigen::VectorXd& v2, double alpha);

    // NOTE to Marco: Insert here other methods that you want to use to perform arbitration
    // 
    //-------------------------------------------------------------------------------------
protected:

    double cosine_similarity_threshold_; // Default threshold for cosine similarity
    double epsilon_; // Small value to avoid division by zero
    bool switch_on_triggered_ = true; // Flag for switch on state
    bool switch_off_triggered_ = false; // Flag for switch off state
    int hysteresis_decision_ = 0;

    double cos_theta_prev_ = 0.0;
    double arbitration_acs_override_ = 0.0; // Variable to store the arbitration value for ACS override
    double score_ = 0.0;
    double filtered_decision_ = 0.0;

};
#endif