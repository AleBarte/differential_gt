#ifndef ARBITRATION_HPP
#define ARBITRATION_HPP

#include <iostream>
#include <string>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>



class Arbitration 
{
public:
    // Constructors
    Arbitration();
    Arbitration(double cosine_similarity_threshold, double epsilon);
    
    // Method to perform I Level arbitration
    void CosineSimilarity(Eigen::VectorXd& v1, Eigen::VectorXd& v2, double& cos_theta, int& decision);
protected:

    double cosine_similarity_threshold_; // Default threshold for cosine similarity
    double epsilon_; // Small value to avoid division by zero
};
#endif