#pragma once

#include <Eigen/Dense>
#include <vector>
#include <iostream>

// Experts
std::vector<double> expert_directional(
    const Eigen::VectorXd& x,
    const Eigen::VectorXd& u_h,
    const std::vector<Eigen::VectorXd>& goals,
    double kappa = 5.0);

std::vector<double> expert_distance(
    const Eigen::VectorXd& x,
    const std::vector<Eigen::VectorXd>& goals,
    double kappa = 1.0);

double compute_max_alignment(
    const Eigen::VectorXd& x,
    const Eigen::VectorXd& u_h,
    const std::vector<Eigen::VectorXd>& goals);



// Gating
std::vector<double> compute_gating_weights(const Eigen::VectorXd& u_h);

std::vector<double> compute_gating_weights_with_alignment(
    const Eigen::VectorXd& x,
    const Eigen::VectorXd& u_h,
    const std::vector<Eigen::VectorXd>& goals);

// MoE
Eigen::VectorXd compute_moe_posterior(
    const Eigen::VectorXd& x,
    const Eigen::VectorXd& u_h,
    const std::vector<Eigen::VectorXd>& goals,
    double& H);
