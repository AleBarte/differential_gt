#include "../include/differential_gt/moe_goal_inference.hpp"
#include <cmath>
#include <numeric>

using namespace Eigen;

std::vector<double> expert_directional(
    const VectorXd& x,
    const VectorXd& u_h,
    const std::vector<VectorXd>& goals,
    double kappa)
{
    std::vector<double> scores;
    double norm_u = u_h.norm();
    double max_score = -1e9;

    for (const auto& g : goals) {
        VectorXd dir = g - x;
        double norm_dir = dir.norm();
        double cos_theta = 0.0;

        if (norm_u > 1e-6 && norm_dir > 1e-6) {
            cos_theta = u_h.dot(dir) / (norm_u * norm_dir);
        }

        double score = kappa * cos_theta;
        scores.push_back(score);
        if (score > max_score) max_score = score;
        max_score = 0; //!
    }

    // Softmax
    std::vector<double> probs;
    double sum_exp = 0.0;
    for (double s : scores) {
        double e = std::exp(s - max_score);
        probs.push_back(e);
        sum_exp += e;
    }

    for (auto& p : probs) p /= (sum_exp + 1e-6);
    return probs;
}

std::vector<double> expert_distance(
    const VectorXd& x,
    const std::vector<VectorXd>& goals,
    double kappa)
{
    std::vector<double> scores;
    double max_score = -1e9;

    for (const auto& g : goals) {
        double dist = (g - x).norm();
        double score = -kappa * dist;
        scores.push_back(score);
        if (score > max_score) max_score = score;
    }
    max_score = 0; //!

    std::vector<double> probs;
    double sum_exp = 0.0;
    for (double s : scores) {
        double e = std::exp(s - max_score);
        probs.push_back(e);
        sum_exp += e;
    }

    for (auto& p : probs) p /= (sum_exp + 1e-6);
    return probs;
}

double compute_max_alignment(
    const Eigen::VectorXd& x,
    const Eigen::VectorXd& u_h,
    const std::vector<Eigen::VectorXd>& goals)
{
    double norm_u = u_h.norm();
    if (norm_u < 1e-6) return 0.0;

    double max_cos = -1.0;
    for (const auto& g : goals) {
        Eigen::VectorXd dir = g - x;
        double norm_dir = dir.norm();
        if (norm_dir < 1e-6) continue;
        double cos_theta = u_h.dot(dir) / (norm_u * norm_dir);
        max_cos = std::max(max_cos, cos_theta);
    }

    return std::max(0.0, max_cos);  // ReLU
}

std::vector<double> compute_gating_weights(const VectorXd& u_h) {
    double mag = u_h.norm();
    double w1 = std::tanh(mag);      // more weight on directional expert as input gets stronger
    double w2 = 1.0 - w1;
    return {w1, w2};
}

// std::vector<double> compute_gating_weights_with_alignment(
//     const VectorXd& x,
//     const VectorXd& u_h,
//     const std::vector<VectorXd>& goals)
// {
//     double m = std::tanh(u_h.norm());  // input strength
//     double a = compute_max_alignment(x, u_h, goals);  // directionality
//     double w_dir = m * a;
//     double w_dist = 1.0 - w_dir;

//     return {w_dir, w_dist};
// }

std::vector<double> compute_gating_weights_with_alignment(
    const VectorXd& x,
    const VectorXd& u_h,
    const std::vector<VectorXd>& goals)
{
    // Input strength and alignment
    double m = std::tanh(u_h.norm());  // in [0,1]
    double a = compute_max_alignment(x, u_h, goals);  // in [0,1]
    double s_dir = m * a;

    // Distance-based expert score: inverse of min distance
    double min_d = std::numeric_limits<double>::max();
    for (const auto& g : goals) {
        double d = (g - x).norm();
        if (d < min_d) min_d = d;
    }
    double s_dist = 1.0 / (1.0 + min_d);  // in (0,1]

    // Softmax over expert scores
    double exp_dir = std::exp(s_dir);
    double exp_dist = std::exp(s_dist);
    double sum = exp_dir + exp_dist;

    double w_dir = exp_dir / sum;
    double w_dist = exp_dist / sum;

    return {w_dir, w_dist};
}

Eigen::VectorXd compute_moe_posterior(
    const VectorXd& x,
    const VectorXd& u_h,
    const std::vector<VectorXd>& goals,
    double& H)
{
    auto w = compute_gating_weights_with_alignment(x, u_h, goals);
    auto p_dir = expert_directional(x, u_h, goals);
    auto p_dist = expert_distance(x, goals);

    Eigen::VectorXd posterior(goals.size());
    for (size_t i = 0; i < goals.size(); ++i) {
        posterior[i] = w[0] * p_dir[i] + w[1] * p_dist[i];
    }

    // Compute entropy H
    H = 0.0;
    for (int i = 0; i < posterior.size(); ++i) {
        if (posterior[i] > 0.3) {
            H -= posterior[i] * std::log(posterior[i]);
        }
    }

    // Normalize
    double sum = std::accumulate(posterior.begin(), posterior.end(), 0.0);
    for (auto& p : posterior) p /= (sum + 1e-6);

    std::cout << "Posterior: " << posterior.transpose() << std::endl;
    return posterior;
}
