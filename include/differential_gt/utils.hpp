#ifndef UTILS_HPP_
#define UTILS_HPP_

#include <Eigen/Dense>
#include <string>
#include <fstream>
#include <vector>

class Utils {
public:
  static void saveMultipleMatrices(const std::string& filename, const std::vector<Eigen::MatrixXd>& matrices);
  static std::vector<Eigen::MatrixXd> loadMultipleMatrices(const std::string& filename);
};

#endif // UTILS_HPP_