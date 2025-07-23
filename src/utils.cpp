#include "../include/differential_gt/utils.hpp"

void Utils::saveMultipleMatrices(const std::string& filename, const std::vector<Eigen::MatrixXd>& matrices)
{
    std::ofstream file(filename, std::ios::binary);
    if (!file) throw std::runtime_error("Cannot open file for writing.");

    int numMatrices = matrices.size();
    file.write(reinterpret_cast<const char*>(&numMatrices), sizeof(int));

    for (const auto& mat : matrices) {
        int rows = mat.rows();
        int cols = mat.cols();
        file.write(reinterpret_cast<const char*>(&rows), sizeof(int));
        file.write(reinterpret_cast<const char*>(&cols), sizeof(int));
        file.write(reinterpret_cast<const char*>(mat.data()), sizeof(double) * rows * cols);
    }

    file.close();
}

std::vector<Eigen::MatrixXd> Utils::loadMultipleMatrices(const std::string& filename)
{
    std::ifstream file(filename, std::ios::binary);
    if (!file) throw std::runtime_error("Cannot open file for reading.");

    int numMatrices = 0;
    file.read(reinterpret_cast<char*>(&numMatrices), sizeof(int));

    std::vector<Eigen::MatrixXd> matrices;
    for (int i = 0; i < numMatrices; ++i) {
        int rows = 0, cols = 0;
        file.read(reinterpret_cast<char*>(&rows), sizeof(int));
        file.read(reinterpret_cast<char*>(&cols), sizeof(int));

        Eigen::MatrixXd mat(rows, cols);
        file.read(reinterpret_cast<char*>(mat.data()), sizeof(double) * rows * cols);

        matrices.push_back(mat);
    }

    file.close();
    return matrices;
}