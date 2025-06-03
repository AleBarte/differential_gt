#include "../include/differential_gt/differential_gt.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DifferentialGT>("differential_gt_node");
    
    // Spin the node
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
}