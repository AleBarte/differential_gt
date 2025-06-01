cmake_minimum_required(VERSION 3.5)
project(differential_gt)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(include)

# Create the executable
add_executable(differential_gt_node src/differential_gt_node.cpp)

# Link libraries
ament_target_dependencies(differential_gt_node rclcpp Eigen3)

# Install the executable
install(TARGETS
  differential_gt_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install include files
install(DIRECTORY include/
  DESTINATION include
)

# Package information
ament_package()