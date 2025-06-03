cmake_minimum_required(VERSION 3.5)
project(differential_gt)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_kdl REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)


include_directories(include)

add_library(gt_lib
  src/cgt.cpp
  src/ncgt.cpp
  src/arbitration.cpp
)

# Create the executable
add_executable(differential_gt_node src/dgt_node.cpp
)

add_executable(differential_gt 
              src/differential_gt_main.cpp
              src/differential_gt.cpp
)

target_link_libraries(differential_gt_node gt_lib)
target_link_libraries(differential_gt gt_lib)


# Link libraries
ament_target_dependencies(differential_gt_node rclcpp Eigen3)
ament_target_dependencies(differential_gt rclcpp geometry_msgs tf2_ros sensor_msgs Eigen3 kdl_parser tf2_eigen tf2_kdl std_msgs std_srvs)

# Install the executable
install(TARGETS
  differential_gt_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  differential_gt
  DESTINATION lib/${PROJECT_NAME}
)

# Install include files
install(DIRECTORY include/
  DESTINATION include
)

# Package information
ament_package()
