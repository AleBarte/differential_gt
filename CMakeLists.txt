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

# Create the library
add_library(gt_lib SHARED
  src/cgt.cpp
  src/ncgt.cpp
  src/arbitration.cpp
)

# Specify include directories for the library
target_include_directories(gt_lib PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)

# Specify dependencies for the library
ament_target_dependencies(gt_lib PUBLIC
  rclcpp
  Eigen3
)

# Create the executables
add_executable(differential_gt_node src/dgt_node.cpp)
add_executable(differential_gt 
  src/differential_gt_main.cpp
  src/differential_gt.cpp
  src/utils.cpp
  src/moe_goal_inference.cpp
)

target_link_libraries(differential_gt_node gt_lib)
target_link_libraries(differential_gt gt_lib)

# Link libraries
ament_target_dependencies(differential_gt_node rclcpp Eigen3)
ament_target_dependencies(differential_gt 
  rclcpp 
  geometry_msgs 
  tf2_ros 
  sensor_msgs 
  Eigen3 
  kdl_parser 
  tf2_eigen 
  tf2_kdl 
  std_msgs 
  std_srvs
)

# Install include files - NOTE: include/${PROJECT_NAME}
install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

# Install the library
install(
  TARGETS gt_lib
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# Install the executables
install(TARGETS
  differential_gt_node
  differential_gt
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

install(
  DIRECTORY saved_configs
  DESTINATION share/${PROJECT_NAME}
)

# Export the library
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp Eigen3)

# Package information
ament_package()