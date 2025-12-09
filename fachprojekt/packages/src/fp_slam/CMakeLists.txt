cmake_minimum_required(VERSION 3.8)
project(fp_slam)

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(slam_toolbox REQUIRED)
find_package(rclcpp REQUIRED)

#
# INSTALL config + launch directories (required for assignment)
#
install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

#
# Package declaration
#
ament_package()

