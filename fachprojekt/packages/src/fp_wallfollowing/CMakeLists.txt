cmake_minimum_required(VERSION 3.8)
project(fp_wallfollowing )

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_srvs REQUIRED)

# Executable hinzufügen
add_executable(fp_wallfollowing src/wallfollowing_node.cpp)

 #Dependencies für Executable 
 ament_target_dependencies(fp_wallfollowing
  rclcpp
  std_msgs
  sensor_msgs
  nav_msgs
  std_srvs
  )
#Installieren des Nodes
install(TARGETS
  fp_wallfollowing
  DESTINATION lib/${PROJECT_NAME}
)

# Testsp
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()