cmake_minimum_required(VERSION 3.8)
project(mobile_robot_lab_cpp)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(obstacle_avoidance_cpp src/obstacle_avoidance.cpp)
target_include_directories(obstacle_avoidance_cpp PRIVATE include)
ament_target_dependencies(
  obstacle_avoidance_cpp geometry_msgs rclcpp sensor_msgs)

install(TARGETS obstacle_avoidance_cpp DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include/)

if(BUILD_TESTING)
  enable_testing()
  add_executable(scan_utils_test test/scan_utils_test.cpp)
  target_include_directories(scan_utils_test PRIVATE include)
  ament_target_dependencies(scan_utils_test sensor_msgs)
  add_test(NAME scan_utils_test COMMAND scan_utils_test)

  add_executable(camera_lidar_fusion_test test/camera_lidar_fusion_test.cpp)
  target_include_directories(camera_lidar_fusion_test PRIVATE include)
  add_test(NAME camera_lidar_fusion_test COMMAND camera_lidar_fusion_test)
endif()

ament_package()
