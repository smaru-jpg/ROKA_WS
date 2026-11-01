cmake_minimum_required(VERSION 3.8)
project(roka_imu_driver)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)

add_executable(ebimu_v5_driver_node src/ebimu_v5_driver_node.cpp)
target_include_directories(ebimu_v5_driver_node PRIVATE include)
ament_target_dependencies(ebimu_v5_driver_node
  diagnostic_msgs
  geometry_msgs
  rclcpp
  sensor_msgs
  std_msgs
  std_srvs
  tf2
)

install(TARGETS ebimu_v5_driver_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config include
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_ebimu_v5_protocol test/test_ebimu_v5_protocol.cpp)
  target_include_directories(test_ebimu_v5_protocol PRIVATE include)
endif()

ament_export_dependencies(
  diagnostic_msgs
  geometry_msgs
  rclcpp
  sensor_msgs
  std_msgs
  std_srvs
  tf2
)

ament_package()
