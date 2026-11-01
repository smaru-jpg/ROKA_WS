cmake_minimum_required(VERSION 3.8)
project(roka_terrain_supervisor)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(roka_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(grid_map_core REQUIRED)
find_package(grid_map_msgs REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(terrain_supervisor_node src/terrain_supervisor_node.cpp)
target_include_directories(terrain_supervisor_node PRIVATE include)
ament_target_dependencies(terrain_supervisor_node
  diagnostic_msgs roka_interfaces geometry_msgs grid_map_core grid_map_msgs grid_map_ros
  nav_msgs rclcpp sensor_msgs std_msgs std_srvs tf2 tf2_geometry_msgs tf2_ros)

add_executable(wheel_contact_estimator_node src/wheel_contact_estimator_node.cpp)
target_include_directories(wheel_contact_estimator_node PRIVATE include)
ament_target_dependencies(wheel_contact_estimator_node
  diagnostic_msgs roka_interfaces nav_msgs rclcpp sensor_msgs std_msgs)

install(TARGETS terrain_supervisor_node wheel_contact_estimator_node
  DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/lifecycle_activator.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME roka_lifecycle_activator)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_supervisor_math test/test_supervisor_math.cpp)
  target_include_directories(test_supervisor_math PRIVATE include)
endif()

ament_package()
