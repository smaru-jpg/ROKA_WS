cmake_minimum_required(VERSION 3.8)
project(roka_terrain_mapping)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(filters REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(grid_map_core REQUIRED)
find_package(grid_map_filters REQUIRED)
find_package(grid_map_msgs REQUIRED)
find_package(grid_map_pcl REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters io sample_consensus search segmentation)
find_package(pcl_conversions REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

add_executable(terrain_mapper_node src/terrain_mapper_node.cpp)
target_include_directories(terrain_mapper_node PRIVATE include)
ament_target_dependencies(terrain_mapper_node
  diagnostic_msgs geometry_msgs grid_map_core grid_map_msgs grid_map_ros
  nav_msgs rclcpp sensor_msgs std_srvs tf2 tf2_ros tf2_sensor_msgs)

add_executable(synthetic_terrain_node src/synthetic_terrain_node.cpp)
target_include_directories(synthetic_terrain_node PRIVATE include)
ament_target_dependencies(synthetic_terrain_node
  geometry_msgs nav_msgs rclcpp sensor_msgs tf2 tf2_ros)

add_executable(terrain_map_server_node src/terrain_map_server_node.cpp)
target_include_directories(terrain_map_server_node PRIVATE include)
ament_target_dependencies(terrain_map_server_node
  grid_map_core grid_map_msgs grid_map_ros rclcpp)

add_executable(terrain_obstacle_filter_node src/terrain_obstacle_filter_node.cpp)
target_include_directories(terrain_obstacle_filter_node PRIVATE include)
ament_target_dependencies(terrain_obstacle_filter_node
  grid_map_core grid_map_msgs grid_map_ros rclcpp sensor_msgs
  tf2 tf2_ros tf2_sensor_msgs)

add_executable(terrain_review_visualizer_node src/terrain_review_visualizer_node.cpp)
target_include_directories(terrain_review_visualizer_node PRIVATE include)
ament_target_dependencies(terrain_review_visualizer_node
  geometry_msgs grid_map_core grid_map_msgs grid_map_ros nav_msgs rclcpp std_srvs
  tf2 tf2_geometry_msgs tf2_ros visualization_msgs)

add_executable(terrain_standard_postprocessor_node src/terrain_standard_postprocessor_node.cpp)
target_include_directories(terrain_standard_postprocessor_node PRIVATE include ${PCL_INCLUDE_DIRS})
target_link_libraries(terrain_standard_postprocessor_node ${PCL_LIBRARIES})
ament_target_dependencies(terrain_standard_postprocessor_node
  filters grid_map_core grid_map_filters grid_map_pcl grid_map_ros rclcpp)

add_executable(leader_follower_node src/leader_follower_node.cpp)
target_include_directories(leader_follower_node PRIVATE include ${PCL_INCLUDE_DIRS})
target_link_libraries(leader_follower_node ${PCL_LIBRARIES})
ament_target_dependencies(leader_follower_node
  geometry_msgs grid_map_core nav_msgs pcl_conversions rclcpp sensor_msgs std_msgs std_srvs
  tf2 tf2_ros visualization_msgs)

install(TARGETS terrain_mapper_node synthetic_terrain_node terrain_map_server_node
  terrain_obstacle_filter_node terrain_review_visualizer_node
  terrain_standard_postprocessor_node leader_follower_node
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch rviz DESTINATION share/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_terrain_math test/test_terrain_math.cpp)
  target_include_directories(test_terrain_math PRIVATE include)
  ament_add_gtest(test_leader_follow_math test/test_leader_follow_math.cpp)
  target_include_directories(test_leader_follow_math PRIVATE include)
endif()

ament_package()
