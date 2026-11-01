cmake_minimum_required(VERSION 3.8)
project(roka_map_localization)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters io registration)
find_package(pcl_conversions REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(fixed_map_localizer src/fixed_map_localizer.cpp)
target_compile_features(fixed_map_localizer PUBLIC cxx_std_17)
target_include_directories(fixed_map_localizer PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(fixed_map_localizer ${PCL_LIBRARIES})
ament_target_dependencies(fixed_map_localizer
  geometry_msgs nav_msgs pcl_conversions rclcpp sensor_msgs std_msgs std_srvs
  tf2 tf2_eigen tf2_ros)

add_executable(pointcloud_sector_filter src/pointcloud_sector_filter.cpp)
target_compile_features(pointcloud_sector_filter PUBLIC cxx_std_17)
target_include_directories(pointcloud_sector_filter PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(pointcloud_sector_filter ${PCL_LIBRARIES})
ament_target_dependencies(pointcloud_sector_filter pcl_conversions rclcpp sensor_msgs)

add_executable(map_cleanup_editor src/map_cleanup_editor.cpp)
target_compile_features(map_cleanup_editor PUBLIC cxx_std_17)
target_include_directories(map_cleanup_editor PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(map_cleanup_editor ${PCL_LIBRARIES})
ament_target_dependencies(map_cleanup_editor
  geometry_msgs pcl_conversions rclcpp sensor_msgs std_srvs)

install(TARGETS
  fixed_map_localizer pointcloud_sector_filter map_cleanup_editor
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch rviz DESTINATION share/${PROJECT_NAME})

ament_package()
