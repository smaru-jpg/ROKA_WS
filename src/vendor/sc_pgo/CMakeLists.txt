cmake_minimum_required(VERSION 3.22)
project(sc_pgo)

# Export compile commands for clangd and other tools
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release")
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(OpenMP REQUIRED)
set(_ROKA_GTSAM_PREFIX
  "${CMAKE_CURRENT_SOURCE_DIR}/../../../third_party/gtsam/opt/ros/humble")
if(EXISTS "${_ROKA_GTSAM_PREFIX}/lib/cmake/GTSAM/GTSAMConfig.cmake")
  set(GTSAM_DIR "${_ROKA_GTSAM_PREFIX}/lib/cmake/GTSAM" CACHE PATH "" FORCE)
endif()
find_package(GTSAM REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_msgs REQUIRED)   # usually needed alongside
find_package(std_srvs REQUIRED)


include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIR}
)

add_executable(alaserPGO
  src/laserPosegraphOptimization.cpp
  include/scancontext/Scancontext.cpp
)

# Use ament_target_dependencies for ROS2 packages (handles includes, links, and definitions)
ament_target_dependencies(alaserPGO
  cv_bridge
  geometry_msgs
  nav_msgs
  pcl_conversions
  pcl_msgs
  rclcpp
  sensor_msgs
  std_msgs
  std_srvs
  tf2
  tf2_ros
)

# Link non-ROS2 libraries using plain signature
# NOTE: Plain signature (no PRIVATE/PUBLIC keywords) is required here because
# ament_target_dependencies uses plain signature internally, and CMake prohibits
# mixing plain and keyword signatures on the same target. This is a necessary
# ROS2 ecosystem compatibility pattern, not a violation of modern CMake practices.
target_compile_options(alaserPGO
  PRIVATE ${OpenMP_CXX_FLAGS}
)
target_link_libraries(alaserPGO
  ${PCL_LIBRARIES}
  OpenMP::OpenMP_CXX
  gtsam
)
set_target_properties(alaserPGO PROPERTIES INSTALL_RPATH "$ORIGIN/..")

install(TARGETS
  alaserPGO
  DESTINATION lib/${PROJECT_NAME}
)

if(EXISTS "${_ROKA_GTSAM_PREFIX}/lib/x86_64-linux-gnu")
  file(GLOB _ROKA_GTSAM_RUNTIME_LIBS
    "${_ROKA_GTSAM_PREFIX}/lib/x86_64-linux-gnu/libgtsam.so*"
    "${_ROKA_GTSAM_PREFIX}/lib/x86_64-linux-gnu/libmetis-gtsam.so*")
  install(FILES ${_ROKA_GTSAM_RUNTIME_LIBS} DESTINATION lib)
endif()

ament_environment_hooks(
  "${CMAKE_CURRENT_SOURCE_DIR}/env/sc_pgo_library_path.dsv.in"
)

install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch
  FILES_MATCHING PATTERN "*.launch.py"
)

install(
  DIRECTORY rviz_cfg
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  # This is vendored third-party code. Portable acceptance verifies the C++
  # loop-constraint behavior below; host-dependent development linters are not
  # registered because globally installed Flake8 plugins can change their
  # result without changing the deployed SC-PGO binary.
  ament_add_gtest(test_loop_constraint test/test_loop_constraint.cpp)
  target_include_directories(test_loop_constraint PRIVATE include ${GTSAM_INCLUDE_DIR})
  target_link_libraries(test_loop_constraint gtsam)
endif()

ament_package()
