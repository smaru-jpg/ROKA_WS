find_package(ament_cmake_gtest REQUIRED)
find_package(ament_cmake_pytest REQUIRED)

ament_add_gtest(test_plugin test_plugin.cpp)
target_link_libraries(test_plugin
  mujoco_ros2_control
  rclcpp::rclcpp
)

ament_add_gtest(test_headless_init test_headless_init.cpp)
target_link_libraries(test_headless_init
  mujoco_ros2_control
  rclcpp::rclcpp
  hardware_interface::hardware_interface
)
# The isolated Humble installer does not provide a setup script. The hardware
# interface creates services at runtime, so Fast DDS loads the generated
# message type-support library with dlopen instead of through the linker's
# RUNPATH. Give this test the same dependency path installed by our env hook.
set_tests_properties(test_headless_init PROPERTIES
  ENVIRONMENT
    "LD_LIBRARY_PATH=${MUJOCO_ROS2_CONTROL_MSGS_PREFIX}/lib:$ENV{LD_LIBRARY_PATH};AMENT_PREFIX_PATH=${MUJOCO_ROS2_CONTROL_MSGS_PREFIX}:$ENV{AMENT_PREFIX_PATH}"
)

ament_add_pytest_test(test_urdf_to_mujoco_utils test_urdf_to_mujoco_utils.py)
