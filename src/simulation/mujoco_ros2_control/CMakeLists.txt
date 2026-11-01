cmake_minimum_required(VERSION 3.16)

project(mujoco_ros2_control)

# The installer used on this machine keeps the upstream MuJoCo ROS packages in
# an isolated prefix without a setup script.  Discover that prefix during a
# normal workspace build while still preferring any prefix supplied by the
# caller or by a future apt package.
set(ROKA_MUJOCO_ROS_PREFIX
  "$ENV{HOME}/.local/ros-humble-mujoco/opt/ros/humble"
)
if(EXISTS
    "${ROKA_MUJOCO_ROS_PREFIX}/share/ros2_control_cmake/cmake/ros2_control_cmakeConfig.cmake")
  list(APPEND CMAKE_PREFIX_PATH "${ROKA_MUJOCO_ROS_PREFIX}")
endif()

find_package(ros2_control_cmake REQUIRED)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(controller_manager REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(fmt REQUIRED)
find_package(glfw3 REQUIRED)
find_package(control_toolbox REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(mujoco_ros2_control_msgs REQUIRED)
find_package(mujoco_ros2_control_plugins REQUIRED)
find_package(Threads REQUIRED)
find_package(transmission_interface REQUIRED)
find_package(backward_ros REQUIRED)
find_package(tinyxml2_vendor QUIET)
find_package(TinyXML2 REQUIRED)

# This workspace can use the upstream MuJoCo ROS packages from either apt or
# the isolated prefix produced by the repository's installer. Preserve the
# resolved message-package library directory in the plugin RPATH so pluginlib
# and the package's own tests do not depend on an interactive shell having
# sourced that secondary prefix.
get_filename_component(
  MUJOCO_ROS2_CONTROL_MSGS_PREFIX
  "${mujoco_ros2_control_msgs_DIR}/../../.."
  ABSOLUTE
)

# Link MuJoCo via the vendor package
find_package(mujoco_vendor REQUIRED)
set(MUJOCO_LIB mujoco::mujoco)

# The vendor package may be installed in different locations depending on if it
# was installed with apt vs pixi. So resolve the install root and use that to
# locate the relevant header files for this project.
# TODO: It would easier if the vendor package exported the variables that we need here.
get_filename_component(MUJOCO_PREFIX "${mujoco_vendor_DIR}/../../.." ABSOLUTE)
set(MUJOCO_ROOT "${MUJOCO_PREFIX}/opt/mujoco_vendor")
if(NOT EXISTS "${MUJOCO_ROOT}/include/mujoco/mujoco.h")
  set(MUJOCO_ROOT "${MUJOCO_PREFIX}")
endif()
set(MUJOCO_INCLUDE_DIR "${MUJOCO_ROOT}/include")
set(MUJOCO_SIMULATE_DIR "${MUJOCO_ROOT}/simulate")
if(NOT EXISTS "${MUJOCO_SIMULATE_DIR}/simulate.h")
  set(MUJOCO_SIMULATE_DIR "${MUJOCO_ROOT}/include/simulate")
endif()
# So downstream packages that include our headers can find glfw_adapter.h etc.
set(MUJOCO_INSTALL_INCLUDE_DIR "${MUJOCO_INCLUDE_DIR}")
set(MUJOCO_INSTALL_SIMULATE_DIR "${MUJOCO_SIMULATE_DIR}")

# Use an in-tree fallback so this patched Humble package can be rebuilt on the
# isolated roka workstation without downloading during a colcon build.
find_package(lodepng QUIET)
if(NOT TARGET lodepng)
  set(LODEPNG_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/third_party/lodepng")
  add_library(lodepng STATIC
    "${LODEPNG_SOURCE_DIR}/lodepng.h"
    "${LODEPNG_SOURCE_DIR}/lodepng.cpp"
  )
  set_target_properties(lodepng PROPERTIES POSITION_INDEPENDENT_CODE ON)
  target_compile_options(lodepng PRIVATE ${MUJOCO_MACOS_COMPILE_OPTIONS})
  target_link_options(lodepng PRIVATE ${MUJOCO_MACOS_LINK_OPTIONS})
  target_include_directories(lodepng PUBLIC "${LODEPNG_SOURCE_DIR}")
  install(TARGETS lodepng
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
  )
endif()

# We don't care about warnings from the external sources
set(MUJOCO_SILENCE_COMPILER_WARNINGS
  -Wno-missing-field-initializers
  -Wno-unused-parameter
  -Wno-sign-compare
  -Wno-psabi
)

# Check if MuJoCo's simulate application is available for linking. The github tarfiles packaged
# by the vendor package has the simulate application compiled with -flto=auto, so we cannot
# actually link against it as normal symbols are not available. If that is the case, we must
# compile the application from source. For the conda installed version, we can link directly
# against the package. We create a `mujoco_simulate` library in both cases.
find_library(mujoco_simulate_lib
  NAMES simulate
  PATHS "${MUJOCO_ROOT}/lib"
  NO_DEFAULT_PATH
)
if(mujoco_simulate_lib AND NOT EXISTS "${MUJOCO_SIMULATE_DIR}/simulate.cc")
  # Conda installed package does not have source, so link against the library directly
  message(STATUS "Linking against the mujoco simulate application directly")
  add_library(mujoco_simulate STATIC IMPORTED)
  set_target_properties(mujoco_simulate PROPERTIES
    IMPORTED_LOCATION "${mujoco_simulate_lib}"
  )
  target_include_directories(mujoco_simulate INTERFACE ${MUJOCO_SIMULATE_DIR} ${MUJOCO_INCLUDE_DIR})
  target_link_libraries(mujoco_simulate INTERFACE ${MUJOCO_LIB} glfw)
else()
  message(STATUS "No linkable simulate application available, compiling from source")
  # Generate a small local patch of MuJoCo's GLFW adapter:
  # - disable the hard-coded 4x MSAA window buffer (the scene already controls
  #   offscreen sampling);
  # - reacquire the GL context before destruction, because this integration
  #   constructs the viewer on its UI thread and destroys it on the controller
  #   thread. PRIME offload otherwise crashes in libGLdispatch on shutdown.
  file(READ "${MUJOCO_SIMULATE_DIR}/glfw_adapter.cc" MUJOCO_GLFW_ADAPTER_SOURCE)
  string(FIND "${MUJOCO_GLFW_ADAPTER_SOURCE}"
    "Glfw().glfwWindowHint(GLFW_SAMPLES, 4);"
    MUJOCO_GLFW_SAMPLES_LOCATION
  )
  string(FIND "${MUJOCO_GLFW_ADAPTER_SOURCE}"
    "GlfwAdapter::~GlfwAdapter() {\n  FreeMjrContext();"
    MUJOCO_GLFW_DESTRUCTOR_LOCATION
  )
  if(MUJOCO_GLFW_SAMPLES_LOCATION EQUAL -1 OR MUJOCO_GLFW_DESTRUCTOR_LOCATION EQUAL -1)
    message(FATAL_ERROR "The MuJoCo GLFW adapter changed; refresh the local performance patch.")
  endif()
  string(REPLACE
    "Glfw().glfwWindowHint(GLFW_SAMPLES, 4);"
    "Glfw().glfwWindowHint(GLFW_SAMPLES, 0);"
    MUJOCO_GLFW_ADAPTER_SOURCE
    "${MUJOCO_GLFW_ADAPTER_SOURCE}"
  )
  string(REPLACE
    "GlfwAdapter::~GlfwAdapter() {\n  FreeMjrContext();"
    "GlfwAdapter::~GlfwAdapter() {\n  Glfw().glfwMakeContextCurrent(window_);\n  FreeMjrContext();"
    MUJOCO_GLFW_ADAPTER_SOURCE
    "${MUJOCO_GLFW_ADAPTER_SOURCE}"
  )
  set(PATCHED_GLFW_ADAPTER "${CMAKE_CURRENT_BINARY_DIR}/glfw_adapter_patched.cc")
  file(WRITE "${PATCHED_GLFW_ADAPTER}" "${MUJOCO_GLFW_ADAPTER_SOURCE}")

  add_library(platform_ui_adapter OBJECT)
  set_target_properties(platform_ui_adapter PROPERTIES
  POSITION_INDEPENDENT_CODE ON
  )
  target_sources(
    platform_ui_adapter
    PRIVATE ${MUJOCO_SIMULATE_DIR}/glfw_adapter.h ${MUJOCO_SIMULATE_DIR}/glfw_dispatch.h ${MUJOCO_SIMULATE_DIR}/platform_ui_adapter.h
    PRIVATE ${PATCHED_GLFW_ADAPTER} ${MUJOCO_SIMULATE_DIR}/glfw_dispatch.cc ${MUJOCO_SIMULATE_DIR}/platform_ui_adapter.cc
  )
  target_include_directories(platform_ui_adapter PUBLIC ${MUJOCO_SIMULATE_DIR} ${MUJOCO_INCLUDE_DIR})
  target_link_libraries(platform_ui_adapter PRIVATE ${MUJOCO_LIB} glfw)
  target_compile_options(platform_ui_adapter PRIVATE ${MUJOCO_SILENCE_COMPILER_WARNINGS})
  add_library(mujoco::platform_ui_adapter ALIAS platform_ui_adapter)

  add_library(mujoco_simulate STATIC $<TARGET_OBJECTS:platform_ui_adapter>)
  set_target_properties(mujoco_simulate PROPERTIES
    OUTPUT_NAME simulate
    POSITION_INDEPENDENT_CODE ON
  )
  target_sources(mujoco_simulate
    PRIVATE ${MUJOCO_SIMULATE_DIR}/simulate.h
    PRIVATE ${MUJOCO_SIMULATE_DIR}/simulate.cc ${MUJOCO_SIMULATE_DIR}/array_safety.h
  )
  target_include_directories(mujoco_simulate PUBLIC ${MUJOCO_SIMULATE_DIR} ${MUJOCO_INCLUDE_DIR})
  target_link_libraries(mujoco_simulate PRIVATE lodepng mujoco::platform_ui_adapter ${MUJOCO_LIB})
  target_compile_options(mujoco_simulate PRIVATE ${MUJOCO_SILENCE_COMPILER_WARNINGS})

  # We set these after creating the targets to avoid propagating them to other third-party targets
  set_compiler_options()
  export_windows_symbols()
  install(TARGETS mujoco_simulate
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
  )
endif()

# MuJoCo ROS 2 control system interface (this package)
add_library(mujoco_ros2_control SHARED
    src/mujoco_system_interface.cpp
    src/mujoco_cameras.cpp
    src/mujoco_lidar.cpp
)
# Ensure MuJoCo library can be found at runtime regardless of how it was installed above.
# mujoco_vendor installs libmujoco.so under opt/mujoco_vendor/lib/ relative to the
# install prefix, so that directory must be in the RPATH alongside the standard lib/.
set_target_properties(mujoco_ros2_control PROPERTIES
  BUILD_RPATH "${MUJOCO_ROS2_CONTROL_MSGS_PREFIX}/lib"
  INSTALL_RPATH "\$ORIGIN/../lib;\$ORIGIN/../opt/mujoco_vendor/lib;${CMAKE_INSTALL_PREFIX}/lib;${MUJOCO_ROS2_CONTROL_MSGS_PREFIX}/lib"
)
# MuJoCo has TinyXML2 statically linked, which conflicts with the system TinyXML2
# used by FastDDS. We need to ensure the system TinyXML2 is loaded first.
# Force tinyxml2 to appear first in the DT_NEEDED entries by using LINK_OPTIONS.
# TODO: Remove this after the following is resolved and the next ROS sync
# https://github.com/pal-robotics/mujoco_vendor/issues/2
target_link_options(mujoco_ros2_control PUBLIC
  "LINKER:--push-state,--no-as-needed"
  "$<TARGET_LINKER_FILE:tinyxml2::tinyxml2>"
  "LINKER:--pop-state"
)
target_link_libraries(mujoco_ros2_control
PUBLIC
  hardware_interface::hardware_interface
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  ${nav_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${mujoco_ros2_control_msgs_TARGETS}
  pluginlib::pluginlib
  fmt::fmt
  ${MUJOCO_LIB}
  control_toolbox::control_toolbox
  Eigen3::Eigen
PRIVATE
  mujoco_simulate
  Threads::Threads
  lodepng
  glfw
)
# Note: the transmissions_interface must NOT be statically linked against the library, only headers
# should be used. Otherwise it will be loaded by the linker at process startup and its
# PLUGINLIB_EXPORT_CLASS static constructors will have run before the ClassLoaders exist. This
# can cause problems when pluginlib calls dlopen, as a previously-loaded handle will be
# returned that will prevent registration of a factory for the transmission type.
target_include_directories(mujoco_ros2_control PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  $<INSTALL_INTERFACE:${MUJOCO_INSTALL_INCLUDE_DIR}>
  $<INSTALL_INTERFACE:${MUJOCO_INSTALL_SIMULATE_DIR}>
  $<TARGET_PROPERTY:transmission_interface::transmission_interface,INTERFACE_INCLUDE_DIRECTORIES>
  $<TARGET_PROPERTY:mujoco_ros2_control_plugins::mujoco_ros2_control_plugins,INTERFACE_INCLUDE_DIRECTORIES>
)
# Mark MuJoCo source as system to avoid compiler warnings
target_include_directories(mujoco_ros2_control SYSTEM PUBLIC
  $<BUILD_INTERFACE:${MUJOCO_INCLUDE_DIR}>
  $<BUILD_INTERFACE:${MUJOCO_SIMULATE_DIR}>
)

# Note: mujoco::mujoco is an IMPORTED target owned by mujoco_vendor and cannot be
# co-installed here. Downstream packages must find_package(mujoco_vendor) directly,
# which is ensured via ament_export_dependencies below.
install(TARGETS mujoco_ros2_control
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/ DESTINATION include)

# Install the ROS 2 control node
add_executable(ros2_control_node src/mujoco_ros2_control_node.cpp)
target_link_libraries(ros2_control_node PUBLIC
  controller_manager::controller_manager
)
install(
  TARGETS ros2_control_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install python tools
install(PROGRAMS
  scripts/find_missing_inertias.py
  scripts/make_mjcf_from_robot_description.py
  scripts/robot_description_to_mjcf.sh
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY resources scripts
  DESTINATION share/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS
)

install(
  FILES scripts/requirements.txt
  DESTINATION share/${PROJECT_NAME}
)

pluginlib_export_plugin_description_file(hardware_interface mujoco_system_interface_plugin.xml)

if(BUILD_TESTING)
  add_subdirectory(tests)
endif()

ament_environment_hooks(
  "${CMAKE_CURRENT_SOURCE_DIR}/env/mujoco_dependency_library_path.dsv.in"
)

ament_python_install_package(mujoco_ros2_control
  SCRIPTS_DESTINATION lib/mujoco_ros2_control
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  hardware_interface
  transmission_interface
  rclcpp
  rclcpp_lifecycle
  nav_msgs
  sensor_msgs
  mujoco_ros2_control_msgs
  mujoco_ros2_control_plugins
  pluginlib
  control_toolbox
  Eigen3
  mujoco_vendor
)
ament_package()
