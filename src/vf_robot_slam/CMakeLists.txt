cmake_minimum_required(VERSION 3.8)
project(vf_robot_slam)

# ==============================================================================
# HYBRID PACKAGE — ament_cmake + Python launch files
#
# Structure:
#   src/                    C++ nodes (empty for now — reserved for future use)
#   include/vf_robot_slam/  C++ headers (empty for now)
#   vf_robot_slam/          Python module → import vf_robot_slam
#     scripts/              Python executables → ros2 run vf_robot_slam <name>
#   launch/                 Python launch files → ros2 launch vf_robot_slam <file>
#   config/                 RTAB-Map YAML parameter files (per camera config)
#   rviz/                   RViz2 layout files
#   docs/                   Package documentation and GIFs
#
# What is NOT here (single source of truth rule):
#   urdf/   → lives in vf_robot_description
#   worlds/ → lives in vf_robot_gazebo
#   models/ → lives in vf_robot_gazebo
#
# SLAM stack: RTAB-Map (rtabmap_ros) — installed system-wide via apt.
#             Do NOT clone rtabmap into this workspace.
#             Install: sudo apt install ros-humble-rtabmap-ros
#
# Cameras:
#   D435i — /camera/*   namespace, front-top, 60° ground-facing
#   D455  — /camera2/*  namespace, rear base-level, wide FOV
# ==============================================================================

# ------------------------------------------------------------------------------
# C++ Standard
# ------------------------------------------------------------------------------
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ------------------------------------------------------------------------------
# Dependencies
# ------------------------------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)   # hybrid C++/Python support

# Core ROS 2 — needed for any future C++ nodes in src/
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# Robot description — URDF source of truth
find_package(vf_robot_description REQUIRED)

# NOTE: No gazebo, no vf_robot_messages, no ultrasound here.
#       This package is SLAM + Localization only.
#       Gazebo deps live in vf_robot_gazebo.

# ------------------------------------------------------------------------------
# C++ Executables
# ------------------------------------------------------------------------------
# No C++ nodes yet — src/ is reserved for future SLAM utilities.
# When you add a node, follow this pattern:
#
#   add_executable(my_node src/my_node.cpp)
#   ament_target_dependencies(my_node rclcpp sensor_msgs ...)
#   target_include_directories(my_node PUBLIC
#     $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#     $<INSTALL_INTERFACE:include>
#   )
#   install(TARGETS my_node DESTINATION lib/${PROJECT_NAME})

# ------------------------------------------------------------------------------
# Python Scripts Macro
# ------------------------------------------------------------------------------
# Installs every .py in a folder as a ros2 run executable (strips .py extension).
# No __init__.py needed inside script folders — just drop .py files in.
# Usage: install_python_scripts("vf_robot_slam/scripts")
macro(install_python_scripts FOLDER)
  file(GLOB _SCRIPTS "${FOLDER}/*.py")
  foreach(_SCRIPT ${_SCRIPTS})
    get_filename_component(_NAME ${_SCRIPT} NAME_WE)
    install(PROGRAMS ${_SCRIPT}
      DESTINATION lib/${PROJECT_NAME}
      RENAME ${_NAME}
    )
  endforeach()
endmacro()

# Register script folders — one line per folder
install_python_scripts("vf_robot_slam/scripts")
# install_python_scripts("vf_robot_slam/tools")    # uncomment when ready
# install_python_scripts("vf_robot_slam/utils")    # uncomment when ready

# ------------------------------------------------------------------------------
# Install
# ------------------------------------------------------------------------------

# C++ headers — install even when empty so include/ path is exported correctly
install(DIRECTORY include/
  DESTINATION include
)

# Python module → import vf_robot_slam
ament_python_install_package(${PROJECT_NAME})

# Data assets
# NOTE: install(DIRECTORY) silently skips missing directories — safe to list
#       config/ and rviz/ even before they have files in them.
install(DIRECTORY
  launch    # Python launch files
  DESTINATION share/${PROJECT_NAME}
)

# config/ and rviz/ are installed separately so each has a clean destination.
# These directories may not exist yet — CMake will skip them without error.
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL                          # skip silently if config/ doesn't exist yet
)

install(DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL                          # skip silently if rviz/ doesn't exist yet
)

install(DIRECTORY maps
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL                         # skip silently if maps/ doesn't exist yet
)

# ------------------------------------------------------------------------------
# Exports
# ------------------------------------------------------------------------------
ament_export_include_directories(include)
ament_export_dependencies(
  rclcpp rclpy
  std_msgs sensor_msgs geometry_msgs nav_msgs
  tf2 tf2_ros
  vf_robot_description
)

ament_package()