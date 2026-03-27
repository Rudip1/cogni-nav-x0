cmake_minimum_required(VERSION 3.8)
project(gazebo_2d_groundtruth_map)

# -----------------------
# C++ Standard
# -----------------------
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# -----------------------
# Find ROS2 / Gazebo dependencies
# -----------------------
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# ROS2
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)

find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(xacro REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(robot_state_publisher REQUIRED)

# Gazebo
find_package(gazebo REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(gazebo_ros_pkgs REQUIRED)

# -----------------------
# Include directories
# -----------------------
include_directories(
  include
  ${gazebo_INCLUDE_DIRS}
)

# -----------------------
# Build C++ Gazebo plugin
# -----------------------
add_library(${PROJECT_NAME} SHARED
  src/groundtruth_map_plugin.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  std_srvs
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  xacro
  kdl_parser
  robot_state_publisher
  gazebo_ros
  gazebo
)

target_link_libraries(${PROJECT_NAME} ${GAZEBO_LIBRARIES})

# -----------------------
# Install C++ library
# -----------------------
install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION lib
)

# -----------------------
# Install headers
# -----------------------
install(DIRECTORY include/
  DESTINATION include
)

# -----------------------
# Install resources / configs / worlds
# -----------------------
install(DIRECTORY
  config
  worlds
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY resource/
  DESTINATION share/${PROJECT_NAME}/resource
)

# ----------------------
# Install Python modules
# ----------------------
ament_python_install_package(${PROJECT_NAME})

# Install all Python executable scripts (without .py extension)
file(GLOB PYTHON_SCRIPTS "gazebo_2d_groundtruth_map/scripts/*.py")
foreach(SCRIPT ${PYTHON_SCRIPTS})
  get_filename_component(SCRIPT_NAME "${SCRIPT}" NAME_WE)  # Get name without extension
  install(PROGRAMS
    "${SCRIPT}"
    DESTINATION lib/${PROJECT_NAME}
    RENAME ${SCRIPT_NAME}
  )
endforeach()

# -----------------------
# Testing / linting (optional)
# -----------------------
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# -----------------------
# Export / finalize
# -----------------------
ament_export_include_directories(include)
ament_export_dependencies(
  rclcpp
  std_srvs
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  xacro
  kdl_parser
  robot_state_publisher
  gazebo_ros
  gazebo
)
ament_package()