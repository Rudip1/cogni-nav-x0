cmake_minimum_required(VERSION 3.8)
project(vf_robot_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O2)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ── Dependencies ─────────────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(Eigen3 REQUIRED)

find_package(PCL QUIET COMPONENTS common search filters)
if(PCL_FOUND)
  add_definitions(-DVFC_HAS_PCL)
  message(STATUS "PCL found — volumetric clearance using PCL voxels")
else()
  message(STATUS "PCL not found — volumetric clearance using 2.5D approximation")
endif()

set(DEPS
  rclcpp
  rclcpp_lifecycle
  nav2_core
  nav2_costmap_2d
  nav2_util
  nav_msgs
  geometry_msgs
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  pluginlib
  visualization_msgs
  rcl_interfaces
)

# ── Source files ──────────────────────────────────────────────────────────────
set(PLUGIN_SOURCES
  # Core plugin
  src/controller.cpp
  src/optimizer.cpp
  src/critic_manager.cpp
  src/parameter_handler.cpp

  # GCF layer
  src/gcf/geometric_complexity_field.cpp
  src/gcf/corridor_analyzer.cpp
  src/gcf/clutter_detector.cpp
  src/gcf/volumetric_clearance.cpp

  # Critics — original 7
  src/critics/obstacle_critic.cpp
  src/critics/volumetric_critic.cpp
  src/critics/dynamic_obstacle_critic.cpp
  src/critics/path_follow_critic.cpp
  src/critics/smoothness_critic.cpp
  src/critics/goal_critic.cpp
  src/critics/velocity_critic.cpp

  # Critics — new 3 (slots 7-9)
  src/critics/corridor_critic.cpp
  src/critics/clearance_critic.cpp
  src/critics/oscillation_critic.cpp

  # Models
  src/models/trajectory.cpp
  src/models/state.cpp

  # Safety shell
  src/safety/hard_safety_shell.cpp
  src/safety/swept_volume_checker.cpp

  # Tools — original 4
  src/tools/noise_generator.cpp
  src/tools/velocity_scheduler.cpp
  src/tools/trajectory_visualizer.cpp
  src/tools/utils.cpp

  # Tools — new 2
  src/tools/weight_adapter.cpp
  src/tools/data_recorder.cpp
)

# ── Build shared library ──────────────────────────────────────────────────────
add_library(${PROJECT_NAME} SHARED ${PLUGIN_SOURCES})

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)

if(PCL_FOUND)
  target_include_directories(${PROJECT_NAME} PUBLIC ${PCL_INCLUDE_DIRS})
  target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES})
endif()

ament_target_dependencies(${PROJECT_NAME} ${DEPS})
target_link_libraries(${PROJECT_NAME} tf2_geometry_msgs::tf2_geometry_msgs)

pluginlib_export_plugin_description_file(nav2_core vf_robot_controller.xml)
pluginlib_export_plugin_description_file(nav2_core critics.xml)

# ── Install — C++ ─────────────────────────────────────────────────────────────
install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/ DESTINATION include/)
install(FILES vf_robot_controller.xml critics.xml DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)

# ── Install — Python ──────────────────────────────────────────────────────────
ament_python_install_package(meta_critic)

install(PROGRAMS
  meta_critic/feature_extractor.py
  meta_critic/inference_node.py
  meta_critic/data_logger.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY meta_critic/models/
  DESTINATION share/${PROJECT_NAME}/meta_critic/models
)

# ── Tests ─────────────────────────────────────────────────────────────────────
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)

  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_gcf test/test_gcf.cpp)
  target_link_libraries(test_gcf ${PROJECT_NAME} tf2_geometry_msgs::tf2_geometry_msgs)
  ament_target_dependencies(test_gcf ${DEPS})

  ament_add_gtest(test_critics test/test_critics.cpp)
  target_link_libraries(test_critics ${PROJECT_NAME} tf2_geometry_msgs::tf2_geometry_msgs)
  ament_target_dependencies(test_critics ${DEPS})

  ament_add_gtest(test_trajectory test/test_trajectory.cpp)
  target_link_libraries(test_trajectory ${PROJECT_NAME} tf2_geometry_msgs::tf2_geometry_msgs)
  ament_target_dependencies(test_trajectory ${DEPS})

  ament_add_gtest(test_safety test/test_safety.cpp)
  target_link_libraries(test_safety ${PROJECT_NAME} tf2_geometry_msgs::tf2_geometry_msgs)
  ament_target_dependencies(test_safety ${DEPS})

  ament_add_gtest(test_optimizer test/test_optimizer.cpp)
  target_link_libraries(test_optimizer ${PROJECT_NAME} tf2_geometry_msgs::tf2_geometry_msgs)
  ament_target_dependencies(test_optimizer ${DEPS})
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${DEPS})
ament_package()
