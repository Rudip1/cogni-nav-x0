find_package(ament_cmake_gtest REQUIRED)
find_package(ament_lint_auto REQUIRED)
ament_lint_auto_find_test_dependencies()

# GCF tests
ament_add_gtest(test_gcf test_gcf.cpp)
target_link_libraries(test_gcf ${PROJECT_NAME})
ament_target_dependencies(test_gcf ${DEPS})

# Trajectory (B-spline + SegmentMap) tests
ament_add_gtest(test_trajectory test_trajectory.cpp)
target_link_libraries(test_trajectory ${PROJECT_NAME})
ament_target_dependencies(test_trajectory ${DEPS})

# Safety shell + swept volume tests
ament_add_gtest(test_safety test_safety.cpp)
target_link_libraries(test_safety ${PROJECT_NAME})
ament_target_dependencies(test_safety ${DEPS})

# Critics + utils tests
ament_add_gtest(test_critics test_critics.cpp)
target_link_libraries(test_critics ${PROJECT_NAME})
ament_target_dependencies(test_critics ${DEPS})

# Optimizer + noise generator tests
ament_add_gtest(test_optimizer test_optimizer.cpp)
target_link_libraries(test_optimizer ${PROJECT_NAME})
ament_target_dependencies(test_optimizer ${DEPS})
