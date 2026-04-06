cmake_minimum_required(VERSION 3.8)
project(vf_robot_bringup)

find_package(ament_cmake REQUIRED)

# ── Install launch files ──
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# ── Install config files ──
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# ── Install rviz config ──
# (already inside config/, but kept explicit for clarity)

# ── ament resource marker ──
install(FILES resource/${PROJECT_NAME}
  DESTINATION share/ament_index/resource_index/packages
)

ament_package()
