from setuptools import setup
from glob import glob

# ==============================================================================
# vf_robot_slam — HYBRID ament_cmake + Python package
# ==============================================================================
#
# This setup.py has a LIMITED role in a hybrid package:
#   - Registers the Python module (vf_robot_slam/) → import vf_robot_slam works
#   - Installs flat data files that glob can find at build time
#
# What CMakeLists.txt handles instead:
#   - C++ compilation (src/ — reserved, empty for now)
#   - Python scripts → ros2 run  (CMakeLists macro, not console_scripts here)
#   - config/, rviz/ install with OPTIONAL flag (safe before dirs exist)
#
# What is NOT here:
#   worlds/  → lives in vf_robot_gazebo
#   models/  → lives in vf_robot_gazebo
#   urdf/    → lives in vf_robot_description
#
# Launch file naming convention:
#   slam_launch_<camera>cam.py           (matches existing files in launch/)
#   localization_launch_<camera>cam.py
#   where <camera> = d435i | d455 | multicamera
# ==============================================================================

package_name = "vf_robot_slam"

setup(
    name=package_name,
    version="0.1.0",
    # Top-level Python module only.
    # vf_robot_slam/scripts/ has no __init__.py and is handled by the
    # CMakeLists.txt install_python_scripts() macro — not listed here.
    packages=[package_name],
    data_files=[
        # ── ament index marker ─────────────────────────────────────────────
        # Required for ros2 pkg list and get_package_share_directory() to work.
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        # ── package manifest ───────────────────────────────────────────────
        (f"share/{package_name}", ["package.xml"]),
        # ── Launch files ───────────────────────────────────────────────────
        # Matches both naming styles:
        #   slam_launch_d435icam.py          (existing files)
        #   slam_launch_d435i.launch.py      (new naming convention)
        # glob("launch/*.py") catches everything — no need to list individually.
        (f"share/{package_name}/launch", glob("launch/*.py")),
        # ── RTAB-Map YAML configs ──────────────────────────────────────────
        # config/ may not exist yet — glob returns [] if missing, which is safe.
        # CMakeLists.txt installs config/ with OPTIONAL so no build error either.
        (f"share/{package_name}/config", glob("config/*.yaml")),
        # ── RViz2 layouts ──────────────────────────────────────────────────
        # rviz/ may not exist yet — same safe-glob pattern as config/.
        (f"share/{package_name}/rviz", glob("rviz/*.rviz")),
        # ── Maps and other data files ───────────────────────────────────────
        (f"share/{package_name}/maps", glob("maps/**/*", recursive=True)),
        # ── Maps — one entry per map folder so subdirectory structure is preserved ──
        # CMakeLists.txt handles maps/ in the install tree (OPTIONAL).
        # Add a new line here each time you save a new map with map_saver.launch.py
        (f"share/{package_name}/maps/house1", glob("maps/house1/*")),
        (f"share/{package_name}/maps/my_map", glob("maps/my_map/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Pravin",
    maintainer_email="olipravin18@gmail.com",
    description=(
        "SLAM and Localization launch package for ViroFighter (uvc1) mobile robot. "
        "Supports D435i-only, D455-only, and multicamera RTAB-Map configurations. "
        "Works on real hardware (use_sim_time:=false) and Gazebo (use_sim_time:=true)."
    ),
    license="Apache-2.0",
    extras_require={"test": ["pytest"]},
    entry_points={
        # Python scripts are installed via CMakeLists.txt install_python_scripts macro.
        # Any .py file in vf_robot_slam/scripts/ becomes a ros2 run executable.
        # No entries needed here.
        "console_scripts": [],
    },
)
