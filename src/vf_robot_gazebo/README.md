# 🤖 `vf_robot_gazebo`

> **ViroFighter UVC-1 Robot — Gazebo Simulation Package**
> ROS 2 Humble · Hybrid C++ / Python · Gazebo Classic 11

[![ROS2 Humble](https://img.shields.io/badge/ROS2-Humble-blue?style=flat-square&logo=ros)](https://docs.ros.org/en/humble/)
[![Gazebo Classic](https://img.shields.io/badge/Gazebo-Classic%2011-orange?style=flat-square)](http://gazebosim.org/)
[![License](https://img.shields.io/badge/License-Apache%202.0-green?style=flat-square)](LICENSE)
[![Build](https://img.shields.io/badge/Build-ament__cmake%20%2B%20ament__cmake__python-purple?style=flat-square)]()
[![C++](https://img.shields.io/badge/C%2B%2B-17-blue?style=flat-square)]()
[![Python](https://img.shields.io/badge/Python-3.10-yellow?style=flat-square)]()

---

## 📋 Table of Contents

- [Overview](#-overview)
- [Package Structure](#-package-structure)
- [Why Hybrid C++ + Python](#-why-hybrid-c--python)
- [URDF vs SDF Spawning](#-urdf-vs-sdf-spawning)
- [URDF from Another Package](#-urdf-from-another-package)
- [Worlds](#-worlds)
- [Launch Files](#-launch-files)
- [Nodes & Scripts](#-nodes--scripts)
- [Header Include Path Fix](#-header-include-path-fix)
- [Environment Setup](#-environment-setup)
- [Build & Run](#-build--run)
- [All Commands Reference](#-all-commands-reference)
- [Troubleshooting](#-troubleshooting)

---

## 🌟 Overview

`vf_robot_gazebo` is the **standalone Gazebo simulation package** for the ViroFighter UVC-1 robot. It contains worlds, models, launch files, and sensor nodes. It does **not** contain the URDF or meshes — those live in `vf_robot_description` (single source of truth).

This package is a **hybrid `ament_cmake` + Python** package, meaning it compiles C++ executables and installs Python scripts from the same `CMakeLists.txt`. The build system is `ament_cmake` (not `ament_python`), with `ament_cmake_python` bridging Python support.

Key responsibilities:
- 🌍 Gazebo worlds (hospital, corridors, my_world, empty, etc.)
- 🏗️ Gazebo models (robot SDF, environment models, meshes)
- 🚀 Launch files for every world × (URDF | SDF) combination
- 🔊 C++ ultrasound aggregator node (`ultrasound_cpp`)
- 🐍 Python ultrasound aggregator node (`ultrasound_py`)
- 🎮 Keyboard teleop script

---
![ViroFighter Gazebo Demo](docs/vf_robot_gazebo.gif)
## 📁 Package Structure

```
vf_robot_gazebo/
├── CMakeLists.txt                     # Hybrid ament_cmake + Python build
├── package.xml                        # Declares ament_cmake build type
├── setup.py                           # Python module install only (no console_scripts)
├── docs/
│   └── vf_robot_gazebo.gif            # Demo animation
├── include/
│   └── vf_robot_gazebo/
│       └── ultrasound.h               # C++ node header
├── src/
│   └── ultrasound.cpp                 # C++ ultrasound aggregator
├── vf_robot_gazebo/                   # Python package (importable)
│   ├── __init__.py
│   └── scripts/
│       ├── teleop_twist_keyboard.py   # Keyboard teleop
│       └── ultrasound_py.py           # Python ultrasound aggregator
├── launch/
│   ├── vf_robot_state_publisher.launch.py
│   ├── vf_spawn_urdf.launch.py
│   ├── vf_spawn_sdf.launch.py
│   ├── vf_empty_world_urdf.launch.py
│   ├── vf_empty_world_sdf.launch.py
│   ├── vf_my_world_urdf.launch.py
│   ├── vf_my_world_sdf.launch.py
│   ├── vf_hospital_world_urdf.launch.py
│   └── vf_hospital_world_sdf.launch.py
├── models/
│   ├── uvc1_virofighter/              # Robot Gazebo model (SDF)
│   │   ├── model.config
│   │   └── model.sdf
│   ├── uvc1_common/                   # Shared meshes (.dae / .stl)
│   │   ├── model.config
│   │   └── meshes/
│   │       ├── bases/
│   │       ├── sensors/
│   │       └── wheels/
│   ├── uvc1_hospital/
│   ├── uvc1_corridors2/
│   ├── uvc1_corridors3/
│   ├── uvc1_corridors4/
│   ├── uvc1_corridors5/
│   ├── uvc1_simple_corridor/
│   └── uvc1_big_rect/
├── rviz/
│   └── vf_robot_gazebo.rviz           # Full nav2+costmap RViz config
├── worlds/
│   ├── empty_world.world
│   ├── my_world.world
│   ├── hospital.world
│   ├── hospital_moving.world
│   ├── aisle.world
│   ├── big_rect.world
│   ├── classroom.world
│   ├── corridor.world
│   ├── corridor3.world
│   ├── corridor4.world
│   ├── corridor5.world
│   ├── corridors2.world
│   └── sote_1.world
└── resource/
    └── vf_robot_gazebo                # ament index marker
```

---

## 🔀 Why Hybrid C++ + Python

This package compiles a C++ node **and** installs Python scripts from the same package. This is done with `ament_cmake` as the build type — not `ament_python`.

### The key decisions in `CMakeLists.txt`

**1. `ament_cmake_python` bridges Python into a cmake build:**
```cmake
find_package(ament_cmake_python REQUIRED)
ament_python_install_package(${PROJECT_NAME})   # installs vf_robot_gazebo/ as importable module
```

**2. A macro auto-installs every `.py` in a folder as a `ros2 run` executable:**
```cmake
macro(install_python_scripts FOLDER)
  file(GLOB _SCRIPTS "${FOLDER}/*.py")
  foreach(_SCRIPT ${_SCRIPTS})
    get_filename_component(_NAME ${_SCRIPT} NAME_WE)
    install(PROGRAMS ${_SCRIPT}
      DESTINATION lib/${PROJECT_NAME}
      RENAME ${_NAME}          # strips .py extension
    )
  endforeach()
endmacro()

install_python_scripts("vf_robot_gazebo/scripts")
```

Drop a `.py` file into `vf_robot_gazebo/scripts/` → rebuild → it becomes a `ros2 run vf_robot_gazebo <name>` command automatically. No `console_scripts` entry in `setup.py` required.

**3. `setup.py` role is intentionally limited:**

`setup.py` only registers the `vf_robot_gazebo/` Python module so it is importable via `import vf_robot_gazebo`. It does **not** list `console_scripts` — that is handled entirely by the cmake macro above. This is the correct pattern for `ament_cmake` hybrid packages.

**4. C++ executable:**
```cmake
add_executable(ultrasound_cpp src/ultrasound.cpp)
ament_target_dependencies(ultrasound_cpp ${COMMON_DEPENDENCIES})
install(TARGETS ultrasound_cpp DESTINATION lib/${PROJECT_NAME})
```

Both `ultrasound_cpp` and `ultrasound_py` end up in `install/lib/vf_robot_gazebo/` and are callable via `ros2 run`. They do the same thing — aggregate individual `sensor_msgs/Range` ultrasound topics and republish as a single `vf_robot_messages/UltraSound` message on `/esp/range`. The C++ version is production, the Python version is useful for rapid debugging.

---

## 🤔 URDF vs SDF Spawning

Every world has two launch variants: `_urdf` and `_sdf`. Understanding the difference matters.

### URDF spawn (`vf_spawn_urdf.launch.py`)

```
robot_state_publisher  →  reads uvc1_virofighter.urdf from vf_robot_description
        ↓ publishes /robot_description topic
spawn_entity.py  →  -topic robot_description  →  Gazebo
```

- Requires `vf_robot_state_publisher.launch.py` to run first (or be included)
- URDF is processed with `xacro`, then published on `/robot_description`
- `spawn_entity.py` reads from the topic, converts internally to SDF, and spawns
- `/robot_description` stays alive — RViz `RobotModel` display works
- Joint states from Gazebo plugin → `robot_state_publisher` → TF tree ✅

### SDF spawn (`vf_spawn_sdf.launch.py`)

```
spawn_entity.py  →  -file models/uvc1_virofighter/model.sdf  →  Gazebo directly
```

- Does **not** require `robot_state_publisher` to be running
- Reads the pre-built `model.sdf` directly from this package's `models/` folder
- Gazebo joint state plugin still publishes `/joint_states`
- But `/robot_description` is not published — RViz `RobotModel` display will be blank
- Faster startup, useful for pure simulation without RViz robot model

### When Gazebo strips plugins

`gz sdf -p` (the URDF→SDF converter) strips all `<gazebo>` plugin tags and breaks `model://` mesh URIs. The `urdf_to_sdf.sh` script in `vf_robot_description` fixes both and writes output to `models/uvc1_virofighter/model.sdf` in this package. **Never use raw gz sdf output directly.**

### Choosing between them

| Use case | Launch variant |
|---|---|
| Full simulation with RViz robot model | `_urdf` |
| Quick Gazebo-only test | `_sdf` |
| SLAM / Nav2 (needs `/robot_description`) | `_urdf` |
| Multi-robot (avoids topic conflicts) | `_sdf` |

---

## 🔗 URDF from Another Package

The URDF lives in `vf_robot_description`, not here. This is intentional — one source of truth for the robot model shared between real robot bringup and simulation.

### How it's linked

`package.xml` declares a build and runtime dependency:
```xml
<depend>vf_robot_description</depend>
```

`CMakeLists.txt` finds it:
```cmake
find_package(vf_robot_description REQUIRED)
```

`vf_robot_state_publisher.launch.py` resolves the path at runtime:
```python
from ament_index_python.packages import get_package_share_directory

urdf_path = os.path.join(
    get_package_share_directory("vf_robot_description"),
    "urdf",
    "uvc1_virofighter.urdf",
)
```

`colcon` build order is enforced by the `<depend>` tag — `vf_robot_description` is always built before `vf_robot_gazebo`.

### Mesh URIs

The URDF uses `package://vf_robot_description/meshes/...` URIs. RViz resolves these via the ament package index. Gazebo resolves `model://uvc1_common/meshes/...` via the `GAZEBO_MODEL_PATH` set in `package.xml`:

```xml
<export>
  <gazebo_ros gazebo_model_path="${prefix}/share/${name}/models"/>
</export>
```

This makes Gazebo search `install/share/vf_robot_gazebo/models/` for model references.

---

## 🌍 Worlds

| World file | Description |
|---|---|
| `empty_world.world` | Flat ground plane, minimal |
| `my_world.world` | Office-style layout with walls, tables, shelves, console |
| `hospital.world` | Static hospital environment with `uvc1_hospital` model |
| `hospital_moving.world` | Hospital with dynamic obstacle actors |
| `aisle.world` | Narrow aisle environment |
| `big_rect.world` | Large rectangular room |
| `classroom.world` | Classroom layout |
| `corridor.world` / `corridor3-5.world` | Corridor variants of increasing complexity |
| `corridors2.world` | Multi-segment corridor maze |
| `sote_1.world` | Custom environment |

Default spawn pose in `vf_my_world_urdf.launch.py`: `x=9.0, y=0.5, θ=3.14` (robot faces left into the environment).

---

## 🚀 Launch Files

### `vf_robot_state_publisher.launch.py`

Reads the URDF from `vf_robot_description` and publishes `/robot_description`. This is a dependency of all `_urdf` world launches. You rarely need to run this standalone.

```bash
ros2 launch vf_robot_gazebo vf_robot_state_publisher.launch.py
ros2 launch vf_robot_gazebo vf_robot_state_publisher.launch.py use_sim_time:=false
```

### `vf_spawn_urdf.launch.py`

Spawns the robot by reading `/robot_description` topic. Requires state publisher already running.

```bash
ros2 launch vf_robot_gazebo vf_spawn_urdf.launch.py
ros2 launch vf_robot_gazebo vf_spawn_urdf.launch.py x_pose:=2.0 y_pose:=1.0 theta:=1.57
```

### `vf_spawn_sdf.launch.py`

Spawns the robot directly from `models/uvc1_virofighter/model.sdf`. No state publisher needed.

```bash
ros2 launch vf_robot_gazebo vf_spawn_sdf.launch.py
ros2 launch vf_robot_gazebo vf_spawn_sdf.launch.py x_pose:=0.0 y_pose:=0.0 theta:=0.0
```

### World launches (URDF variants)

Each includes: gzserver + gzclient + robot_state_publisher + spawn_urdf + rqt_robot_steering + rviz2.

```bash
ros2 launch vf_robot_gazebo vf_empty_world_urdf.launch.py
ros2 launch vf_robot_gazebo vf_my_world_urdf.launch.py
ros2 launch vf_robot_gazebo vf_hospital_world_urdf.launch.py
```

### World launches (SDF variants)

Each includes: gzserver + gzclient + spawn_sdf. No RViz, no state publisher.

```bash
ros2 launch vf_robot_gazebo vf_empty_world_sdf.launch.py
ros2 launch vf_robot_gazebo vf_my_world_sdf.launch.py
ros2 launch vf_robot_gazebo vf_hospital_world_sdf.launch.py
```

### Launch arguments (all world launches)

| Argument | Default | Description |
|---|---|---|
| `use_sim_time` | `true` | Use Gazebo clock |
| `x_pose` | `9.0` (my_world) / `0.0` | Spawn X |
| `y_pose` | `0.5` (my_world) / `0.0` | Spawn Y |
| `theta` | `3.14` (my_world) / `0.0` | Spawn yaw (radians) |

---

## 🔧 Nodes & Scripts

### `ultrasound_cpp` — C++ aggregator

Subscribes to 5 individual ultrasonic `sensor_msgs/Range` topics and republishes as `vf_robot_messages/UltraSound` on `/esp/range`. The `code` field maps frame_id to sensor index (0–4).

```bash
ros2 run vf_robot_gazebo ultrasound_cpp
```

**Subscribed topics:**

| Topic | Sensor code |
|---|---|
| `/ultrasound/front_left` | `0` |
| `/ultrasound/front_right` | `1` |
| `/ultrasound/right` | `2` |
| `/ultrasound/rear` | `3` |
| `/ultrasound/left` | `4` |

**Published topic:** `/esp/range` (`vf_robot_messages/UltraSound`)

### `ultrasound_py` — Python aggregator

Identical functionality to `ultrasound_cpp`, implemented in Python. Uses `frame_id` dict lookup instead of `std::map`. Useful for debugging the pipeline without recompiling.

```bash
ros2 run vf_robot_gazebo ultrasound_py
```

### `teleop_twist_keyboard` — Keyboard teleop

Raw terminal keyboard control. Publishes `geometry_msgs/Twist` to `/cmd_vel`.

```bash
ros2 run vf_robot_gazebo teleop_twist_keyboard
```

**Key bindings:**

| Key | Action |
|---|---|
| `w` | Forward |
| `x` | Backward |
| `a` | Turn left |
| `d` | Turn right |
| `space` | Stop |
| `Ctrl+C` | Quit (publishes zero twist on exit) |

Default linear speed: `0.5 m/s` · Default angular speed: `1.0 rad/s`

---

## 🔧 Header Include Path Fix

The C++ ultrasound node uses a header at `include/vf_robot_gazebo/ultrasound.h`. 

**1. The header file location:**
```
include/vf_robot_gazebo/ultrasound.h   
```

**2. The `#include` directive in `ultrasound.cpp`:**
```cpp
#include "vf_robot_gazebo/ultrasound.h"
```

**3. `CMakeLists.txt` include path:**
```cmake
include_directories(include ${GAZEBO_INCLUDE_DIRS})
target_include_directories(ultrasound_cpp PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
```

If the path in the `.cpp` doesn't match the actual directory name under `include/`, the build fails with:
```
fatal error: vf_robot_gazebo/ultrasound.h: No such file or directory
```

Also note the ROS 2-style include for the custom message:
```cpp
#include <vf_robot_messages/msg/ultra_sound.hpp>   // ROS2 style

```

---

## ⚙️ Environment Setup

### `.bashrc` entries

Add these to `~/.bashrc` for the workspace to auto-source and Gazebo to find models:

```bash
# Workspace
source ~/cogni-nav-x0/install/setup.bash

# Gazebo model path — lets Gazebo find uvc1_hospital, uvc1_corridors etc.
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/cogni-nav-x0/install/share/vf_robot_gazebo/models

# Optional: suppress libGL warnings on some systems
export LIBGL_ALWAYS_SOFTWARE=0
```

Apply immediately:
```bash
source ~/.bashrc
```

### Verify Gazebo finds your models

```bash
echo $GAZEBO_MODEL_PATH
# Should include: .../install/share/vf_robot_gazebo/models
```

### Verify the package is found

```bash
ros2 pkg prefix vf_robot_gazebo
# → /home/<user>/cogni-nav-x0/install/vf_robot_gazebo

ros2 pkg list | grep vf_robot
# → vf_robot_description
# → vf_robot_gazebo
# → vf_robot_messages
```

---

## 🔨 Build & Run

### Prerequisites

```bash
sudo apt install \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-gazebo-ros \
  ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher \
  ros-humble-rqt-robot-steering \
  ros-humble-rviz2 \
  ros-humble-xacro \
  ros-humble-tf2-tools
```

### Build

```bash
cd ~/cogni-nav-x0

# Build both description and gazebo packages together
colcon build --packages-select vf_robot_description vf_robot_gazebo --symlink-install

# Or build just this package (if description is already built)
colcon build --packages-select vf_robot_gazebo --symlink-install

# Source
source install/setup.bash
```

> **`--symlink-install`** is important for Python scripts — changes to `.py` files under `vf_robot_gazebo/scripts/` are picked up without rebuilding. C++ files still require a rebuild.

### Quick smoke test

```bash
# 1. Confirm executables installed
ls install/vf_robot_gazebo/lib/vf_robot_gazebo/
# → teleop_twist_keyboard  ultrasound_cpp  ultrasound_py

# 2. Confirm launch files installed
ls install/vf_robot_gazebo/share/vf_robot_gazebo/launch/

# 3. Launch my_world with URDF
ros2 launch vf_robot_gazebo vf_my_world_urdf.launch.py
```

---

## 📖 All Commands Reference

### Build

```bash
# Full workspace build
colcon build --symlink-install

# Specific packages
colcon build --packages-select vf_robot_gazebo --symlink-install
colcon build --packages-select vf_robot_description vf_robot_gazebo --symlink-install

# With verbose output
colcon build --packages-select vf_robot_gazebo --event-handlers console_direct+
```

### Launch worlds

```bash
# Empty world
ros2 launch vf_robot_gazebo vf_empty_world_urdf.launch.py
ros2 launch vf_robot_gazebo vf_empty_world_sdf.launch.py

# My world (office-style)
ros2 launch vf_robot_gazebo vf_my_world_urdf.launch.py
ros2 launch vf_robot_gazebo vf_my_world_sdf.launch.py

# Hospital world
ros2 launch vf_robot_gazebo vf_hospital_world_urdf.launch.py
ros2 launch vf_robot_gazebo vf_hospital_world_sdf.launch.py

# Custom spawn pose
ros2 launch vf_robot_gazebo vf_my_world_urdf.launch.py x_pose:=5.0 y_pose:=2.0 theta:=0.0
```

### Run nodes individually

```bash
# C++ ultrasound aggregator
ros2 run vf_robot_gazebo ultrasound_cpp

# Python ultrasound aggregator
ros2 run vf_robot_gazebo ultrasound_py

# Keyboard teleop
ros2 run vf_robot_gazebo teleop_twist_keyboard
```

### Manual robot spawn (after Gazebo is already running)

```bash
# State publisher first (if using URDF spawn)
ros2 launch vf_robot_gazebo vf_robot_state_publisher.launch.py

# Spawn via URDF topic
ros2 launch vf_robot_gazebo vf_spawn_urdf.launch.py x_pose:=0.0 y_pose:=0.0

# Spawn via SDF file
ros2 launch vf_robot_gazebo vf_spawn_sdf.launch.py x_pose:=0.0 y_pose:=0.0
```

### Inspect topics

```bash
# Verify ultrasound topics published by Gazebo plugin
ros2 topic list | grep ultrasound
ros2 topic echo /ultrasound/front_left

# Check aggregated output
ros2 topic echo /esp/range

# Drive commands
ros2 topic echo /cmd_vel

# Odometry
ros2 topic echo /odom

# Joint states
ros2 topic echo /joint_states --once
```

### TF inspection

```bash
# View full TF tree (saves frames.pdf)
ros2 run tf2_tools view_frames

# Check specific transform
ros2 run tf2_ros tf2_echo base_link wheel_front_left_link
ros2 run tf2_ros tf2_echo odom base_footprint

# List all static transforms
ros2 topic echo /tf_static --once
```

### Verify robot description loaded

```bash
ros2 param get /robot_state_publisher robot_description | head -5
ros2 topic echo /robot_description --once | head -3
```

### rqt tools

```bash
# Drive robot with GUI sliders
ros2 run rqt_robot_steering rqt_robot_steering

# Topic monitor
ros2 run rqt_topic rqt_topic

# TF tree visual
ros2 run rqt_tf_tree rqt_tf_tree
```

### RViz standalone

```bash
# Open with the pre-configured nav2 layout
ros2 run rviz2 rviz2 -d $(ros2 pkg prefix vf_robot_gazebo)/share/vf_robot_gazebo/rviz/vf_robot_gazebo.rviz
```

### Confirm package install paths

```bash
ros2 pkg prefix vf_robot_gazebo
ros2 pkg prefix vf_robot_description

# Check model path visible to Gazebo
echo $GAZEBO_MODEL_PATH

# Find a specific world file
find $(ros2 pkg prefix vf_robot_gazebo) -name "my_world.world"
```

---

## 🐛 Troubleshooting

### `fatal error: vf_robot_gazebo/ultrasound.h: No such file or directory`

The `#include` in `ultrasound.cpp` doesn't match the actual directory name under `include/`.

```bash
# Check what directory exists
ls include/
# Must be: vf_robot_gazebo/

# Fix the include in ultrasound.cpp
# Change: #include "uvc1_gazebo/ultrasound.h"
# To:     #include "vf_robot_gazebo/ultrasound.h"
```

### `Could not find package vf_robot_messages`

```bash
colcon build --packages-select vf_robot_messages --symlink-install
source install/setup.bash
colcon build --packages-select vf_robot_gazebo --symlink-install
```

### Gazebo can't find `uvc1_hospital` or `uvc1_corridors` models

```bash
# Check model path
echo $GAZEBO_MODEL_PATH

# Should contain the models directory. If not:
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/cogni-nav-x0/install/share/vf_robot_gazebo/models
```

The `package.xml` export tag handles this automatically after sourcing:
```xml
<gazebo_ros gazebo_model_path="${prefix}/share/${name}/models"/>
```
If models still aren't found, add the export manually to `.bashrc`.

### Robot spawns but no TF / joint states

The Gazebo diff drive and joint state publisher plugins are only in `model.sdf`, not in the raw URDF. Confirm:

```bash
# Should show odom→base_footprint being broadcast
ros2 topic echo /tf --once | grep frame_id

# Should show 6 joint values
ros2 topic echo /joint_states --once
```

If `/joint_states` is missing, the Gazebo plugin failed to load. Check `gzserver` terminal for plugin errors.

### RViz `RobotModel` shows "No transform from odom to base_footprint"

Gazebo isn't publishing `odom→base_footprint`. Usually the diff drive plugin failed. Check:
```bash
# Is Gazebo actually running?
ros2 topic list | grep odom

# Did spawn succeed?
ros2 service list | grep gazebo
```

### Python script not found after build

With `--symlink-install`, Python scripts are symlinked. Without it they are copied — changes require rebuild.

```bash
# Check the script is installed
ls install/vf_robot_gazebo/lib/vf_robot_gazebo/

# If missing, rebuild
colcon build --packages-select vf_robot_gazebo --symlink-install
```

### `setup.py install is deprecated` warning

This is a colcon/pip warning about `setup.py` in hybrid packages. It is cosmetic and does not affect the build. The package still installs correctly.

---
## ⚡ if robot model error?  Quick Setup

Add to `~/.bashrc`:
```bash
export UVC1_MODEL=virofighter
```

Verify:
```bash
echo $UVC1_MODEL
```

Expected output:
```
virofighter
```

## 🗺️ TF Chain (Simulation)

```
map  ──►  odom  ──────────────►  base_footprint  ──►  base_link  ──►  all sensor/wheel links
      (amcl/slam)   (gazebo diff drive plugin)    (fixed)        (robot_state_publisher)
```

| Transform | Publisher | Requires |
|---|---|---|
| `map → odom` | `amcl` or `slam_toolbox` | Nav2 + map |
| `odom → base_footprint` | Gazebo diff drive plugin | Gazebo + spawned robot |
| `base_footprint → base_link` | `robot_state_publisher` | URDF loaded |
| `base_link → wheels/sensors` | `robot_state_publisher` + `/joint_states` | Always |

---

## 📄 License

Apache 2.0 — see [LICENSE](LICENSE)

---

## 👤 Maintainer

**Pravin** — olipravin18@gmail.com
Project: **cogni-nav-x0** | Package: `vf_robot_gazebo`
