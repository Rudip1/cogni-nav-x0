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

- [🌟 Overview](#-overview)
- [⚡ Quick Start](#-quick-start)
- [📁 Package Structure](#-package-structure)
- [🏗️ Architecture: Single Source of Truth](#️-architecture-single-source-of-truth)
- [🔀 Two Simulation Pipelines: Xacro vs SDF](#-two-simulation-pipelines-xacro-vs-sdf)
- [🔗 Why robot_state_publisher is Shared](#-why-robot_state_publisher-is-shared)
- [🚀 Launch Files](#-launch-files)
- [🌍 Worlds](#-worlds)
- [🔧 Nodes & Scripts](#-nodes--scripts)
- [⚙️ Environment Variables (Critical)](#️-environment-variables-critical)
- [🗺️ TF Chain (Simulation)](#️-tf-chain-simulation)
- [🔨 Build & Run](#-build--run)
- [📖 All Commands Reference](#-all-commands-reference)
- [🐛 Troubleshooting](#-troubleshooting)
- [📄 License](#-license)

---

## 🌟 Overview

`vf_robot_gazebo` is the **standalone Gazebo simulation package** for the ViroFighter UVC-1 robot. It contains worlds, models, launch files, and sensor nodes. It does **not** contain the URDF or xacro files — those live in `vf_robot_description` (single source of truth).

This package is a **hybrid `ament_cmake` + Python** package, meaning it compiles C++ executables and installs Python scripts from the same `CMakeLists.txt`. The build system is `ament_cmake` (not `ament_python`), with `ament_cmake_python` bridging Python support.

**Key responsibilities:**

- 🌍 Gazebo worlds (hospital, corridors, my_world, empty, etc.)
- 🏗️ Gazebo models (robot SDF, environment models, meshes)
- 🚀 Launch files for every world × (Xacro | SDF) combination
- 🔊 C++ ultrasound aggregator node (`ultrasound_cpp`)
- 🐍 Python ultrasound aggregator node (`ultrasound_py`)
- 🎮 Keyboard teleop script

**Two parallel simulation pipelines:**

- **Xacro pipeline** — processes xacro from `vf_robot_description` at runtime (development)
- **SDF pipeline** — uses pre-converted `model.sdf` for faster startup (production)

Both pipelines share the same `robot_state_publisher` for TF tree publishing.

![ViroFighter Gazebo Demo](docs/vf_robot_gazebo.gif)

---

## ⚡ Quick Start

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

# Source
source install/setup.bash
```

### Launch Simulation (Xacro pipeline — recommended for development)

```bash
# Empty world — Gazebo only, no RViz
ros2 launch vf_robot_gazebo vf_empty_world_xacro.launch.py

# My world with RViz (costmap + Nav2 panels)
ros2 launch vf_robot_gazebo vf_my_world_xacro_rviz.launch.py
```

### Launch Simulation (SDF pipeline — faster startup)

```bash
# Empty world — Gazebo only, no RViz
ros2 launch vf_robot_gazebo vf_empty_world_sdf.launch.py

# My world with RViz (costmap + Nav2 panels)
ros2 launch vf_robot_gazebo vf_my_world_sdf_rviz.launch.py
```

You should see Gazebo with the ViroFighter robot and rqt_robot_steering GUI.

---

## 📁 Package Structure

```
vf_robot_gazebo/
├── 📂 CMakeLists.txt                     # Hybrid ament_cmake + Python build
├── 📂 package.xml                        # Declares ament_cmake build type
├── 📂 setup.py                           # Python module install only (no console_scripts)
├── 📂 docs/
│   └── vf_robot_gazebo.gif               # Demo animation
├── 📂 include/
│   └── vf_robot_gazebo/
│       └── ultrasound.h                  # C++ node header
├── 📂 src/
│   └── ultrasound.cpp                    # C++ ultrasound aggregator
├── 📂 vf_robot_gazebo/                   # Python package (importable)
│   ├── __init__.py
│   └── scripts/
│       ├── teleop_twist_keyboard.py      # Keyboard teleop
│       └── ultrasound_py.py              # Python ultrasound aggregator
├── 📂 launch/
│   ├── vf_robot_state_publisher.launch.py   # Shared by BOTH pipelines
│   ├── vf_spawn_xacro.launch.py             # Spawns from /robot_description topic
│   ├── vf_spawn_sdf.launch.py               # Spawns from model.sdf file
│   ├── vf_empty_world_xacro.launch.py       # Xacro pipeline, no RViz
│   ├── vf_empty_world_sdf.launch.py         # SDF pipeline, no RViz
│   ├── vf_my_world_xacro.launch.py          # Xacro pipeline, no RViz
│   ├── vf_my_world_xacro_rviz.launch.py     # Xacro pipeline + RViz
│   ├── vf_my_world_sdf.launch.py            # SDF pipeline, no RViz
│   ├── vf_my_world_sdf_rviz.launch.py       # SDF pipeline + RViz
│   ├── vf_hospital_world_xacro.launch.py
│   └── vf_hospital_world_sdf.launch.py
├── 📂 models/
│   ├── uvc1_virofighter/                 # Robot Gazebo model
│   │   ├── model.config
│   │   └── model.sdf                     # ⚠️ AUTO-GENERATED — never edit manually
│   ├── uvc1_common/                      # Shared meshes for model:// resolution
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
├── 📂 rviz/
│   └── vf_robot_gazebo.rviz              # Full nav2+costmap RViz config
├── 📂 worlds/
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
└── 📂 resource/
    └── vf_robot_gazebo                   # ament index marker
```

---

## 🏗️ Architecture: Single Source of Truth

The robot model lives in `vf_robot_description`, not here. This is intentional — one source of truth for the robot model shared between real robot bringup and simulation.

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                         vf_robot_description                                 │
│                      (SINGLE SOURCE OF TRUTH)                                │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│  urdf/xacro/                                                                │
│  ├── uvc1_virofighter.xacro  ◄── MASTER FILE                               │
│  │       ├── includes: common_properties.xacro                             │
│  │       ├── includes: sensors.xacro                                       │
│  │       └── contains: <gazebo> plugin blocks                              │
│  ├── common_properties.xacro (materials)                                   │
│  ├── sensors.xacro (D455, D435i, fisheye×4, ultrasonic×5)                  │
│  └── xacro_to_sdf.sh → converts to model.sdf                               │
│                                                                             │
│  meshes/  (package://vf_robot_description/meshes/...)                       │
│  ├── bases/virofighter_base.dae, Cube.019.dae                              │
│  ├── sensors/camera_d435i.dae, ...                                         │
│  └── wheels/wheel_*.dae, wrist_*.dae                                       │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
                           │
                           │ xacro processed at runtime via
                           │ $(find vf_robot_description)
                           ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                          vf_robot_gazebo                                     │
│                       (SIMULATION PACKAGE)                                   │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│  models/uvc1_virofighter/model.sdf  ← AUTO-GENERATED by xacro_to_sdf.sh    │
│  models/uvc1_common/meshes/         ← COPY for model:// URI resolution     │
│  worlds/*.world                     ← Gazebo world files                   │
│  launch/*_xacro.launch.py           ← Xacro pipeline launches              │
│  launch/*_sdf.launch.py             ← SDF pipeline launches                │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

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
xacro_path = PathJoinSubstitution([
    FindPackageShare('vf_robot_description'),
    'urdf', 'xacro',
    'uvc1_virofighter.xacro',
])
```

`colcon` build order is enforced by the `<depend>` tag — `vf_robot_description` is always built before `vf_robot_gazebo`.

### Mesh URIs — Two Different Schemes

| Pipeline | Mesh URI Format | Resolved By |
|----------|-----------------|-------------|
| **Xacro** | `package://vf_robot_description/meshes/...` | ament package index |
| **SDF** | `model://uvc1_common/meshes/...` | `GAZEBO_MODEL_PATH` |

This is why `models/uvc1_common/meshes/` contains a **copy** of the meshes from `vf_robot_description` — Gazebo's `model://` resolver needs them in the model path.

---

## 🔀 Two Simulation Pipelines: Xacro vs SDF

Every world has two launch variants: `*_xacro.launch.py` and `*_sdf.launch.py`. Understanding the difference matters.

### Xacro Pipeline (`*_xacro.launch.py`)

```
vf_robot_state_publisher.launch.py
       │
       ├──► xacro uvc1_virofighter.xacro (from vf_robot_description)
       │         │
       │         ▼
       │    /robot_description topic ───► spawn_entity -topic robot_description
       │         │                                │
       │         ▼                                ▼
       └──► robot_state_publisher           Gazebo (robot appears)
                 │                                │
                 ▼                                ▼
            TF broadcasts ◄────────────── /joint_states
```

**Characteristics:**

- Requires `vf_robot_state_publisher.launch.py` to run first (or be included)
- Xacro is processed at launch time, then published on `/robot_description`
- `spawn_entity.py` reads from the topic, converts internally to SDF, and spawns
- `/robot_description` stays alive — RViz `RobotModel` display works
- Joint states from Gazebo plugin → `robot_state_publisher` → TF tree ✅
- **Requires timing delays** (gzclient 3s, spawn 5s) to prevent race conditions
- Changes to xacro are reflected immediately on next launch (no rebuild needed)

### SDF Pipeline (`*_sdf.launch.py`)

```
vf_spawn_sdf.launch.py
       │
       └──► spawn_entity -file model.sdf ──► Gazebo (robot appears)
                                                   │
                                                   ▼
vf_robot_state_publisher.launch.py           /joint_states
       │                                           │
       ├──► xacro uvc1_virofighter.xacro          │
       │         │                                 │
       │         ▼                                 │
       └──► robot_state_publisher ◄────────────────┘
                 │
                 ▼
            TF broadcasts
```

**Characteristics:**

- Reads the pre-built `model.sdf` directly from this package's `models/` folder
- Does **not** require xacro processing for spawn (faster startup)
- Gazebo joint state plugin still publishes `/joint_states`
- **Still needs `robot_state_publisher`** for TF tree (explained below)
- `/robot_description` is published for RViz robot model display
- **No timing delays needed** — model:// URIs are resolved differently
- Requires running `xacro_to_sdf.sh` after xacro edits

### Comparison Table

| Feature | Xacro Pipeline | SDF Pipeline |
|---------|---------------|--------------|
| Source file | Live xacro processing | Pre-converted `model.sdf` |
| Mesh URIs | `package://vf_robot_description/...` | `model://uvc1_common/...` |
| Edit workflow | Edit xacro → launch | Edit xacro → run `xacro_to_sdf.sh` → launch |
| Startup time | Slower (xacro processing + delays) | Faster |
| Timing delays | Yes (gzclient 3s, spawn 5s) | No |
| RViz robot model | ✅ Works | ✅ Works |
| TF tree | ✅ Full | ✅ Full |
| Use case | Development, testing | Production, demos |

### When Gazebo strips plugins

`gz sdf -p` (the URDF→SDF converter) strips all `<gazebo>` plugin tags and breaks `model://` mesh URIs. The `xacro_to_sdf.sh` script in `vf_robot_description` fixes both and writes output to `models/uvc1_virofighter/model.sdf` in this package. **Never use raw gz sdf output directly. Never edit model.sdf manually.**

### Choosing between them

| Use case | Launch variant |
|----------|----------------|
| Development / rapid iteration | `*_xacro.launch.py` |
| Production / demos | `*_sdf.launch.py` |
| Full simulation with RViz | Either (both work) |
| SLAM / Nav2 (needs `/robot_description`) | Either (both publish it) |
| Quick Gazebo-only test | `*_sdf.launch.py` |

---

## 🔗 Why `robot_state_publisher` is Shared

**Both pipelines use the same `vf_robot_state_publisher.launch.py`!**

This is not obvious at first, so here's the full explanation.

### The TF Tree Problem

Gazebo's `libgazebo_ros_diff_drive.so` plugin **only publishes ONE transform**:

```
odom → base_footprint
```

That's it. Gazebo doesn't know or care about your robot's internal link structure (base_link, wheels, sensors, etc.).

But RViz and Nav2 need the **complete TF tree** to visualize and navigate:

```
odom
  └── base_footprint
        └── base_link
              ├── wheel_front_left_link
              ├── wheel_front_right_link
              ├── camera_d455_link
              │     ├── camera_d455_color_optical_frame
              │     ├── camera_d455_depth_optical_frame
              │     └── camera_d455_imu_frame
              ├── camera_d435i_link
              │     ├── camera_d435i_color_optical_frame
              │     ├── camera_d435i_depth_optical_frame
              │     └── camera_d435i_imu_frame
              ├── camera_fisheye_front_link
              ├── camera_fisheye_left_link
              ├── camera_fisheye_right_link
              ├── camera_fisheye_rear_link
              ├── ultrasonic_front_left_link
              ├── ultrasonic_front_right_link
              ├── ultrasonic_side_left_link
              ├── ultrasonic_side_right_link
              ├── ultrasonic_rear_link
              ├── wrist_rear_left_link
              │     └── wheel_rear_left_link
              ├── wrist_rear_right_link
              │     └── wheel_rear_right_link
              └── uvc_lights_link
```

**Who publishes all those other transforms?** → `robot_state_publisher`

### What `robot_state_publisher` Does

1. **Reads** the xacro from `vf_robot_description` (always the xacro, never the SDF)
2. **Parses** all `<joint>` elements to understand the kinematic tree
3. **Subscribes** to `/joint_states` topic (published by Gazebo's joint_state plugin)
4. **Publishes** TF transforms for every link in the robot
5. **Publishes** `/robot_description` topic (for RViz robot model display)

### Why SDF Mode Still Needs RSP

Even though SDF mode spawns from `model.sdf`, it still needs `robot_state_publisher` because:

| Reason | Explanation |
|--------|-------------|
| **SDF format is different** | RSP only understands URDF format, not SDF |
| **Gazebo only publishes one TF** | `odom → base_footprint` — all other TF comes from RSP |
| **RViz needs `/robot_description`** | RSP publishes this topic for the robot model display |
| **Single source of truth** | RSP reads xacro, keeping TF in sync with the actual robot definition |
| **Nav2 needs sensor frames** | Without RSP, sensor TF frames don't exist |

### The Code in `vf_robot_state_publisher.launch.py`

```python
# Always reads from vf_robot_description xacro (single source of truth)
xacro_path = PathJoinSubstitution([
    FindPackageShare('vf_robot_description'),  # ← Always this package
    'urdf', 'xacro',
    'uvc1_virofighter.xacro',
])

# Process xacro → URDF string
robot_description_content = ParameterValue(
    Command([FindExecutable(name='xacro'), ' ', xacro_path]),
    value_type=str,
)

# robot_state_publisher node
Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    parameters=[{
        'use_sim_time': use_sim_time,  # ← Important for Gazebo clock sync
        'robot_description': robot_description_content,
    }],
)
```

### What Happens Without RSP?

| Component | With RSP | Without RSP |
|-----------|----------|-------------|
| Robot in Gazebo | ✅ Appears | ✅ Appears |
| Robot moves with `/cmd_vel` | ✅ Works | ✅ Works |
| `/odom` topic | ✅ Works | ✅ Works |
| `/joint_states` topic | ✅ Works | ✅ Works |
| RViz robot model | ✅ Shows | ❌ Blank |
| TF: `odom → base_footprint` | ✅ Exists | ✅ Exists (from Gazebo) |
| TF: `base_footprint → base_link` | ✅ Exists | ❌ Missing |
| TF: sensor frames | ✅ Exist | ❌ Missing |
| Nav2 can find sensors | ✅ Yes | ❌ No |

---

## 🚀 Launch Files

### Helper Launches (included by world launches)

| Launch File | Purpose | Used By |
|-------------|---------|---------|
| `vf_robot_state_publisher.launch.py` | Processes xacro, publishes `/robot_description` and TF | **Both pipelines** |
| `vf_spawn_xacro.launch.py` | Spawns robot from `/robot_description` topic | Xacro pipeline |
| `vf_spawn_sdf.launch.py` | Spawns robot from `model.sdf` file directly | SDF pipeline |

### `vf_robot_state_publisher.launch.py`

Reads the xacro from `vf_robot_description` and publishes `/robot_description` + TF. This is a dependency of **all** world launches (both Xacro and SDF).

```bash
ros2 launch vf_robot_gazebo vf_robot_state_publisher.launch.py
ros2 launch vf_robot_gazebo vf_robot_state_publisher.launch.py use_sim_time:=false
```

### `vf_spawn_xacro.launch.py`

Spawns the robot by reading `/robot_description` topic. Requires state publisher already running.

```bash
ros2 launch vf_robot_gazebo vf_spawn_xacro.launch.py
ros2 launch vf_robot_gazebo vf_spawn_xacro.launch.py x_pose:=2.0 y_pose:=1.0 theta:=1.57
```

### `vf_spawn_sdf.launch.py`

Spawns the robot directly from `models/uvc1_virofighter/model.sdf`. No topic dependency.

```bash
ros2 launch vf_robot_gazebo vf_spawn_sdf.launch.py
ros2 launch vf_robot_gazebo vf_spawn_sdf.launch.py x_pose:=0.0 y_pose:=0.0 theta:=0.0
```

---

### World Launches — Xacro Pipeline

Each includes: gzserver + gzclient (delayed 3s) + robot_state_publisher + spawn_xacro (delayed 5s) + rqt_robot_steering.

```bash
ros2 launch vf_robot_gazebo vf_empty_world_xacro.launch.py
ros2 launch vf_robot_gazebo vf_my_world_xacro.launch.py
ros2 launch vf_robot_gazebo vf_hospital_world_xacro.launch.py
```

**Why the delays?**

| Component | Delay | Reason |
|-----------|-------|--------|
| gzclient | 3s | Prevents "Assertion px != 0" GUI race crash |
| spawn_entity | 5s | Prevents gzserver crash when loading `package://` mesh URIs before rendering scene is ready |
| rviz2 | 6s | Ensures robot is spawned before visualization |

---

### World Launches — SDF Pipeline

Each includes: gzserver + gzclient + robot_state_publisher + spawn_sdf + rqt_robot_steering. No delays needed.

```bash
ros2 launch vf_robot_gazebo vf_empty_world_sdf.launch.py
ros2 launch vf_robot_gazebo vf_my_world_sdf.launch.py
ros2 launch vf_robot_gazebo vf_hospital_world_sdf.launch.py
```

---

### Launch Arguments (all world launches)

| Argument | Default | Description |
|----------|---------|-------------|
| `use_sim_time` | `true` | Use Gazebo clock |
| `x_pose` | varies by world | Spawn X position |
| `y_pose` | varies by world | Spawn Y position |
| `theta` | varies by world | Spawn yaw (radians) |

**Default spawn poses:**

| World | x_pose | y_pose | theta |
|-------|--------|--------|-------|
| `empty_world` | `0.0` | `0.0` | `0.0` |
| `hospital` | `-2.0` | `-0.5` | `0.0` |
| `my_world` | `9.0` | `0.5` | `3.14` |

**Example with custom pose:**

```bash
ros2 launch vf_robot_gazebo vf_my_world_xacro.launch.py x_pose:=5.0 y_pose:=2.0 theta:=1.57
```

---

## 🌍 Worlds

| World file | Description |
|------------|-------------|
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

---

## 🔧 Nodes & Scripts

### `ultrasound_cpp` — C++ aggregator

Subscribes to 5 individual ultrasonic `sensor_msgs/Range` topics and republishes as `vf_robot_messages/UltraSound` on `/esp/range`. The `code` field maps frame_id to sensor index (0–4).

```bash
ros2 run vf_robot_gazebo ultrasound_cpp
```

**Subscribed topics:**

| Topic | Sensor code |
|-------|-------------|
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
|-----|--------|
| `w` | Forward |
| `x` | Backward |
| `a` | Turn left |
| `d` | Turn right |
| `space` | Stop |
| `Ctrl+C` | Quit (publishes zero twist on exit) |

Default linear speed: `0.5 m/s` · Default angular speed: `1.0 rad/s`

---

## ⚙️ Environment Variables (Critical)

### The Problem: SetEnvironmentVariable Replaces, Not Prepends

ROS 2 launch `SetEnvironmentVariable` **replaces** environment variables by default. If you do this:

```python
# ❌ WRONG — destroys Gazebo's own paths
SetEnvironmentVariable(
    name='GAZEBO_RESOURCE_PATH',
    value=os.path.dirname(pkg_desc),
)
```

You destroy Gazebo's own resource paths (`/usr/share/gazebo-11`), causing:

- RTShaderSystem errors
- gzserver exit code 255
- Missing textures and shaders
- Gazebo crashes on startup

### The Solution: Hardcode All Required Paths

The corrected launch files **prepend** by hardcoding all required paths:

```python
# ✅ CORRECT — preserves system paths
SetEnvironmentVariable(
    name='GAZEBO_RESOURCE_PATH',
    value=os.pathsep.join([
        os.path.dirname(pkg_desc),           # Your package
        '/usr/share/gazebo-11',              # Gazebo resources (REQUIRED)
        '/opt/ros/humble/share',             # ROS resources
    ]),
)

SetEnvironmentVariable(
    name='GAZEBO_MODEL_PATH',
    value=os.pathsep.join([
        os.path.join(pkg_vf_gazebo, 'models'),  # Your models
        os.path.dirname(pkg_desc),              # For package:// resolution
        '/usr/share/gazebo-11/models',          # Gazebo default models
    ]),
)

SetEnvironmentVariable(
    name='GAZEBO_PLUGIN_PATH',
    value=os.pathsep.join([
        '/opt/ros/humble/lib',                              # ROS Gazebo plugins
        '/usr/lib/x86_64-linux-gnu/gazebo-11/plugins',      # Gazebo plugins
    ]),
)
```

### Why Hardcode Instead of Reading os.environ?

Launch files run in a subprocess where environment variables may not be set yet. Reading `os.environ.get('GAZEBO_RESOURCE_PATH', '')` often returns empty, so prepending to empty still loses the system paths.

### `.bashrc` Setup (Optional)

The launch files handle environment variables automatically. But you can add these to `~/.bashrc` for manual testing:

```bash
# Workspace
source ~/cogni-nav-x0/install/setup.bash

# Optional: Gazebo model path (launch files handle this automatically)
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/cogni-nav-x0/install/share/vf_robot_gazebo/models
```

---

## 🗺️ TF Chain (Simulation)

```
[navigation]        [gazebo]                   [robot_state_publisher]
     │                   │                            │
     ▼                   ▼                            ▼
   map    ──────►   odom    ──────►   base_footprint ──► base_link ──► all links
              (amcl/slam)    (diff drive)        (fixed)        (RSP + /joint_states)
```

### Who publishes what

| TF Transform | Publisher | When available |
|--------------|-----------|----------------|
| `map → odom` | `amcl` or `slam_toolbox` | Nav2 running + map loaded |
| `odom → base_footprint` | Gazebo diff drive plugin | Gazebo running |
| `base_footprint → base_link` | `robot_state_publisher` | Always (start RSP first) |
| `base_link → wheel_*` | `robot_state_publisher` + `/joint_states` | Always |
| `base_link → camera_*` | `robot_state_publisher` (static) | Always |
| `base_link → ultrasonic_*` | `robot_state_publisher` (static) | Always |

### Verifying the TF tree

```bash
# Save TF tree to frames.pdf
ros2 run tf2_tools view_frames

# Check diff drive TF (from Gazebo)
ros2 run tf2_ros tf2_echo odom base_footprint

# Check RSP TF (from robot_state_publisher)
ros2 run tf2_ros tf2_echo base_link camera_d455_link

# List all frames
ros2 topic echo /tf --once | grep frame_id
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

# 3. Launch empty_world with xacro pipeline
ros2 launch vf_robot_gazebo vf_empty_world_xacro.launch.py

# 4. Or with SDF pipeline (faster)
ros2 launch vf_robot_gazebo vf_empty_world_sdf.launch.py
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

### Launch worlds — Xacro pipeline (development)

```bash
ros2 launch vf_robot_gazebo vf_empty_world_xacro.launch.py
ros2 launch vf_robot_gazebo vf_my_world_xacro.launch.py
ros2 launch vf_robot_gazebo vf_hospital_world_xacro.launch.py

# Custom spawn pose
ros2 launch vf_robot_gazebo vf_my_world_xacro.launch.py x_pose:=5.0 y_pose:=2.0 theta:=0.0
```

### Launch worlds — SDF pipeline (production)

```bash
ros2 launch vf_robot_gazebo vf_empty_world_sdf.launch.py
ros2 launch vf_robot_gazebo vf_my_world_sdf.launch.py
ros2 launch vf_robot_gazebo vf_hospital_world_sdf.launch.py
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
# State publisher first
ros2 launch vf_robot_gazebo vf_robot_state_publisher.launch.py

# Spawn via xacro topic
ros2 launch vf_robot_gazebo vf_spawn_xacro.launch.py x_pose:=0.0 y_pose:=0.0

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

# Robot description
ros2 topic echo /robot_description --once | head -5
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

### Verify robot_state_publisher

```bash
# Check RSP is running
ros2 node list | grep robot_state_publisher

# Check /robot_description is published
ros2 topic list | grep robot_description

# Check TF is being broadcast
ros2 topic echo /tf --once | grep frame_id
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
# Open with the pre-configured layout
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

### gzserver crashes with exit code 255 / RTShaderSystem errors

**Cause:** `GAZEBO_RESOURCE_PATH` was replaced instead of prepended, destroying Gazebo's shader paths.

**Fix:** Use the corrected launch files that hardcode system paths:
```python
value=os.pathsep.join([
    os.path.dirname(pkg_desc),
    '/usr/share/gazebo-11',        # ← This was missing
    '/opt/ros/humble/share',
])
```

---

### RViz robot model is blank / "No transform from X to Y"

**Cause:** `robot_state_publisher` is not running or failed.

**Fix:**
```bash
# Check RSP is running
ros2 node list | grep robot_state_publisher

# Check /robot_description exists
ros2 topic list | grep robot_description

# If missing, RSP failed — check launch output for errors
```

---

### No TF from odom to base_footprint

**Cause:** Gazebo diff drive plugin failed to load or robot didn't spawn.

**Fix:**
```bash
# Is Gazebo running?
ros2 topic list | grep odom

# Did spawn succeed?
ros2 service list | grep gazebo

# Check gzserver terminal for plugin errors
```

---

### Robot spawns but sensors have no TF frames

**Cause:** `robot_state_publisher` is not running.

**Fix:** Ensure `vf_robot_state_publisher.launch.py` is included in your launch. Both Xacro and SDF pipelines need it.

---

### Xacro changes not reflected in SDF pipeline

**Cause:** SDF pipeline uses pre-converted `model.sdf`, not live xacro.

**Fix:** Run the conversion script after editing xacro:
```bash
cd ~/cogni-nav-x0/src/vf_robot_description/urdf/xacro
./xacro_to_sdf.sh uvc1_virofighter.xacro
```

---

### Gazebo can't find `uvc1_hospital` or `uvc1_corridors` models

**Cause:** `GAZEBO_MODEL_PATH` doesn't include the models directory.

**Fix:** The corrected launch files set this automatically. If still failing:
```bash
# Check model path
echo $GAZEBO_MODEL_PATH

# Should contain the models directory. If not, add manually:
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/cogni-nav-x0/install/share/vf_robot_gazebo/models
```

---

### Python script not found after build

**Cause:** Without `--symlink-install`, Python scripts are copied — changes require rebuild.

**Fix:**
```bash
# Check the script is installed
ls install/vf_robot_gazebo/lib/vf_robot_gazebo/

# If missing or outdated, rebuild with symlink
colcon build --packages-select vf_robot_gazebo --symlink-install
```

---

### `setup.py install is deprecated` warning

This is a colcon/pip warning about `setup.py` in hybrid packages. It is cosmetic and does not affect the build. The package still installs correctly.

---

### `fatal error: vf_robot_gazebo/ultrasound.h: No such file or directory`

**Cause:** The `#include` in `ultrasound.cpp` doesn't match the actual directory name under `include/`.

**Fix:**
```bash
# Check what directory exists
ls include/
# Must be: vf_robot_gazebo/

# The include in ultrasound.cpp must match:
#include "vf_robot_gazebo/ultrasound.h"
```

---

### `Could not find package vf_robot_messages`

**Fix:**
```bash
colcon build --packages-select vf_robot_messages --symlink-install
source install/setup.bash
colcon build --packages-select vf_robot_gazebo --symlink-install
```

---

## 📄 License

Apache 2.0 — see [LICENSE](LICENSE)

---

## 👤 Maintainer

**Pravin Oli** — olipravin18@gmail.com
Project: **cogni-nav-x0** | Package: `vf_robot_gazebo`
