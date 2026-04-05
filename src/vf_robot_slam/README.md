# 🤖 `vf_robot_slam`

> **ViroFighter UVC-1 Robot — SLAM & Localization Package**
> ROS 2 Humble · RTAB-Map · Intel RealSense D435i + D455

[![ROS2 Humble](https://img.shields.io/badge/ROS2-Humble-blue?style=flat-square&logo=ros)](https://docs.ros.org/en/humble/)
[![RTAB-Map](https://img.shields.io/badge/RTAB--Map-Visual%20SLAM-orange?style=flat-square)](http://introlab.github.io/rtabmap/)
[![Nav2](https://img.shields.io/badge/Nav2-Compatible-green?style=flat-square)](https://navigation.ros.org/)
[![License](https://img.shields.io/badge/License-Apache%202.0-green?style=flat-square)](LICENSE)

---

## 📋 Table of Contents

- [🌟 Overview](#-overview)
- [⚡ Quick Start](#-quick-start)
- [📁 Package Structure](#-package-structure)
- [🏗️ Architecture: Modular Launch Design](#️-architecture-modular-launch-design)
- [📷 Camera Configuration](#-camera-configuration)
- [🔀 Two Depth-to-Scan Methods: dimg vs pc2scan](#-two-depth-to-scan-methods-dimg-vs-pc2scan)
- [🔗 Dual Camera Scan Merging](#-dual-camera-scan-merging)
- [🚀 Launch Files](#-launch-files)
- [🗺️ Operating Modes (4 Modes)](#️-operating-modes-4-modes)
- [📦 Map Storage & Workflow](#-map-storage--workflow)
- [⚙️ Launch Arguments Reference](#️-launch-arguments-reference)
- [📡 Topics & TF Published](#-topics--tf-published)
- [🔧 Nav2 Costmap Configuration](#-nav2-costmap-configuration)
- [📖 All Commands Reference](#-all-commands-reference)
- [🐛 Troubleshooting](#-troubleshooting)
- [📄 License](#-license)

---

## 🌟 Overview

`vf_robot_slam` provides all SLAM, localization, and depth-to-laserscan capabilities for the ViroFighter UVC-1 robot. The robot has **no 2D lidar** — it relies entirely on Intel RealSense depth cameras for perception, making visual SLAM the core approach.

**Key responsibilities:**

- 🗺️ RTAB-Map SLAM (build new maps with RGB-D cameras)
- 📍 RTAB-Map Localization (navigate in previously built maps)
- 🔄 Depth-to-LaserScan conversion (for Nav2 / AMCL / SLAM Toolbox compatibility)
- 🔀 Dual camera scan merging (via `ira_laser_tools` or `topic_tools relay` fallback)

**Design principles:**

- **Modular launch architecture** — reusable `include/` building blocks, no copy-paste
- **Single entry point** — `depth_to_scan.launch.py` routes to the correct method
- **All parameters inline** — no external YAML config files that drift out of sync
- **Future-proof** — designed for integration with `vf_robot_navigation`

---

## ⚡ Quick Start

### Prerequisites

```bash
sudo apt install ros-humble-rtabmap-ros \
                 ros-humble-depthimage-to-laserscan \
                 ros-humble-pointcloud-to-laserscan \
                 ros-humble-topic-tools
```

### Build

```bash
cd ~/cogni-nav-x0
colcon build --packages-select vf_robot_slam --symlink-install
source install/setup.bash
```

### Launch SLAM (Gazebo must be running first)

```bash
# Terminal 1: Gazebo
ros2 launch vf_robot_gazebo vf_my_world_xacro.launch.py

# Terminal 2: RTAB-Map SLAM
ros2 launch vf_robot_slam rtabmap_slam.launch.py camera:=dual map_name:=my_office

# Terminal 3: Depth to LaserScan (for Nav2 costmap)
ros2 launch vf_robot_slam depth_to_scan.launch.py camera:=dual

# Terminal 4: Drive the robot
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

---

## 📁 Package Structure

```
vf_robot_slam/
├── CMakeLists.txt
├── package.xml
├── README.md
├── config/
│   ├── cameras/                          # Camera topic/frame reference
│   │   ├── d435i.yaml                    #   D435i topics, frames, specs
│   │   ├── d455.yaml                     #   D455 topics, frames, specs
│   │   └── dual.yaml                     #   Both cameras combined
│   └── laser_merger/
│       └── merger.yaml                   # ira_laser_tools config (loaded by launch)
├── launch/
│   ├── rtabmap_slam.launch.py            # Mode 1: RTAB-Map SLAM
│   ├── rtabmap_loc.launch.py             # Mode 2: RTAB-Map Localization
│   ├── depth_to_scan.launch.py           # Router → picks dimg or pc2scan method
│   └── include/                          # Reusable building blocks
│       ├── rgbd_sync.launch.py           #   RGBD sync nodes (shared by slam + loc)
│       ├── depth_to_scan_dimg.launch.py  #   depthimage_to_laserscan (Gazebo-fast)
│       └── depth_to_scan_pc2scan.launch.py # pointcloud_to_laserscan (real robot)
└── rviz/
    ├── rtabmap_slam.rviz
    └── rtabmap_loc.rviz

# Maps stored OUTSIDE the package at workspace level:
~/cogni-nav-x0/maps/                      # Auto-created by launch files
```

### Why no config YAML files for RTAB-Map parameters?

All RTAB-Map parameters are defined **inline** in the launch files with detailed comments explaining *why* each value is set. This is intentional:

- ROS 2 YAML loading requires the exact node name as the top-level key — node names change depending on camera mode, so no single YAML can serve all modes
- Inline parameters with comments serve as both config AND documentation
- No risk of YAML files drifting out of sync with actual launch behavior

---

## 🏗️ Architecture: Modular Launch Design

The launch files use an `include/` folder to eliminate code duplication. Think of it as building with reusable blocks.

### The Problem (Before)

`rtabmap_slam.launch.py` and `rtabmap_loc.launch.py` both contained **identical** rgbd_sync node blocks (~40 lines each). Any fix had to be applied in two places — miss one and you get a bug that only shows up in one mode.

### The Solution

```
include/rgbd_sync.launch.py          ← Defines sync nodes ONCE
         │
         ├── rtabmap_slam.launch.py   Includes rgbd_sync + adds RTAB-Map SLAM
         └── rtabmap_loc.launch.py    Includes rgbd_sync + adds RTAB-Map LOC
```

Same pattern for depth-to-scan — one router, two methods:

```
depth_to_scan.launch.py               ← Router (the ONLY file users/nav call)
         │
         │  argument: method:=dimg or method:=pc2scan
         │
         ├── include/depth_to_scan_dimg.launch.py      (depthimage_to_laserscan)
         └── include/depth_to_scan_pc2scan.launch.py   (pointcloud_to_laserscan)
```

### Why This Matters for `vf_robot_navigation`

The navigation package only needs to know **one launch file name** per function:

| Navigation calls | From this package |
|------------------|-------------------|
| RTAB-Map SLAM | `rtabmap_slam.launch.py` |
| RTAB-Map Localization | `rtabmap_loc.launch.py` |
| Depth → LaserScan | `depth_to_scan.launch.py` |

It never needs to know about `include/` files or which method is being used internally.

---

## 📷 Camera Configuration

| Camera | Position (m) | Orientation | Height | Best Use |
|--------|-------------|-------------|--------|----------|
| D435i | (0.045, 0, 1.773) | 60° downward, faces front | 1.773 m | RTAB-Map SLAM (rich visual features) |
| D455 | (-0.525, 0, 0.429) | Horizontal, faces rear | 0.429 m | Obstacle detection (`/scan`) |

### Critical notes

- **D435i floor issue:** Tilted 60° downward at 1.773 m — its depth-to-scan output primarily detects the floor, not obstacles. The `depthimage_to_laserscan` method clips floor returns with `range_min=1.1 m` (creating a blind zone). The `pointcloud_to_laserscan` method filters in world-space Z and has no blind zone.

- **frame_id must be `base_footprint`:** Gazebo publishes odometry as `odom → base_footprint`. All RTAB-Map launch files use `frame_id: base_footprint`. Do **not** change to `base_link` — RTAB-Map will silently fail TF lookups and never publish `map → odom`.

- **use_sim_time is mandatory for Gazebo:** Gazebo timestamps are ~1000 s while wall time is ~1.77 billion seconds. Without `use_sim_time:=true`, the message filter drops every RGBD frame silently and the `map` frame never appears.

---

## 🔀 Two Depth-to-Scan Methods: dimg vs pc2scan

Every depth-to-scan conversion goes through one router launch file with two methods:

```bash
# Gazebo (fast — recommended for simulation)
ros2 launch vf_robot_slam depth_to_scan.launch.py method:=dimg camera:=dual

# Real robot (accurate — recommended for production)
ros2 launch vf_robot_slam depth_to_scan.launch.py method:=pc2scan camera:=dual
```

### `method:=dimg` — depthimage_to_laserscan

```
Depth IMAGE (2D) ──► collapse centre rows ──► LaserScan
```

- Processes cheap 2D depth image — runs at full 15–30 Hz even in Gazebo
- Scan plane locked to camera optical axis direction
- **D435i limitation:** Floor hits scan at ~1.024 m → `range_min=1.1 m` clips it, but obstacles < 1.1 m are invisible
- **D455:** Works perfectly — horizontal at 0.429 m, scan at correct obstacle height
- **Best for:** Gazebo simulation, D455-only setups

### `method:=pc2scan` — pointcloud_to_laserscan

```
PointCloud2 (3D) ──► TF transform to base_footprint ──► Z height filter ──► LaserScan
```

- Transforms every 3D point into `base_footprint` frame (Z=0 = ground)
- Filters by height: keeps Z ∈ [0.05 m, 10.0 m] — floor excluded regardless of camera tilt
- **D435i:** No floor issue — world-space Z filter handles 60° tilt perfectly
- **Performance:** Real hardware at 15–30 Hz. Gazebo only ~2–3 Hz (CPU bottleneck)
- **Best for:** Real robot, D435i obstacle detection without blind zone

### Comparison

| Feature | `dimg` | `pc2scan` |
|---------|--------|-----------|
| Speed in Gazebo | 15–30 Hz ✅ | ~2–3 Hz ❌ |
| Speed on real robot | 15–30 Hz ✅ | 15–30 Hz ✅ |
| D435i floor handling | `range_min=1.1 m` hack (blind zone) | Height filter (no blind zone) ✅ |
| D455 floor handling | No issue ✅ | No issue ✅ |
| Dependencies | `depthimage_to_laserscan` | `pointcloud_to_laserscan` |

---

## 🔗 Dual Camera Scan Merging

In dual camera mode (`camera:=dual`), both cameras produce individual scans. These need to be combined into a single `/scan` topic for AMCL and SLAM Toolbox.

```
depth_to_scan.launch.py  camera:=dual
│
├── ALWAYS creates:
│     /scan_d435i   (front arc, ~87°)
│     /scan_d455    (rear arc, ~87°)
│
├── IF ira_laser_tools installed:
│     merger combines both → /scan   (~174° coverage)
│
└── IF ira_laser_tools NOT installed:
      relay copies /scan_d455 → /scan  (~87° rear only, zero CPU overhead)
```

**`/scan` always exists** regardless of whether `ira_laser_tools` is installed. AMCL, SLAM Toolbox, and Nav2 always have a scan topic to work with.

### Why `topic_tools relay` instead of a duplicate node?

Previous versions created a **second** `pointcloud_to_laserscan` node to re-process the same D455 point cloud. The relay node just forwards already-computed messages with near-zero CPU usage.

### Installing ira_laser_tools (optional)

```bash
cd ~/cogni-nav-x0/src
git clone https://github.com/iralabdisco/ira_laser_tools.git -b ros2
cd ~/cogni-nav-x0
colcon build --packages-select ira_laser_tools
```

**Alternative:** Skip the merger entirely and configure Nav2 costmaps with multiple observation sources (see Nav2 Costmap Configuration section).

---

## 🚀 Launch Files

### Top-Level Launches (what users and navigation call)

| Launch File | Purpose | Modes |
|-------------|---------|-------|
| `rtabmap_slam.launch.py` | Build a new map with RTAB-Map visual SLAM | Mode 1 |
| `rtabmap_loc.launch.py` | Localize in a previously built map | Mode 2 |
| `depth_to_scan.launch.py` | Convert depth to LaserScan (router) | Modes 1–4 |

### Include Launches (reusable building blocks — not called directly)

| Launch File | Purpose | Used By |
|-------------|---------|---------|
| `include/rgbd_sync.launch.py` | Spawns `rtabmap_sync/rgbd_sync` nodes for selected camera | `rtabmap_slam`, `rtabmap_loc` |
| `include/depth_to_scan_dimg.launch.py` | depthimage_to_laserscan nodes | `depth_to_scan` (method:=dimg) |
| `include/depth_to_scan_pc2scan.launch.py` | pointcloud_to_laserscan nodes | `depth_to_scan` (method:=pc2scan) |

### `rtabmap_slam.launch.py`

Builds a new map. Publishes `map → odom` TF and `/map` topic.

```bash
# New map with both cameras (default)
ros2 launch vf_robot_slam rtabmap_slam.launch.py camera:=dual map_name:=my_office

# Single camera
ros2 launch vf_robot_slam rtabmap_slam.launch.py camera:=d455 map_name:=my_office

# Continue existing map
ros2 launch vf_robot_slam rtabmap_slam.launch.py camera:=dual map_name:=my_office new_map:=false

# Real robot
ros2 launch vf_robot_slam rtabmap_slam.launch.py camera:=dual map_name:=my_office use_sim_time:=false
```

The launch file automatically:
1. Creates `~/cogni-nav-x0/maps/my_office/`
2. Starts RTAB-Map — `map → odom` TF appears after ~3–10 seconds
3. Saves `my_office.db` on shutdown (Ctrl+C)

Save 2D map for AMCL/Nav2 while SLAM is running:
```bash
ros2 run nav2_map_server map_saver_cli -f ~/cogni-nav-x0/maps/my_office/my_office
```

### `rtabmap_loc.launch.py`

Localizes within an existing map. Requires `.db` file from a prior SLAM session.

```bash
ros2 launch vf_robot_slam rtabmap_loc.launch.py camera:=dual map_name:=my_office
```

The launch file checks that the `.db` file exists and prints an error with instructions if missing.

### `depth_to_scan.launch.py`

Router — single entry point for all depth-to-scan conversion.

```bash
# Gazebo (fast)
ros2 launch vf_robot_slam depth_to_scan.launch.py method:=dimg camera:=dual

# Real robot (accurate)
ros2 launch vf_robot_slam depth_to_scan.launch.py method:=pc2scan camera:=dual

# Single camera
ros2 launch vf_robot_slam depth_to_scan.launch.py method:=dimg camera:=d455

# Dual without merging
ros2 launch vf_robot_slam depth_to_scan.launch.py camera:=dual merge_scans:=false
```

---

## 🗺️ Operating Modes (4 Modes)

This package supports 4 mutually exclusive operating modes, designed for integration with `vf_robot_navigation`:

| Mode | This package provides | `vf_robot_navigation` provides |
|------|----------------------|-------------------------------|
| **Mode 1:** RTAB-Map SLAM | `/map`, `map→odom` TF, `/scan` | Planners, controllers, costmaps |
| **Mode 2:** RTAB-Map Loc | `/map`, `map→odom` TF, `/scan` | Planners, controllers, costmaps |
| **Mode 3:** AMCL | `/scan` only | `map_server`, AMCL, planners |
| **Mode 4:** SLAM Toolbox | `/scan` only | SLAM Toolbox, planners |

**Key constraint:** Each mode has exactly ONE `/map` publisher and ONE `map→odom` TF publisher to prevent conflicts.

### Mode 1: RTAB-Map SLAM + Nav2

```bash
# Terminal 1: Gazebo
ros2 launch vf_robot_gazebo vf_my_world_xacro.launch.py

# Terminal 2: RTAB-Map SLAM
ros2 launch vf_robot_slam rtabmap_slam.launch.py camera:=dual map_name:=my_office

# Terminal 3: Depth to LaserScan
ros2 launch vf_robot_slam depth_to_scan.launch.py camera:=dual

# Terminal 4: Nav2 (future)
ros2 launch vf_robot_navigation bringup_rtabmap_slam.launch.py
```

### Mode 2: RTAB-Map Localization + Nav2

```bash
# Terminal 1: Gazebo
ros2 launch vf_robot_gazebo vf_my_world_xacro.launch.py

# Terminal 2: RTAB-Map Localization
ros2 launch vf_robot_slam rtabmap_loc.launch.py camera:=dual map_name:=my_office

# Terminal 3: Depth to LaserScan
ros2 launch vf_robot_slam depth_to_scan.launch.py camera:=dual

# Terminal 4: Nav2 (future)
ros2 launch vf_robot_navigation bringup_rtabmap_loc.launch.py
```

### Mode 3: AMCL + Nav2

```bash
# Terminal 1: Gazebo
ros2 launch vf_robot_gazebo vf_my_world_xacro.launch.py

# Terminal 2: Depth to LaserScan only
ros2 launch vf_robot_slam depth_to_scan.launch.py camera:=dual

# Terminal 3: Nav2 with AMCL (future)
ros2 launch vf_robot_navigation bringup_amcl.launch.py \
    map:=~/cogni-nav-x0/maps/my_office/my_office.yaml
```

### Mode 4: SLAM Toolbox + Nav2

```bash
# Terminal 1: Gazebo
ros2 launch vf_robot_gazebo vf_my_world_xacro.launch.py

# Terminal 2: Depth to LaserScan only
ros2 launch vf_robot_slam depth_to_scan.launch.py camera:=dual

# Terminal 3: Nav2 with SLAM Toolbox (future)
ros2 launch vf_robot_navigation bringup_slam_toolbox.launch.py
```

---

## 📦 Map Storage & Workflow

### Storage structure

Maps are stored **outside** the ROS package at workspace level:

```
~/cogni-nav-x0/
├── src/
│   ├── vf_robot_slam/               # Package source (no maps here)
│   └── vf_robot_navigation/
│
└── maps/                            # Auto-created by launch files
    ├── my_office/
    │   ├── my_office.db             # RTAB-Map database (auto-saved on Ctrl+C)
    │   ├── my_office.pgm            # 2D occupancy grid (save manually)
    │   └── my_office.yaml           # Map metadata for AMCL/Nav2 (save manually)
    └── warehouse/
        ├── warehouse.db
        ├── warehouse.pgm
        └── warehouse.yaml
```

### Map file reference

| File | Format | Typical Size | Used By | Contains |
|------|--------|-------------|---------|----------|
| `*.db` | SQLite | 10–500+ MB | RTAB-Map localization | 3D map, RGB images, depth, visual features, pose graph |
| `*.pgm` | Grayscale image | ~100 KB | AMCL, Nav2 `map_server` | 2D occupancy grid |
| `*.yaml` | YAML | ~200 B | AMCL, Nav2 `map_server` | Resolution, origin, path to `.pgm` |

### Complete SLAM workflow (step by step)

**Step 1:** Start Gazebo
```bash
ros2 launch vf_robot_gazebo vf_my_world_xacro.launch.py
```

**Step 2:** Start RTAB-Map SLAM
```bash
ros2 launch vf_robot_slam rtabmap_slam.launch.py camera:=dual map_name:=my_office
```

**Step 3:** Verify SLAM is initializing (~10 seconds)
```bash
ros2 run tf2_ros tf2_echo map odom
# Should show real translation values, not all zeros
```

**Step 4:** Drive the robot to build the map
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
# Drive slowly, cover all areas, return to start for loop closure
```

**Step 5A:** Save 2D map while SLAM is running
```bash
ros2 run nav2_map_server map_saver_cli -f ~/cogni-nav-x0/maps/my_office/my_office
```

**Step 5B:** Stop SLAM — `.db` saves automatically on Ctrl+C

**Step 6:** Export 2D map from database (if you skipped Step 5A)
```bash
rtabmap-databaseViewer ~/cogni-nav-x0/maps/my_office/my_office.db
# File → Export 2D Grid Map → save as .pgm
```

**Step 7:** Use the map for navigation
```bash
# Option A: RTAB-Map Localization (uses .db)
ros2 launch vf_robot_slam rtabmap_loc.launch.py camera:=dual map_name:=my_office

# Option B: AMCL (uses .pgm/.yaml)
ros2 launch vf_robot_navigation bringup_amcl.launch.py \
    map:=~/cogni-nav-x0/maps/my_office/my_office.yaml
```

---

## ⚙️ Launch Arguments Reference

### `rtabmap_slam.launch.py`

| Argument | Values | Default | Description |
|----------|--------|---------|-------------|
| `camera` | `d435i`, `d455`, `dual` | `dual` | Camera configuration |
| `map_name` | any string | `default_map` | Name for the map folder and `.db` file |
| `maps_dir` | path | `~/cogni-nav-x0/maps` | Base directory for all maps |
| `new_map` | `true`, `false` | `true` | Delete existing `.db` (true) or continue it (false) |
| `rviz` | `true`, `false` | `true` | Launch RViz |
| `use_sim_time` | `true`, `false` | `true` | Gazebo sim time (true) or wall time (false) |

### `rtabmap_loc.launch.py`

| Argument | Values | Default | Description |
|----------|--------|---------|-------------|
| `camera` | `d435i`, `d455`, `dual` | `dual` | Camera configuration |
| `map_name` | any string | *(required)* | Name of the map to load |
| `maps_dir` | path | `~/cogni-nav-x0/maps` | Base directory for maps |
| `rviz` | `true`, `false` | `true` | Launch RViz |
| `use_sim_time` | `true`, `false` | `true` | Gazebo sim time (true) or wall time (false) |

### `depth_to_scan.launch.py`

| Argument | Values | Default | Description |
|----------|--------|---------|-------------|
| `method` | `dimg`, `pc2scan` | `dimg` | Conversion method (see comparison above) |
| `camera` | `d435i`, `d455`, `dual` | `dual` | Camera configuration |
| `merge_scans` | `true`, `false` | `true` | Merge dual scans into `/scan` |
| `use_sim_time` | `true`, `false` | `true` | Gazebo sim time (true) or wall time (false) |

---

## 📡 Topics & TF Published

### RTAB-Map SLAM / Localization

| Topic | Type | Description |
|-------|------|-------------|
| `/map` | `nav_msgs/OccupancyGrid` | 2D occupancy grid |
| `/rtabmap/cloud_map` | `sensor_msgs/PointCloud2` | 3D point cloud map |
| `/rtabmap/mapPath` | `nav_msgs/Path` | Robot trajectory |
| `/rtabmap/info` | `rtabmap_msgs/Info` | Per-frame stats (loop closures, landmarks) |

### Depth to LaserScan

| Topic | Type | Published when |
|-------|------|----------------|
| `/scan` | `sensor_msgs/LaserScan` | Single camera, or dual with merging/relay |
| `/scan_d435i` | `sensor_msgs/LaserScan` | Dual mode only |
| `/scan_d455` | `sensor_msgs/LaserScan` | Dual mode only |

### TF transforms

| Transform | Publisher | Active in |
|-----------|-----------|-----------|
| `map → odom` | RTAB-Map | Modes 1 and 2 |
| `odom → base_footprint` | Gazebo / robot_localization | Always |
| `base_footprint → base_link` | `robot_state_publisher` | Always (static) |

### TF chain (full)

```
[slam/amcl]          [gazebo]                [robot_state_publisher]
     │                   │                            │
     ▼                   ▼                            ▼
   map ──────────► odom ──────────► base_footprint ──► base_link
                                                         ├── camera_d435i_link
                                                         │     ├── color_optical_frame
                                                         │     ├── depth_optical_frame
                                                         │     └── imu_frame
                                                         ├── camera_d455_link
                                                         │     ├── color_optical_frame
                                                         │     ├── depth_optical_frame
                                                         │     └── imu_frame
                                                         ├── camera_fisheye_*_link
                                                         ├── ultrasonic_*_link
                                                         └── wheel_*_link
```

---

## 🔧 Nav2 Costmap Configuration

For dual camera mode without `ira_laser_tools`, configure Nav2 to consume both scan topics as separate observation sources. This avoids the merger entirely for costmaps (AMCL still requires a single `/scan`):

```yaml
# In nav2_params.yaml
local_costmap:
  local_costmap:
    ros__parameters:
      plugins: ["obstacle_layer", "inflation_layer"]
      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        observation_sources: scan_d435i scan_d455
        scan_d435i:
          topic: /scan_d435i
          sensor_frame: camera_d435i_depth_optical_frame
          data_type: "LaserScan"
          clearing: true
          marking: true
          max_obstacle_height: 2.0
          min_obstacle_height: 0.0
        scan_d455:
          topic: /scan_d455
          sensor_frame: camera_d455_depth_optical_frame
          data_type: "LaserScan"
          clearing: true
          marking: true
          max_obstacle_height: 2.0
          min_obstacle_height: 0.0
```

---

## 📖 All Commands Reference

### Build

```bash
cd ~/cogni-nav-x0
colcon build --packages-select vf_robot_slam --symlink-install
source install/setup.bash
```

### SLAM

```bash
# New map (Gazebo)
ros2 launch vf_robot_slam rtabmap_slam.launch.py camera:=dual map_name:=my_office

# Continue existing map
ros2 launch vf_robot_slam rtabmap_slam.launch.py camera:=dual map_name:=my_office new_map:=false

# Real robot
ros2 launch vf_robot_slam rtabmap_slam.launch.py camera:=dual map_name:=my_office use_sim_time:=false

# Save 2D map while running
ros2 run nav2_map_server map_saver_cli -f ~/cogni-nav-x0/maps/my_office/my_office
```

### Localization

```bash
ros2 launch vf_robot_slam rtabmap_loc.launch.py camera:=dual map_name:=my_office
ros2 launch vf_robot_slam rtabmap_loc.launch.py camera:=d455 map_name:=my_office
```

### Depth to LaserScan

```bash
# Gazebo (depthimage method — fast)
ros2 launch vf_robot_slam depth_to_scan.launch.py method:=dimg camera:=dual

# Real robot (pointcloud method — accurate)
ros2 launch vf_robot_slam depth_to_scan.launch.py method:=pc2scan camera:=dual

# Single camera
ros2 launch vf_robot_slam depth_to_scan.launch.py method:=dimg camera:=d455

# Dual without merging
ros2 launch vf_robot_slam depth_to_scan.launch.py camera:=dual merge_scans:=false
```

### Verify SLAM is working

```bash
# Map frame should appear within ~10 seconds
ros2 run tf2_ros tf2_echo map odom

# RTAB-Map processing stats
ros2 topic echo /rtabmap/info --once

# RGBD sync rate (expect 15–30 Hz, not ~6 Hz)
ros2 topic hz /rgbd_image/d455
ros2 topic hz /rgbd_image/d435i

# Scan topics
ros2 topic hz /scan
ros2 topic hz /scan_d435i
ros2 topic hz /scan_d455
```

### TF inspection

```bash
ros2 run tf2_tools view_frames
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo base_link camera_d455_link
```

### Topic inspection

```bash
ros2 topic list | grep -E "scan|map|rtabmap|rgbd"
ros2 topic echo /odom --once | grep child_frame_id
ros2 topic echo /clock --once
```

---

## 🐛 Troubleshooting

### `map` frame never appears / `tf2_echo map odom` times out

This is the most common issue. Work through in order:

**1. Verify sim time is consistent.**
Gazebo sim time is ~1000 s. Wall time is ~1.77 billion seconds. Any mismatch causes every RGBD frame to be silently dropped.

```bash
ros2 topic echo /clock --once
ros2 topic echo /rgbd_image/d455 --once | grep sec
# Both sec values must be in the same range
```

Fix: ensure `use_sim_time:=true` in the launch command.

**2. Verify odometry frame ID.**
RTAB-Map uses `frame_id: base_footprint`. If odometry publishes a different `child_frame_id`, RTAB-Map silently fails.

```bash
ros2 topic echo /odom --once | grep child_frame_id
# Must print: child_frame_id: base_footprint
```

**3. Verify RGBD sync rate.**

```bash
ros2 topic hz /rgbd_image/d455
# Expected: 15–30 Hz. If ~6 Hz: approx_sync_max_interval issue.
```

**4. Check for message filter drops.**

```bash
ros2 topic echo /rosout 2>/dev/null | grep -i "dropping\|warn\|transform"
```

**5. Check RTAB-Map processing stats.**

```bash
ros2 topic echo /rtabmap/info --once
# If this times out: zero frames processed. Check topic remappings.
```

---

### Laser scan detecting floor (D435i)

Expected with `method:=dimg`. The D435i at 60° tilt produces a scan plane that intersects the floor at ~1.024 m. Solutions:

- Use `camera:=d455` or `camera:=dual` for obstacle detection
- Use `method:=pc2scan` which filters in world-space Z (no floor issue)
- With `method:=dimg`, `range_min=1.1 m` clips floor but creates a blind zone

---

### RTAB-Map not detecting loop closures

- Ensure sufficient texture and lighting
- Move slowly to avoid motion blur
- Return to a previously visited area
- Check `Vis/MinInliers` (currently 15) — lower for more permissive closure

---

### RGBD sync producing ~6 Hz instead of 15–30 Hz

Caused by `approx_sync_max_interval: 0.0` which defeats approximate sync. The launch files use `0.05` (50 ms tolerance). If you're loading parameters from external YAML files, check for overrides.

---

### `ira_laser_tools` not found warning

```
depth_to_scan: dual — ira_laser_tools NOT found
Fallback: /scan_d455 relayed to /scan (rear arc)
```

Not an error — the fallback works. Install `ira_laser_tools` from source for merged coverage, or use Nav2 multi-source costmap configuration with `merge_scans:=false`.

---

### Scan is empty with `method:=pc2scan`

The node performs a TF lookup at each cloud's timestamp. If TF is stale, the cloud is silently dropped. `transform_tolerance` is set to 0.5 s by default. If still empty:

```bash
# Check TF is being published
ros2 run tf2_ros tf2_echo base_footprint camera_d455_depth_optical_frame

# Check point cloud is arriving
ros2 topic hz /d455/depth/d455_depth/points
```

---

### TF errors for camera frames in RViz

Camera frames are static transforms published by `robot_state_publisher`. If missing, RSP is not running or the URDF is not loaded:

```bash
ros2 topic echo /robot_description --once | head -5
ros2 run tf2_tools view_frames
```

---

## 📄 Dependencies

### Required

```bash
sudo apt install ros-humble-rtabmap-ros \
                 ros-humble-depthimage-to-laserscan \
                 ros-humble-pointcloud-to-laserscan \
                 ros-humble-topic-tools
```

### Optional (for dual camera scan merging)

```bash
cd ~/cogni-nav-x0/src
git clone https://github.com/iralabdisco/ira_laser_tools.git -b ros2
cd ~/cogni-nav-x0
colcon build --packages-select ira_laser_tools
```

### Optional (for improved multi-camera SLAM)

```bash
# OpenGV improves multi-camera pose estimation
cd ~
git clone https://github.com/laurentkneip/opengv.git
cd opengv && mkdir build && cd build
cmake .. && make -j$(nproc) && sudo make install

# Rebuild rtabmap with OpenGV
cd ~/rtabmap/build
cmake .. -DWITH_OPENGV=ON
make -j$(nproc) && sudo make install

# Rebuild rtabmap_ros
cd ~/cogni-nav-x0
colcon build --symlink-install
```

---

## 📄 License

Apache 2.0

---

## 👤 Maintainer

**Pravin Oli** — olipravin18@gmail.com
Project: **cogni-nav-x0** | Package: `vf_robot_slam`
