# `vf_robot_bringup`

> **ViroFighter UVC-1 Robot — Navigation Bringup Package**
> ROS 2 Humble · Nav2 · RTAB-Map · AMCL · SLAM Toolbox

[![ROS2 Humble](https://img.shields.io/badge/ROS2-Humble-blue?style=flat-square&logo=ros)](https://docs.ros.org/en/humble/)
[![Nav2](https://img.shields.io/badge/Nav2-Navigation-green?style=flat-square)](https://navigation.ros.org/)
[![License](https://img.shields.io/badge/License-Apache%202.0-green?style=flat-square)](LICENSE)

---

## 📋 Table of Contents

- [Overview](#-overview)
- [Quick Start](#-quick-start)
- [All 4 Localization Modes](#-all-4-localization-modes)
- [Package Structure](#-package-structure)
- [Arguments Reference](#-arguments-reference)
- [Architecture](#️-architecture)
- [Nav2 Parameters — Composed Config](#-nav2-parameters--composed-config)
- [Using with VFRobotController](#-using-with-vfrobotcontroller)
- [RViz Controls](#-rviz-controls)
- [Verification](#-verification)
- [Troubleshooting](#-troubleshooting)
- [License](#-license)

---

## 🌟 Overview

`vf_robot_bringup` is the **single entry point** for all ViroFighter navigation. One launch file with three key arguments — `robot:=`, `controller:=`, `localization:=` — it orchestrates SLAM/localization, depth-to-scan, Nav2, and RViz automatically.

**What it does:**

```
bringup_launch.py  robot:=virofighter  controller:=mppi  localization:=rtabmap_slam
       │
       ├── depth_to_scan  (from vf_robot_slam)     → /scan
       ├── SLAM / Localization  (localization-specific)  → /map + map→odom TF
       ├── Nav2 stack  (planners, controllers, BT)  → autonomous navigation
       └── RViz  (optional)                         → visualization + goal setting
```

**What it does NOT contain:**

| Responsibility | Lives in |
|---|---|
| URDF / meshes / sensor TF | `vf_robot_description` |
| Gazebo worlds / simulation | `vf_robot_gazebo` |
| RTAB-Map / depth-to-scan nodes | `vf_robot_slam` |
| Controller plugin C++ / training | `vf_robot_controller` |
| Map files (.db, .pgm, .yaml) | `~/cogni-nav-x0/maps/` |

---

## ⚡ Quick Start

### Prerequisites

```bash
sudo apt install ros-humble-navigation2 \
                 ros-humble-nav2-bringup \
                 ros-humble-slam-toolbox \
                 ros-humble-rtabmap-ros \
                 ros-humble-depthimage-to-laserscan
```

### Build

```bash
cd ~/cogni-nav-x0
colcon build --packages-select vf_robot_bringup --symlink-install
source install/setup.bash
```

### Launch (2 terminals)

```bash
# Terminal 1: Gazebo
ros2 launch vf_robot_gazebo vf_my_world_xacro.launch.py

# Terminal 2: Everything else
ros2 launch vf_robot_bringup bringup_launch.py \
    robot:=virofighter \
    controller:=mppi \
    localization:=rtabmap_slam \
    camera:=dual \
    map_name:=my_office
```

In RViz: click **Nav2 Goal** → click on the map → robot navigates while building the map.

---

## 🗺️ All 4 Localization Modes

### Mode 1 — RTAB-Map SLAM (build a new map)

```bash
# New map
ros2 launch vf_robot_bringup bringup_launch.py \
    robot:=virofighter \
    controller:=mppi \
    localization:=rtabmap_slam \
    camera:=dual \
    map_name:=my_office \
    new_map:=true

# Continue existing map
ros2 launch vf_robot_bringup bringup_launch.py \
    robot:=virofighter \
    controller:=mppi \
    localization:=rtabmap_slam \
    camera:=dual \
    map_name:=my_office \
    new_map:=false
```

When done: Ctrl+C saves `my_office/rtabmap.db` automatically. To export 2D map for AMCL:

```bash
ros2 run nav2_map_server map_saver_cli -f ~/cogni-nav-x0/maps/my_office/my_office
```

### Mode 2 — RTAB-Map Localization (navigate in existing map)

```bash
ros2 launch vf_robot_bringup bringup_launch.py \
    robot:=virofighter \
    controller:=mppi \
    localization:=rtabmap_loc \
    camera:=dual \
    map_name:=my_office
```

Requires `my_office/rtabmap.db` from a prior SLAM session.

### Mode 3 — AMCL (navigate in 2D map)

```bash
ros2 launch vf_robot_bringup bringup_launch.py \
    robot:=virofighter \
    controller:=mppi \
    localization:=amcl \
    camera:=dual \
    map:=$HOME/cogni-nav-x0/maps/my_office/my_office.yaml
```

Requires `.pgm` + `.yaml` files. In RViz, use **2D Pose Estimate** to set initial robot position.

### Mode 4 — SLAM Toolbox (laser-based SLAM)

```bash
ros2 launch vf_robot_bringup bringup_launch.py \
    robot:=virofighter \
    controller:=mppi \
    localization:=slam_toolbox \
    camera:=dual
```

Builds a map from `/scan` data. Good alternative when RTAB-Map visual SLAM struggles.

### Real Robot (any mode)

```bash
ros2 launch vf_robot_bringup bringup_launch.py \
    robot:=virofighter \
    controller:=mppi \
    localization:=rtabmap_loc \
    camera:=dual \
    map_name:=my_office \
    use_sim_time:=false
```

---

## 📁 Package Structure

```
vf_robot_bringup/
├── CMakeLists.txt
├── package.xml
├── setup.py
├── config/
│   ├── nav2/
│   │   ├── nav2_base.yaml              # Base Nav2 skeleton (composed at runtime)
│   │   ├── controllers/                # Per-controller YAML fragments
│   │   │   ├── vf_inference.yaml       # VF controller — meta-critic inference
│   │   │   ├── vf_collect.yaml         # VF controller — data collection
│   │   │   ├── vf_passive.yaml         # VF controller — imitation passthrough
│   │   │   ├── vf_fixed.yaml           # VF controller — fixed weights
│   │   │   ├── mppi.yaml               # Stock Nav2 MPPI
│   │   │   └── dwb.yaml                # Stock Nav2 DWB
│   │   └── localization/
│   │       ├── amcl.yaml               # AMCL params
│   │       └── slam_toolbox.yaml       # SLAM Toolbox params
│   └── robots/
│       ├── virofighter.yaml            # ViroFighter robot profile
│       └── turtlebot3_waffle.yaml      # TurtleBot3 robot profile
├── rviz/
│   └── vf_bringup.rviz                 # RViz with Nav2 panels
├── launch/
│   ├── bringup_launch.py               # THE entry point (all modes)
│   ├── navigation_launch.py            # Nav2 stack (controller, planner, BT, etc.)
│   ├── localization_launch.py          # AMCL + map_server (Mode 3)
│   ├── slam_launch.py                  # SLAM Toolbox (Mode 4)
│   └── rviz_launch.py                  # Standalone RViz
├── resource/
│   └── vf_robot_bringup
└── vf_robot_bringup/
    ├── __init__.py
    └── launch_utils/
        └── compose_params.py           # Merges nav2_base + controller + robot fragments
```

---

## ⚙️ Arguments Reference

### `bringup_launch.py`

| Argument | Values | Default | Description |
|---|---|---|---|
| `robot` | `virofighter`, `turtlebot3_waffle` | `virofighter` | Robot profile (footprint, velocity limits) |
| `controller` | `vf_inference`, `vf_fixed`, `vf_collect`, `vf_passive`, `mppi`, `dwb` | `mppi` | Nav2 local controller |
| `localization` | `rtabmap_slam`, `rtabmap_loc`, `amcl`, `slam_toolbox` | `rtabmap_slam` | Navigation/SLAM mode |
| `camera` | `d435i`, `d455`, `dual` | `dual` | Camera config for depth-to-scan |
| `scan_method` | `dimg`, `pc2scan` | `pc2scan` | Depth-to-scan conversion method |
| `merge_scans` | `true`, `false` | `true` | Merge dual scans into /scan |
| `map_name` | string | `default_map` | Map folder name (RTAB-Map modes) |
| `map` | path | `""` | Full path to .yaml map (AMCL mode only) |
| `maps_dir` | path | `~/cogni-nav-x0/maps` | Base maps directory |
| `new_map` | `true`, `false` | `true` | Fresh map or continue (RTAB-Map SLAM only) |
| `use_sim_time` | `true`, `false` | `true` | Simulation or real robot |
| `rviz` | `true`, `false` | `true` | Launch RViz |
| `rviz_config` | path | auto | Custom RViz config file path |
| `autostart_sidecar` | `true`, `false` | `false` | Auto-start ML sidecar (for unattended runs) |

> **Note**: The `params_file` argument does **not** exist. Nav2 parameters are composed
> automatically from per-robot and per-controller YAML fragments by `compose_params.py`.
> To customise params, edit the appropriate fragment in `config/nav2/`.

---

## 🏗️ Architecture

### What `bringup_launch.py` starts per mode

```
                           ┌─────────────────────────────────────┐
                           │        bringup_launch.py            │
                           │  robot:= controller:= localization:=│
                           └──────────┬──────────────────────────┘
                                      │
              ┌───────────────────────┼───────────────────────┐
              │                       │                       │
     ┌────────▼────────┐   ┌─────────▼─────────┐   ┌────────▼────────┐
     │  depth_to_scan  │   │  Localization      │   │  Nav2 stack     │
     │  (ALL modes)    │   │  (mode-specific)   │   │  (ALL modes)    │
     │                 │   │                    │   │                 │
     │  from           │   │  rtabmap_slam  OR  │   │  controller     │
     │  vf_robot_slam  │   │  rtabmap_loc   OR  │   │  planner        │
     │                 │   │  amcl+map_srv  OR  │   │  behaviors      │
     │  → /scan        │   │  slam_toolbox      │   │  bt_navigator   │
     └─────────────────┘   │                    │   │  vel_smoother   │
                           │  → /map            │   │  lifecycle_mgr  │
                           │  → map→odom TF     │   └─────────────────┘
                           └────────────────────┘
```

### Nav2 param composition

`compose_params.py` merges three YAML fragments at runtime:

```
nav2_base.yaml  +  config/robots/<robot>.yaml  +  config/nav2/controllers/<controller>.yaml
       └─────────────────────────────────────────────────────────┘
                       written to /tmp/nav2_<robot>_<controller>_<localization>_*.yaml
                       passed to navigation_launch.py as params_file
```

### Who publishes what

| Output | rtabmap_slam | rtabmap_loc | amcl | slam_toolbox |
|---|---|---|---|---|
| `/map` | RTAB-Map | RTAB-Map (.db) | map_server (.pgm) | SLAM Toolbox |
| `map→odom` TF | RTAB-Map | RTAB-Map | AMCL | SLAM Toolbox |
| `/scan` | depth_to_scan | depth_to_scan | depth_to_scan | depth_to_scan |
| `/odom` | Gazebo | Gazebo | Gazebo | Gazebo |
| `odom→base_footprint` | Gazebo | Gazebo | Gazebo | Gazebo |
| sensor TFs | robot_state_pub | robot_state_pub | robot_state_pub | robot_state_pub |

---

## 🔧 Nav2 Parameters — Composed Config

Nav2 parameters are **not** a single flat file. They are composed from fragments
at launch time by `compose_params.py`. The resulting file is written to
`/tmp/nav2_<robot>_<controller>_<localization>_<timestamp>.yaml`.

### Inspect the composed YAML at runtime

```bash
ls -lt /tmp/nav2_*.yaml | head -1   # most recent
cat /tmp/nav2_virofighter_mppi_rtabmap_loc_*.yaml | less
```

### Key defaults (virofighter robot profile)

| Parameter | Value | Why |
|---|---|---|
| `max_vel_x` | 0.3 m/s | UVC disinfection robot — not a racing bot |
| `max_vel_theta` | 1.0 rad/s | Conservative turning for stability |
| `footprint` | `[0.1,0.2], [0.1,-0.2], [-0.5,-0.2], [-0.5,0.2]` | ViroFighter body shape |
| `inflation_radius` | 0.55 m | Slightly larger than half robot width |
| `laser_max_range` | 6.0 m | Matches RealSense depth camera range |
| `base_frame_id` | `base_footprint` | Matches Gazebo diff drive output |

---

## 🎮 Using with VFRobotController

The VF controller plugin supports four modes controlled by `controller:=`:

| `controller:=` | Plugin mode | Python sidecar | Purpose |
|---|---|---|---|
| `vf_fixed` | Fixed YAML weights | None | Ablation baseline |
| `vf_inference` | Meta-critic NN weights | `meta_critic_inference_launch.py` | **Thesis main result** |
| `vf_collect` | Fixed + HDF5 logging | `meta_critic_collect_launch.py` | Data collection |
| `vf_passive` | Zero cmd_vel (passthrough) | `imitation_inference_launch.py` | Imitation baseline |

### Running VF Inference (example)

**Terminal 2 — bringup:**
```bash
ros2 launch vf_robot_bringup bringup_launch.py \
    robot:=virofighter \
    controller:=vf_inference \
    localization:=rtabmap_loc \
    camera:=dual \
    map_name:=my_office
```

**Terminal 3 — ML sidecar (requires conda dl env):**
```bash
conda activate dl
source ~/cogni-nav-x0/install/setup.bash
ros2 launch vf_robot_controller meta_critic_inference_launch.py
```

For the full experimental workflow see
[`README_quick_Launch.md`](README_quick_Launch.md).

---

## 🖱️ RViz Controls

| Button | What it does | Which modes |
|---|---|---|
| **2D Pose Estimate** | Sets initial robot pose → publishes `/initialpose` | AMCL mode (required) |
| **Nav2 Goal** | Sets navigation target → publishes `/goal_pose` | All modes |
| **Publish Point** | Click to publish a point (debugging) | All modes |

### Fixed Frame selection

| Mode | Fixed Frame |
|---|---|
| All modes (with map) | `map` (default in rviz config) |
| No map yet (early SLAM) | `odom` — switch to `map` once it appears |

---

## ✅ Verification

After launching, verify the full stack is working:

```bash
# 1. /map topic exists
ros2 topic hz /map

# 2. map→odom TF exists
ros2 run tf2_ros tf2_echo map odom

# 3. /scan is publishing
ros2 topic hz /scan

# 4. Nav2 nodes are active
ros2 lifecycle get /controller_server    # should be: active
ros2 lifecycle get /planner_server       # should be: active
ros2 lifecycle get /bt_navigator         # should be: active

# 5. Inspect the composed Nav2 yaml
ls -lt /tmp/nav2_*.yaml | head -1
```

---

## 🐛 Troubleshooting

### Nav2 nodes stuck in UNCONFIGURED state

**Cause:** `map→odom` TF not available yet. Lifecycle manager waits for the full TF chain.

**Fix:** Ensure your localization mode is running:

```bash
ros2 run tf2_ros tf2_echo map odom
# If this times out: SLAM/localization node hasn't initialised yet
# For RTAB-Map: wait ~10 seconds, check /rtabmap/info topic
```

### Robot doesn't move after setting Nav2 Goal

```bash
ros2 topic hz /plan           # planner output
ros2 topic hz /local_plan     # controller output
ros2 topic hz /cmd_vel        # final velocity to robot
```

In RViz, enable Local Costmap display — robot must not be inside an obstacle.

### AMCL: robot position jumps wildly

Initial pose not set. Click **2D Pose Estimate** in RViz and click the approximate robot position on the map.

### "map yaml file not found" in AMCL mode

Use the full path to the `.yaml` file:

```bash
ros2 launch vf_robot_bringup bringup_launch.py \
    robot:=virofighter controller:=mppi localization:=amcl \
    map:=$HOME/cogni-nav-x0/maps/my_office/my_office.yaml
```

### Controller plugin not found

```bash
colcon build --packages-select vf_robot_controller --symlink-install
source install/setup.bash
```

### Costmap shows no obstacles

```bash
ros2 topic hz /scan
ros2 topic echo /scan --once | grep -E "range_min|range_max|ranges"
```

---

## 📄 License

Apache 2.0

---

## 👤 Maintainer

**Pravin Oli** — olipravin18@gmail.com
Project: **cogni-nav-x0** | Package: `vf_robot_bringup`
