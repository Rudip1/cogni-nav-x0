# vf_robot_slam

SLAM and localization package for the ViroFighter UVC-1 robot.

## Overview

This package provides SLAM (Simultaneous Localization and Mapping) capabilities for the ViroFighter UVC-1 robot using RTAB-Map with Intel RealSense depth cameras. It also provides depth-to-laserscan conversion for Nav2 compatibility.

## Robot Configuration

| Camera | Position | Orientation | Height | Best Use |
|--------|----------|-------------|--------|----------|
| D435i | (0.045, 0, 1.773) m | 60° downward, faces front | 1.773 m | RTAB-Map SLAM (visual features) |
| D455 | (-0.525, 0, 0.429) m | Horizontal, faces rear | 0.429 m | Obstacle detection (`/scan`) |

**Important notes:**

- The D435i is tilted 60° downward at 1.773 m — its depth-to-scan output primarily detects the floor, not obstacles. Use the D455 or dual mode for Nav2 obstacle avoidance.
- Gazebo publishes odometry as `odom → base_footprint`. The `frame_id` in all RTAB-Map launch files is set to `base_footprint` to match. Do **not** change it to `base_link` or RTAB-Map will silently fail to look up transforms and never publish the `map → odom` TF.
- When running in Gazebo, `use_sim_time:=true` (the default) is mandatory. Gazebo timestamps are small integers (~1000 s) while wall time is ~1.77 billion seconds. Without sim time, the message filter drops every RGBD frame silently and the `map` frame never appears.

## Package Structure

```
vf_robot_slam/
├── config/
│   ├── cameras/                # Camera topic/frame reference configs
│   │   ├── d435i.yaml
│   │   ├── d455.yaml
│   │   └── dual.yaml
│   ├── rtabmap/                # RTAB-Map parameter files (reference only)
│   │   ├── rtabmap_common.yaml
│   │   ├── rtabmap_slam.yaml
│   │   └── rtabmap_loc.yaml
│   ├── depth_to_scan/          # depthimage_to_laserscan parameters
│   │   ├── common.yaml
│   │   ├── d435i.yaml
│   │   └── d455.yaml
│   └── laser_merger/           # ira_laser_tools merger config
│       └── merger.yaml
├── launch/
│   ├── rtabmap_slam.launch.py  # SLAM mode (build new maps)
│   ├── rtabmap_loc.launch.py   # Localization mode (use existing maps)
│   └── depth_to_scan.launch.py # Depth image → LaserScan conversion
└── rviz/
    ├── rtabmap_slam.rviz
    └── rtabmap_loc.rviz

# Maps are stored OUTSIDE the package at workspace level:
~/cogni-nav-x0/maps/            # Auto-created by launch files
```

## Dependencies

```bash
# Required ROS packages
sudo apt install ros-humble-rtabmap-ros \
                 ros-humble-depthimage-to-laserscan

# Optional: dual camera scan merging (AMCL requires a single /scan topic)
# ira_laser_tools must be built from source — not available as a binary:
cd ~/cogni-nav-x0/src
git clone https://github.com/iralabdisco/ira_laser_tools.git -b ros2
cd ~/cogni-nav-x0
colcon build --packages-select ira_laser_tools
```

**Alternative to ira_laser_tools:** Configure Nav2 costmaps with multiple observation sources directly (see "Nav2 Costmap Configuration" section). This avoids the merger entirely for costmaps, though AMCL still requires a single `/scan`.

## Usage

### Prerequisites

Start the Gazebo simulation first (or bring up the real robot):

```bash
ros2 launch vf_robot_gazebo vf_my_world_xacro.launch.py
```

Confirm odometry is publishing before starting SLAM:

```bash
ros2 topic hz /odom
# Expected: ~100 Hz
# child_frame_id must be: base_footprint
```

### Optional: Build RTAB-Map with OpenGV (recommended for dual camera)

OpenGV improves multi-camera pose estimation. Without it, the launch files fall back to ICP-based registration (`Reg/Strategy: 1`, `Vis/EstimationType: 0`), which works but is less accurate.

```bash
# Install OpenGV
cd ~
git clone https://github.com/laurentkneip/opengv.git
cd opengv && mkdir build && cd build
cmake .. && make -j$(nproc) && sudo make install

# Rebuild rtabmap with OpenGV support
cd ~/rtabmap/build
cmake .. -DWITH_OPENGV=ON
make -j$(nproc) && sudo make install

# Rebuild rtabmap_ros
cd ~/cogni-nav-x0
colcon build --symlink-install
```

---

### SLAM Mode (Build a New Map)

```bash
# New map using both cameras (default, recommended)
ros2 launch vf_robot_slam rtabmap_slam.launch.py camera:=dual map_name:=my_office

# New map using only D455 (rear, horizontal — best for obstacle detection)
ros2 launch vf_robot_slam rtabmap_slam.launch.py camera:=d455 map_name:=my_office

# New map using only D435i (front, tilted — visual features only, scan sees floor)
ros2 launch vf_robot_slam rtabmap_slam.launch.py camera:=d435i map_name:=my_office

# Continue an existing map instead of starting fresh
ros2 launch vf_robot_slam rtabmap_slam.launch.py camera:=dual map_name:=my_office new_map:=false

# Real robot (disable sim time)
ros2 launch vf_robot_slam rtabmap_slam.launch.py camera:=dual map_name:=my_office use_sim_time:=false
```

The launch file automatically:
1. Creates `~/cogni-nav-x0/maps/my_office/`
2. Starts RTAB-Map and publishes `map → odom` TF once enough frames are processed (typically 3–10 seconds after launch)
3. Saves `my_office.db` automatically on shutdown (Ctrl+C)

Save the 2D occupancy grid for AMCL/Nav2 while SLAM is running:

```bash
ros2 run nav2_map_server map_saver_cli -f ~/cogni-nav-x0/maps/my_office/my_office
# Creates my_office.pgm and my_office.yaml
```

Drive the robot around slowly to build the map. Try to return to the starting point to trigger a loop closure.

#### Verifying SLAM is working

After launching, run in a separate terminal:

```bash
# The map frame should appear within ~10 seconds — real translation values, not all zeros
ros2 run tf2_ros tf2_echo map odom

# RTAB-Map should be processing frames at ~1 Hz
ros2 topic echo /rtabmap/info --once

# RGBD sync should be producing ~15–30 Hz (not ~6 Hz)
ros2 topic hz /rgbd_image/d455
ros2 topic hz /rgbd_image/d435i
```

If `map → odom` never appears, see the Troubleshooting section.

---

### Localization Mode (Navigate in an Existing Map)

Requires a `.db` file saved from a previous SLAM session.

```bash
# Localize using both cameras
ros2 launch vf_robot_slam rtabmap_loc.launch.py camera:=dual map_name:=my_office

# Localize using D455 only
ros2 launch vf_robot_slam rtabmap_loc.launch.py camera:=d455 map_name:=my_office

# Custom maps directory
ros2 launch vf_robot_slam rtabmap_loc.launch.py camera:=dual map_name:=my_office \
    maps_dir:=~/my_robot_maps

# Real robot (disable sim time)
ros2 launch vf_robot_slam rtabmap_loc.launch.py camera:=dual map_name:=my_office \
    use_sim_time:=false
```

```bash
# Terminal 1 — Gazebo
ros2 launch vf_robot_gazebo vf_my_world_xacro.launch.py

# Terminal 2 — Localization
ros2 launch vf_robot_slam rtabmap_loc.launch.py camera:=dual map_name:=my_office

# Terminal 3 — Verify map frame appears
ros2 run tf2_ros tf2_echo map odom
```

The launch file checks that the `.db` file exists before starting and prints an error with instructions if it is missing.

---

### Depth to LaserScan Only

Converts depth images to `sensor_msgs/LaserScan` for Nav2 compatibility without running RTAB-Map. Use this alongside SLAM Toolbox or AMCL.

```bash
# Single camera — D455 recommended (horizontal at 0.429 m, good for obstacles)
ros2 launch vf_robot_slam depth_to_scan.launch.py camera:=d455

# Single camera — D435i (tilted 60° down, scan detects floor more than obstacles)
ros2 launch vf_robot_slam depth_to_scan.launch.py camera:=d435i

# Dual cameras → /scan_d435i + /scan_d455 + merged /scan (requires ira_laser_tools)
ros2 launch vf_robot_slam depth_to_scan.launch.py camera:=dual

# Dual cameras without merging (configure Nav2 costmap with multiple sources instead)
ros2 launch vf_robot_slam depth_to_scan.launch.py camera:=dual merge_scans:=false
```

---

## Launch Arguments

### `rtabmap_slam.launch.py`

| Argument | Values | Default | Description |
|----------|--------|---------|-------------|
| `camera` | `d435i`, `d455`, `dual` | `dual` | Camera configuration |
| `map_name` | any string | `default_map` | Name for the map folder and `.db` file |
| `maps_dir` | path | `~/cogni-nav-x0/maps` | Base directory for all maps |
| `new_map` | `true`, `false` | `true` | Delete existing `.db` (true) or continue it (false) |
| `rviz` | `true`, `false` | `true` | Launch RViz |
| `use_sim_time` | `true`, `false` | `true` | Use Gazebo sim time (true) or wall time for real robot (false) |

### `rtabmap_loc.launch.py`

| Argument | Values | Default | Description |
|----------|--------|---------|-------------|
| `camera` | `d435i`, `d455`, `dual` | `dual` | Camera configuration |
| `map_name` | any string | *(required)* | Name of the map to load |
| `maps_dir` | path | `~/cogni-nav-x0/maps` | Base directory for maps |
| `rviz` | `true`, `false` | `true` | Launch RViz |

### `depth_to_scan.launch.py`

| Argument | Values | Default | Description |
|----------|--------|---------|-------------|
| `camera` | `d435i`, `d455`, `dual` | `dual` | Camera configuration |
| `merge_scans` | `true`, `false` | `true` | Merge dual scans into `/scan` (requires `ira_laser_tools`) |

---

## Map Storage Structure

Maps are stored outside the ROS package at workspace level:

```
~/cogni-nav-x0/
├── src/
│   ├── vf_robot_slam/               # Package source (no maps stored here)
│   └── vf_robot_navigation/
│
└── maps/                            # Auto-created by launch files
    ├── my_office/
    │   ├── my_office.db             # RTAB-Map database (auto-saved on Ctrl+C)
    │   ├── my_office.pgm            # 2D occupancy grid image (save manually)
    │   └── my_office.yaml           # Map metadata for AMCL/Nav2 (save manually)
    ├── warehouse/
    │   ├── warehouse.db
    │   ├── warehouse.pgm
    │   └── warehouse.yaml
    └── gazebo_test/
        └── gazebo_test.db
```

### Map file reference

| File | Format | Typical Size | Used By | Contains |
|------|--------|-------------|---------|----------|
| `*.db` | SQLite | 10–500+ MB | RTAB-Map localization | 3D map, RGB images, depth, visual features, pose graph |
| `*.pgm` | Grayscale image | ~100 KB | AMCL, Nav2 map_server | 2D occupancy grid |
| `*.yaml` | YAML | ~200 B | AMCL, Nav2 map_server | Resolution, origin, and path to `.pgm` |

---

## Operating Modes (Integration with vf_robot_navigation)

| Mode | What this package provides | What vf_robot_navigation provides |
|------|---------------------------|-----------------------------------|
| **Mode 1:** RTAB-Map SLAM | `/map`, `map→odom` TF, `/scan` | Planners, controllers |
| **Mode 2:** RTAB-Map Localization | `/map`, `map→odom` TF, `/scan` | Planners, controllers |
| **Mode 3:** AMCL | `/scan` only | `map_server`, AMCL, planners |
| **Mode 4:** SLAM Toolbox | `/scan` only | SLAM Toolbox, planners |

### Mode 1: RTAB-Map SLAM + Nav2

```bash
# Terminal 1: Gazebo
ros2 launch vf_robot_gazebo vf_my_world_xacro.launch.py

# Terminal 2: RTAB-Map SLAM
ros2 launch vf_robot_slam rtabmap_slam.launch.py camera:=dual map_name:=my_office

# Terminal 3: Depth to LaserScan (for Nav2 costmap)
ros2 launch vf_robot_slam depth_to_scan.launch.py camera:=dual

# Terminal 4: Nav2
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

# Terminal 4: Nav2
ros2 launch vf_robot_navigation bringup_rtabmap_loc.launch.py
```

### Mode 3: AMCL + Nav2

```bash
# Terminal 1: Gazebo
ros2 launch vf_robot_gazebo vf_my_world_xacro.launch.py

# Terminal 2: Depth to LaserScan only (AMCL needs /scan)
ros2 launch vf_robot_slam depth_to_scan.launch.py camera:=dual

# Terminal 3: Nav2 with AMCL and map_server
ros2 launch vf_robot_navigation bringup_amcl.launch.py \
    map:=~/cogni-nav-x0/maps/my_office/my_office.yaml
```

### Mode 4: SLAM Toolbox + Nav2

```bash
# Terminal 1: Gazebo
ros2 launch vf_robot_gazebo vf_my_world_xacro.launch.py

# Terminal 2: Depth to LaserScan only (SLAM Toolbox needs /scan)
ros2 launch vf_robot_slam depth_to_scan.launch.py camera:=dual

# Terminal 3: Nav2 with SLAM Toolbox
ros2 launch vf_robot_navigation bringup_slam_toolbox.launch.py
```

---

## Complete SLAM Workflow (Step by Step)

### Step 1: Start Gazebo

```bash
ros2 launch vf_robot_gazebo vf_my_world_xacro.launch.py
```

### Step 2: Start RTAB-Map SLAM

```bash
ros2 launch vf_robot_slam rtabmap_slam.launch.py camera:=dual map_name:=my_office
```

### Step 3: Verify SLAM is initializing

In a new terminal, watch for the `map` frame to appear (should happen within ~10 seconds):

```bash
ros2 run tf2_ros tf2_echo map odom
```

You should see real translation values instead of all zeros. If `map` never appears after 30 seconds, see Troubleshooting.

### Step 4: Drive the robot to build the map

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Drive slowly and methodically. Cover all areas. Try to return to your starting position — this triggers a loop closure which dramatically improves map accuracy.

### Step 5A: Save the 2D map while SLAM is running

```bash
ros2 run nav2_map_server map_saver_cli \
    -f ~/cogni-nav-x0/maps/my_office/my_office
# Creates my_office.pgm and my_office.yaml
```

### Step 5B: Stop SLAM — the database saves automatically

Press Ctrl+C in the SLAM terminal. The `.db` file is saved automatically.

### Step 6: Export 2D map from the database (if you skipped Step 5A)

```bash
# Open the database viewer GUI
rtabmap-databaseViewer ~/cogni-nav-x0/maps/my_office/my_office.db
# File → Export 2D Grid Map → save as .pgm
```

Or use the ROS service while RTAB-Map is running:

```bash
ros2 service call /rtabmap/publish_map std_srvs/srv/Empty
ros2 run nav2_map_server map_saver_cli -f ~/cogni-nav-x0/maps/my_office/my_office
```

### Step 7: Use the map for navigation

```bash
# Option A: RTAB-Map Localization (uses .db — visual features, most robust)
ros2 launch vf_robot_slam rtabmap_loc.launch.py camera:=dual map_name:=my_office

# Option B: AMCL (uses .pgm/.yaml — standard Nav2 stack)
ros2 launch vf_robot_navigation bringup_amcl.launch.py \
    map:=~/cogni-nav-x0/maps/my_office/my_office.yaml
```

---

## Topics Published

### RTAB-Map SLAM / Localization

| Topic | Type | Description |
|-------|------|-------------|
| `/map` | `nav_msgs/OccupancyGrid` | 2D occupancy grid |
| `/rtabmap/cloud_map` | `sensor_msgs/PointCloud2` | 3D point cloud map |
| `/rtabmap/mapPath` | `nav_msgs/Path` | Robot trajectory |
| `/rtabmap/info` | `rtabmap_msgs/Info` | Per-frame processing stats (loop closures, landmarks) |

### Depth to LaserScan

| Topic | Type | Published when |
|-------|------|----------------|
| `/scan` | `sensor_msgs/LaserScan` | Single camera mode, or dual with `merge_scans:=true` |
| `/scan_d435i` | `sensor_msgs/LaserScan` | Dual mode only |
| `/scan_d455` | `sensor_msgs/LaserScan` | Dual mode only |

### TF

| Transform | Publisher | Active in |
|-----------|-----------|-----------|
| `map → odom` | RTAB-Map | Modes 1 and 2 |
| `odom → base_footprint` | Gazebo / robot_localization | Always |
| `base_footprint → base_link` | robot_state_publisher | Always (static) |

---

## Nav2 Costmap Configuration

For dual camera mode without `ira_laser_tools`, configure Nav2 to consume both scan topics directly as separate observation sources. This avoids the merger for costmaps (AMCL still requires a single `/scan`):

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

## Troubleshooting

### `map` frame never appears in RViz / `tf2_echo map odom` times out

This is the most common issue. Work through these checks in order:

**1. Verify sim time is consistent across all nodes.**
Gazebo sim time is a small number (~1000 s). Wall time is ~1.77 billion seconds. If any node runs on wall time while sensor data carries sim timestamps, the message filter discards every frame silently.

```bash
ros2 topic echo /clock --once
ros2 topic echo /rgbd_image/d455 --once | grep sec
# Both sec values must be in the same range (both ~1000, not one ~1000 and one ~1775000000)
```

The fix: ensure `use_sim_time:=true` (the default) in the launch command, and confirm your Gazebo launch also sets `use_sim_time: true` on `robot_state_publisher`.

**2. Verify the odometry frame ID matches `frame_id` in the launch file.**
RTAB-Map is configured with `frame_id: base_footprint`. If your odometry publishes a different `child_frame_id`, RTAB-Map silently fails every TF lookup.

```bash
ros2 topic echo /odom --once | grep child_frame_id
# Must print: child_frame_id: base_footprint
```

**3. Verify RGBD sync is producing frames at a healthy rate.**

```bash
ros2 topic hz /rgbd_image/d455
ros2 topic hz /rgbd_image/d435i
# Expected: 15–30 Hz. If you see ~6 Hz, check approx_sync_max_interval and queue_size.
```

**4. Check for message filter drop warnings.**

```bash
ros2 topic echo /rosout 2>/dev/null | grep -i "dropping\|warn\|transform\|did not receive"
```

Repeated `dropping message: frame 'odom'` lines mean timestamps are mismatched (sim time issue). `Did not receive data since 5 seconds` means RTAB-Map is not getting RGBD input at all (topic remapping issue).

**5. Check RTAB-Map processing stats.**

```bash
ros2 topic echo /rtabmap/info --once
```

If this times out entirely, RTAB-Map has processed zero frames. If it prints but `loopClosureId` is always 0 and `landmarksCount` is very small, RTAB-Map is processing frames but failing to track features (common if the environment is texture-poor or the D435i is only seeing floor).

---

### Laser scan detecting floor (D435i)

This is expected. The D435i is mounted at 1.773 m and tilted 60° downward — its projected scan plane hits the floor at roughly 3 m range. Use `camera:=d455` or `camera:=dual` for obstacle detection. In dual mode the D455 provides the meaningful horizontal scan.

---

### RTAB-Map not detecting loop closures

- Ensure the environment has sufficient texture and lighting for visual feature extraction.
- Move slowly to avoid motion blur.
- Try returning to a previously visited area — loop closure requires recognizing a prior location.
- Check `Vis/MinInliers` (currently 15) — lowering it makes loop closure more permissive but less accurate.

---

### RGBD sync producing ~6 Hz instead of 15–30 Hz

This is caused by `approx_sync_max_interval: 0.0` combined with `approx_sync: true`. On some `rtabmap_sync` versions `0.0` is treated as "must be exactly simultaneous", which defeats approximate sync. The launch files use `0.05` (50 ms tolerance). If you are loading parameters from the YAML config files instead, check `config/rtabmap/rtabmap_common.yaml` for overrides.

---

### `ira_laser_tools` merger not found

```
WARNING: ira_laser_tools not found! Scan merging disabled.
```

Either install it from source (see Dependencies) or switch to Nav2 multi-source costmap configuration and use `merge_scans:=false`. AMCL still requires a single `/scan` topic regardless.

---

### TF errors for camera frames in RViz

All camera frames (`camera_d455_depth_optical_frame`, etc.) are static transforms published by `robot_state_publisher`. They appear in the TF tree with `most_recent_transform: 0.000` and `rate: 10000` — this is correct for static transforms. RViz errors about camera frames usually mean `robot_state_publisher` is not running or the URDF is not loaded.

```bash
ros2 topic echo /robot_description --once | head -5
ros2 run tf2_tools view_frames && cat frames.pdf
```

---

## License

Apache-2.0
