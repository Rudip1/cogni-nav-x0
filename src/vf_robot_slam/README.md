# vf_robot_slam

SLAM and localization package for the ViroFighter UVC-1 robot.

## Overview

This package provides SLAM (Simultaneous Localization and Mapping) capabilities for the ViroFighter UVC-1 robot using RTAB-Map with Intel RealSense depth cameras. It also provides depth-to-laserscan conversion for Nav2 compatibility.

## Robot Configuration

| Camera | Position | Orientation | Best Use |
|--------|----------|-------------|----------|
| D435i | (0.045, 0, 1.773)m | 60° down, faces front | RTAB-Map SLAM (visual features) |
| D455 | (-0.525, 0, 0.429)m | Horizontal, faces rear | Obstacle detection (/scan) |

**Note:** The D435i is tilted 60° downward, so its laser scan primarily detects the floor. For obstacle detection, D455 or dual camera mode is recommended.

## Package Structure

```
vf_robot_slam/
├── config/
│   ├── cameras/           # Camera-specific topic/frame configs
│   │   ├── d435i.yaml
│   │   ├── d455.yaml
│   │   └── dual.yaml
│   ├── rtabmap/           # RTAB-Map parameters
│   │   ├── rtabmap_common.yaml
│   │   ├── rtabmap_slam.yaml
│   │   └── rtabmap_loc.yaml
│   ├── depth_to_scan/     # depthimage_to_laserscan params
│   └── laser_merger/      # ira_laser_tools merger config
├── launch/
│   ├── rtabmap_slam.launch.py    # SLAM mode
│   ├── rtabmap_loc.launch.py     # Localization mode
│   └── depth_to_scan.launch.py   # Depth → LaserScan
└── rviz/                  # RViz configurations

# Maps are stored OUTSIDE the package:
~/cogni-nav-x0/maps/       # Auto-created by launch files
```

## Dependencies

```bash
# Required packages
sudo apt install ros-humble-rtabmap-ros \
                 ros-humble-depthimage-to-laserscan

# Optional: For dual camera scan merging (AMCL requires single /scan)
# ira_laser_tools must be built from source:
cd ~/cogni-nav-x0/src
git clone https://github.com/iralabdisco/ira_laser_tools.git -b ros2
cd ~/cogni-nav-x0
colcon build --packages-select ira_laser_tools
```

**Alternative to ira_laser_tools:** Configure Nav2 costmaps with multiple observation sources (see "Nav2 Costmap Configuration" section below).

## Usage

### Prerequisites

Start Gazebo simulation (or real robot):
```bash
ros2 launch vf_robot_gazebo vf_my_world_xacro.launch.py
```
```bash
ros2 launch vf_robot_gazebo <your gazebo.launch.py>
```
### For best multi-camera support (optional but recommended):
Build RTAB-Map with OpenGV:

```bash
# Install OpenGV
cd ~
git clone https://github.com/laurentkneip/opengv.git
cd opengv && mkdir build && cd build
cmake .. && make -j$(nproc) && sudo make install

# Rebuild rtabmap with OpenGV
cd ~/rtabmap/build
cmake .. -DWITH_OPENGV=ON
make -j$(nproc) && sudo make install

# Rebuild rtabmap_ros
cd ~/rtabmap_ros  # wherever your workspace is
colcon build --symlink-install

```

### SLAM Mode (Build a New Map)

```bash
# Build a map named "my_office" using both cameras
ros2 launch vf_robot_slam rtabmap_slam.launch.py camera:=dual map_name:=my_office

# Using only D455 (rear camera, recommended for obstacle detection)
ros2 launch vf_robot_slam rtabmap_slam.launch.py camera:=d455 map_name:=my_office

# Custom maps directory
ros2 launch vf_robot_slam rtabmap_slam.launch.py camera:=dual map_name:=my_office \
    maps_dir:=~/my_robot_maps
```
```bash
# NEW map (deletes existing my_office.db)
ros2 launch vf_robot_slam rtabmap_slam.launch.py camera:=dual map_name:=my_office new_map:=true

# CONTINUE existing map
ros2 launch vf_robot_slam rtabmap_slam.launch.py camera:=dual map_name:=my_office new_map:=false

# Save 2D map while running (in another terminal)
ros2 run nav2_map_server map_saver_cli -f ~/cogni-nav-x0/maps/my_office/my_office
```

This automatically:
1. Creates directory: `~/cogni-nav-x0/maps/my_office/`
2. On shutdown (Ctrl+C), saves ALL map files:
   - `my_office.db` - RTAB-Map database (for RTAB-Map localization)
   - `my_office.pgm` - 2D occupancy grid image (for AMCL/Nav2)
   - `my_office.yaml` - Map metadata (for AMCL/Nav2)

Drive the robot around to build the map. **All files are automatically saved when you stop the node (Ctrl+C).**

### Localization Mode (Use Existing Map)

```bash
# RTAB-Map localization (uses .db file)
ros2 launch vf_robot_slam rtabmap_loc.launch.py camera:=dual map_name:=my_office
```

### Depth to LaserScan Only

For Nav2 compatibility without running RTAB-Map:

```bash
# Single camera → /scan
ros2 launch vf_robot_slam depth_to_scan.launch.py camera:=d455

# Dual cameras → /scan_d435i + /scan_d455 + /scan (merged, if ira_laser_tools installed)
ros2 launch vf_robot_slam depth_to_scan.launch.py camera:=dual

# Dual cameras without merging (use Nav2 multi-source costmap instead)
ros2 launch vf_robot_slam depth_to_scan.launch.py camera:=dual merge_scans:=false
```

## Launch Arguments

| Argument | Values | Default | Description |
|----------|--------|---------|-------------|
| `camera` | `d435i`, `d455`, `dual` | `dual` | Camera configuration |
| `map_name` | any string | `default_map` | Name for map folder and files |
| `maps_dir` | path | `~/cogni-nav-x0/maps` | Base directory for all maps |
| `rviz` | `true`, `false` | `true` | Launch RViz |
| `merge_scans` | `true`, `false` | `true` | Merge dual scans (depth_to_scan only) |

## Map Storage Structure

Maps are stored **outside** the ROS packages at workspace level:

```
~/cogni-nav-x0/                      # Your workspace
├── src/
│   ├── vf_robot_slam/               # Package (no maps here)
│   └── vf_robot_navigation/
│
└── maps/                            # Maps folder (auto-created)
    ├── my_office/
    │   ├── my_office.db             # RTAB-Map database (auto-saved)
    │   ├── my_office.pgm            # 2D grid (you save manually)
    │   └── my_office.yaml           # 2D grid metadata
    ├── warehouse/
    │   ├── warehouse.db
    │   ├── warehouse.pgm
    │   └── warehouse.yaml
    └── gazebo_test/
        └── gazebo_test.db
```

## Operating Modes (Integration with vf_robot_navigation)

This package is designed to work with `vf_robot_navigation` in 4 modes:

| Mode | SLAM Package Provides | Navigation Package Provides |
|------|----------------------|----------------------------|
| **Mode 1:** RTAB-Map SLAM | `/map`, `map→odom` TF, `/scan` | Planners, controllers |
| **Mode 2:** RTAB-Map Loc | `/map`, `map→odom` TF, `/scan` | Planners, controllers |
| **Mode 3:** AMCL | `/scan` only | map_server, AMCL, planners |
| **Mode 4:** SLAM Toolbox | `/scan` only | SLAM Toolbox, planners |

### Mode 1: RTAB-Map SLAM + Nav2

```bash
# Terminal 1: Gazebo
ros2 launch vf_robot_gazebo gazebo.launch.py

# Terminal 2: SLAM + depth_to_scan
ros2 launch vf_robot_slam rtabmap_slam.launch.py camera:=dual

# Terminal 3: depth_to_scan (if not included above)
ros2 launch vf_robot_slam depth_to_scan.launch.py camera:=dual

# Terminal 4: Nav2 (future - from vf_robot_navigation)
ros2 launch vf_robot_navigation bringup_rtabmap_slam.launch.py
```

### Mode 2: RTAB-Map Localization + Nav2

```bash
ros2 launch vf_robot_slam rtabmap_loc.launch.py camera:=dual \
    database_path:=/path/to/map.db
ros2 launch vf_robot_slam depth_to_scan.launch.py camera:=dual
# + Nav2 navigation
```

### Mode 3: AMCL + Nav2 (Standard Nav2 Stack)

```bash
# Only depth_to_scan from this package
ros2 launch vf_robot_slam depth_to_scan.launch.py camera:=dual

# AMCL and map_server from vf_robot_navigation
ros2 launch vf_robot_navigation bringup_amcl.launch.py map:=/path/to/map.yaml
```

### Mode 4: SLAM Toolbox + Nav2

```bash
# Only depth_to_scan from this package
ros2 launch vf_robot_slam depth_to_scan.launch.py camera:=dual

# SLAM Toolbox from vf_robot_navigation
ros2 launch vf_robot_navigation bringup_slam_toolbox.launch.py
```

## Exporting 2D Maps for AMCL

RTAB-Map can export its 3D map to a 2D occupancy grid:

### Method 1: Using RTAB-Map GUI
```bash
rtabmap-databaseViewer ~/.ros/rtabmap.db
# File → Export 2D grid map → Save as .pgm
```

### Method 2: Using ROS Service (while RTAB-Map is running)
```bash
# Publish the grid map
ros2 service call /rtabmap/publish_map std_srvs/srv/Empty

# Then save using map_saver
ros2 run nav2_map_server map_saver_cli -f /path/to/my_map
```

## Complete Map Workflow

### Understanding Map Files

| File | Format | Size | Used By | Contains |
|------|--------|------|---------|----------|
| `rtabmap.db` | SQLite | 10-500+ MB | RTAB-Map Localization | Full 3D map, RGB images, depth, features, pose graph |
| `map.pgm` | Image | ~100 KB | AMCL, Nav2 | 2D occupancy grid (grayscale pixels) |
| `map.yaml` | YAML | ~200 B | AMCL, Nav2 | Map metadata (resolution, origin) |

### Step-by-Step Workflow

```bash
# ═══════════════════════════════════════════════════════════════════════════
# STEP 1: Start Gazebo Simulation
# ═══════════════════════════════════════════════════════════════════════════
ros2 launch vf_robot_gazebo gazebo.launch.py


# ═══════════════════════════════════════════════════════════════════════════
# STEP 2: Start RTAB-Map SLAM (in new terminal)
# ═══════════════════════════════════════════════════════════════════════════
ros2 launch vf_robot_slam rtabmap_slam.launch.py camera:=dual

# Database will be saved to: ~/.ros/rtabmap.db
# To use custom path:
ros2 launch vf_robot_slam rtabmap_slam.launch.py camera:=dual \
    database_path:=~/cogni-nav-x0/src/vf_robot_slam/maps/my_map.db


# ═══════════════════════════════════════════════════════════════════════════
# STEP 3: Drive the robot to build the map (in new terminal)
# ═══════════════════════════════════════════════════════════════════════════
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Use keys: i/k/j/l to move forward/backward/turn
# Drive slowly in a loop, covering all areas
# Try to return to starting point for loop closure


# ═══════════════════════════════════════════════════════════════════════════
# STEP 4A: Save 2D map WHILE RTAB-Map is running (in new terminal)
# ═══════════════════════════════════════════════════════════════════════════
# Make sure /map topic is being published
ros2 topic echo /map --once

# Save the 2D map
mkdir -p ~/cogni-nav-x0/src/vf_robot_slam/maps
ros2 run nav2_map_server map_saver_cli \
    -f ~/cogni-nav-x0/src/vf_robot_slam/maps/my_map

# This creates:
#   my_map.pgm  (grayscale image of occupancy grid)
#   my_map.yaml (metadata file)


# ═══════════════════════════════════════════════════════════════════════════
# STEP 4B: OR Stop RTAB-Map (Ctrl+C) - Database auto-saves
# ═══════════════════════════════════════════════════════════════════════════
# Press Ctrl+C in the RTAB-Map terminal
# Database is automatically saved to database_path (default: ~/.ros/rtabmap.db)


# ═══════════════════════════════════════════════════════════════════════════
# STEP 5: (Optional) Export 2D map from saved database
# ═══════════════════════════════════════════════════════════════════════════
# Use RTAB-Map Database Viewer GUI
rtabmap-databaseViewer ~/.ros/rtabmap.db

# In the GUI:
# 1. Wait for database to load
# 2. Click "View" → "Grid Map" to preview
# 3. Click "File" → "Export 2D Grid Map"
# 4. Save as .pgm file


# ═══════════════════════════════════════════════════════════════════════════
# STEP 6: Use the saved map for navigation
# ═══════════════════════════════════════════════════════════════════════════

# Option A: RTAB-Map Localization (visual features, robust)
ros2 launch vf_robot_slam rtabmap_loc.launch.py camera:=dual \
    database_path:=~/.ros/rtabmap.db

# Option B: AMCL Localization (requires .pgm/.yaml from Step 4/5)
# (This will be in vf_robot_navigation package)
ros2 launch vf_robot_navigation bringup_amcl.launch.py \
    map:=~/cogni-nav-x0/src/vf_robot_slam/maps/my_map.yaml
```

### Map Storage Recommendations

```
# Recommended directory structure
vf_robot_slam/maps/
├── home_v1.db              # RTAB-Map database (for RTAB-Map localization)
├── home_v1.pgm             # 2D grid image (for AMCL)
├── home_v1.yaml            # 2D grid metadata (for AMCL)
├── office_v1.db
├── office_v1.pgm
├── office_v1.yaml
└── README.md               # Notes about each map
```

## Topics Published

### RTAB-Map SLAM/Localization
| Topic | Type | Description |
|-------|------|-------------|
| `/map` | nav_msgs/OccupancyGrid | 2D occupancy grid |
| `/rtabmap/cloud_map` | sensor_msgs/PointCloud2 | 3D point cloud map |
| `/rtabmap/mapPath` | nav_msgs/Path | Robot trajectory |

### Depth to LaserScan
| Topic | Type | Camera Mode |
|-------|------|-------------|
| `/scan` | sensor_msgs/LaserScan | Single camera or merged |
| `/scan_d435i` | sensor_msgs/LaserScan | Dual mode only |
| `/scan_d455` | sensor_msgs/LaserScan | Dual mode only |

### TF Published
| Transform | Publisher | Mode |
|-----------|-----------|------|
| `map` → `odom` | RTAB-Map | Modes 1 & 2 |

## Nav2 Costmap Configuration

For dual camera mode, instead of using the merged `/scan`, you can configure Nav2 to use both scan topics as separate observation sources:

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

This eliminates the need for `ira_laser_tools` merger for costmaps (but AMCL still requires a single `/scan` topic).

## Troubleshooting

### RTAB-Map not detecting loop closures
- Ensure good lighting for visual features
- Move slowly to avoid motion blur
- Check that both RGB and depth images are being received

### Laser scan detecting floor (D435i)
- D435i is tilted 60° down - this is expected
- Use `camera:=d455` or `camera:=dual` for obstacle detection
- For dual mode, the D455 scan provides horizontal obstacle data

### TF errors
- Ensure Gazebo (or robot) is publishing `odom → base_footprint`
- Check that `robot_state_publisher` is running
- Verify frame names: `ros2 run tf2_tools view_frames`

### ira_laser_tools merger issues
- Ensure TF is available between camera frames and base_link
- Check that both `/scan_d435i` and `/scan_d455` are publishing
- Alternative: Use Nav2 multiple observation sources instead

## License

Apache-2.0
