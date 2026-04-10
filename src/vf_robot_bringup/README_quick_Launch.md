# vf_robot_bringup — Cookbook

A practical reference for running every navigation mode in this package.
Save this file. Copy commands from here. Don't re-derive them every session.

> **Convention**: angle-bracket placeholders like `<map_name>` are things
> you substitute. Everything else is literal — copy as-is.

---

## Table of contents

1. [One-time setup](#0-one-time-setup)
2. [Phase 1 — Build a map](#phase-1--build-a-map)
3. [Phase 2 — Collect training data (META_CRITIC)](#phase-2--collect-training-data-meta_critic)
4. [Phase 3 — Train models offline](#phase-3--train-models-offline)
5. [Phase 4 — Main comparison experiments](#phase-4--main-comparison-experiments)
6. [Phase 5 — Ablation experiments](#phase-5--ablation-experiments)
7. [Phase 6 — Portability proof (TurtleBot3)](#phase-6--portability-proof-turtlebot3)
8. [Phase 7 — Localization sweep (one controller, all 4 modes)](#phase-7--localization-sweep)
9. [Reference tables](#reference-tables)
10. [Sidecar terminal (T3) commands](#sidecar-terminal-t3-commands)
11. [Quick diagnostics](#quick-diagnostics)
12. [Common errors and fixes](#common-errors-and-fixes)

---

## 0. One-time setup

Run these once per shell session before anything else.

```bash
# Source the workspace (add to ~/.bashrc to skip this every time)
cd ~/cogni-nav-x0
source install/setup.bash

# Confirm vf_robot_bringup is found
ros2 pkg list | grep vf_robot
# Expected: vf_robot_bringup, vf_robot_controller, vf_robot_description,
#           vf_robot_gazebo, vf_robot_messages, vf_robot_slam

# Confirm bringup_launch.py loads (parses args)
ros2 launch vf_robot_bringup bringup_launch.py --show-args | head -20

# Verify maps directory exists
mkdir -p ~/cogni-nav-x0/maps
ls -la ~/cogni-nav-x0/maps/
```

If anything above fails, **stop here** and rebuild:

```bash
cd ~/cogni-nav-x0
colcon build --packages-select vf_robot_bringup --symlink-install
source install/setup.bash
```

---

## Phase 1 — Build a map

**Goal**: Drive the robot around the Gazebo world manually so RTAB-Map
builds a 3D map. Save it. Optionally export a 2D occupancy grid for AMCL.

### T1 — Gazebo

```bash
ros2 launch vf_robot_gazebo vf_my_world_xacro.launch.py
```

Wait until you see Gazebo open and the robot spawn before starting T2.

### T2 — Build map (uses `mppi` for known-good driving)

```bash
ros2 launch vf_robot_bringup bringup_launch.py \
    robot:=virofighter \
    controller:=mppi \
    localization:=rtabmap_slam \
    camera:=dual \
    scan_method:=pc2scan \
    merge_scans:=true \
    map_name:=<map_name> \
    new_map:=true \
    use_sim_time:=true \
    rviz:=true
```

**Substitute** `<map_name>` with whatever you want to call this map, e.g.
`my_office`, `narrow_corridor`, `cluttered_room`. The map will be saved
to `~/cogni-nav-x0/maps/<map_name>/rtabmap.db`.

**Driving protocol**:
1. In RViz, use the "2D Nav Goal" button to send goals around the world.
2. Drive into every corner — RTAB-Map needs to see each region from multiple
   angles to build a clean loop-closed map.
3. Watch RTAB-Map's "RTABMapMap" display in RViz; the colored blocks are
   the map being built.
4. Drive for 3-10 minutes depending on world size.

**When it's "done"**:
- Every navigable area appears as colored blocks in RViz.
- Loop closure happens (you'll see the map "snap" into alignment when you
  drive back to a place you've been before).

**Save and quit**:
- Press `Ctrl+C` in T2 (the bringup terminal). RTAB-Map auto-saves on
  shutdown to `~/cogni-nav-x0/maps/<map_name>/rtabmap.db`.
- Verify: `ls -la ~/cogni-nav-x0/maps/<map_name>/`

### T2 (continued) — Optional: export to 2D map for AMCL

After Phase 1 saves the .db, if you also want to use the same map with
AMCL, export it as a 2D `.pgm` + `.yaml` pair:

```bash
# Run rtabmap_loc to publish /map from the .db
ros2 launch vf_robot_bringup bringup_launch.py \
    robot:=virofighter \
    controller:=mppi \
    localization:=rtabmap_loc \
    map_name:=<map_name> \
    rviz:=false &
LAUNCH_PID=$!
sleep 8

# Save the published /map to disk
ros2 run nav2_map_server map_saver_cli \
    -f ~/cogni-nav-x0/maps/<map_name>/<map_name> \
    --ros-args -p save_map_timeout:=10.0

# Stop the bringup
kill $LAUNCH_PID

# Verify
ls -la ~/cogni-nav-x0/maps/<map_name>/
# Expected: rtabmap.db, <map_name>.pgm, <map_name>.yaml
```

---

## Phase 2 — Collect training data (META_CRITIC)

**Goal**: Drive the robot around with the VF controller in `vf_collect`
mode. The plugin logs critic scores + observations to an HDF5 file that
the meta-critic training script consumes.

**Prerequisite**: A built map exists at `~/cogni-nav-x0/maps/<map_name>/`
(see Phase 1).

### T1 — Gazebo

```bash
ros2 launch vf_robot_gazebo vf_my_world_xacro.launch.py
```

### T2 — Bringup in collect mode

```bash
ros2 launch vf_robot_bringup bringup_launch.py \
    robot:=virofighter \
    controller:=vf_collect \
    localization:=rtabmap_loc \
    camera:=dual \
    scan_method:=pc2scan \
    merge_scans:=true \
    map_name:=<map_name> \
    use_sim_time:=true \
    rviz:=true
```

`vf_collect` is the data-logging variant of the VF controller. It runs
the full critic stack but **records** everything to an HDF5 file.

**Where the data goes**:
- `~/cogni-nav-x0/src/vf_robot_controller/training/data/run_<timestamp>.h5`
- Each `vf_collect` run creates a fresh `.h5` file
- Watch the controller_server logs for the "writing to ..." line

**Driving protocol**:
- Send 20-50 goals across the map using the "2D Nav Goal" button
- Cover varied scenarios: open corridors, tight doorways, near obstacles,
  near goals
- Each goal navigation = one "episode" of training data
- Drive for 20-40 minutes for a thesis-quality dataset

**Stop**: `Ctrl+C` in T2. The .h5 file is finalized on shutdown.

**Verify the file**:
```bash
cd ~/cogni-nav-x0/src/vf_robot_controller/training
python3 inspect_h5.py data/run_<timestamp>.h5
```

---

## Phase 3 — Train models offline

**No Gazebo, no ROS needed.** Pure Python.

```bash
cd ~/cogni-nav-x0/src/vf_robot_controller/training
conda activate dl

# Train the META_CRITIC model
python3 train.py --method meta_critic

# Or train the IMITATION baseline
python3 train.py --method imitation
```

**Outputs**:
- Best checkpoint: `checkpoints/meta_critic_best.pt` (or `imitation_best.pt`)
- Loss curves: `figures/loss_curve_META_CRITIC.png`

**Copy the trained model** to where the inference node expects it:
```bash
cp checkpoints/meta_critic_best.pt \
   ../meta_critic/models/meta_critic.pt
```

Then rebuild only if you symlinked (with `--symlink-install` you don't
need to rebuild — just rerun bringup).

---

## Phase 4 — Main comparison experiments

**Goal**: Run the four-way controller comparison on the same map, same
world, same localization. This is the core thesis result.

**Prerequisite**: Map exists at `~/cogni-nav-x0/maps/<map_name>/` AND
`meta_critic_best.pt` exists at `vf_robot_controller/meta_critic/models/`.

### T1 — Gazebo (run once, leave running for all 4 controllers)

```bash
ros2 launch vf_robot_gazebo vf_my_world_xacro.launch.py
```

### T2 — Bringup (run one at a time, change `controller:=` between runs)

#### Run 1: VF Inference (the thesis main result)

```bash
ros2 launch vf_robot_bringup bringup_launch.py \
    robot:=virofighter \
    controller:=vf_inference \
    localization:=rtabmap_loc \
    camera:=dual \
    scan_method:=pc2scan \
    merge_scans:=true \
    map_name:=<map_name> \
    use_sim_time:=true \
    rviz:=true
```

**Also start T3** (the sidecar) — see [Sidecar terminal](#sidecar-terminal-t3-commands).

#### Run 2: VF Fixed (ablation — same plugin, no meta-critic)

```bash
ros2 launch vf_robot_bringup bringup_launch.py \
    robot:=virofighter \
    controller:=vf_fixed \
    localization:=rtabmap_loc \
    camera:=dual \
    scan_method:=pc2scan \
    merge_scans:=true \
    map_name:=<map_name> \
    use_sim_time:=true \
    rviz:=true
```

No T3 needed — fixed mode runs entirely in C++.

#### Run 3: Stock Nav2 MPPI (primary baseline)

```bash
ros2 launch vf_robot_bringup bringup_launch.py \
    robot:=virofighter \
    controller:=mppi \
    localization:=rtabmap_loc \
    camera:=dual \
    scan_method:=pc2scan \
    merge_scans:=true \
    map_name:=<map_name> \
    use_sim_time:=true \
    rviz:=true
```

#### Run 4: Stock Nav2 DWB (secondary baseline)

```bash
ros2 launch vf_robot_bringup bringup_launch.py \
    robot:=virofighter \
    controller:=dwb \
    localization:=rtabmap_loc \
    camera:=dual \
    scan_method:=pc2scan \
    merge_scans:=true \
    map_name:=<map_name> \
    use_sim_time:=true \
    rviz:=true
```

**Between runs**: `Ctrl+C` T2 (and T3 if running). Wait 5 seconds for
cleanup, then start the next. Don't restart T1 — Gazebo can stay up.

**To collect comparable data across the 4 runs**: send the same set of
Nav2 goals each time. (Phase 8 will be a runner script that automates
this — for now, do it manually.)

---

## Phase 5 — Ablation experiments

**Goal**: Compare your META_CRITIC contribution against your IMITATION
baseline. Same plugin, same map, two learning targets.

### Run 1: VF Inference (META_CRITIC)

```bash
ros2 launch vf_robot_bringup bringup_launch.py \
    robot:=virofighter \
    controller:=vf_inference \
    localization:=rtabmap_loc \
    camera:=dual \
    map_name:=<map_name>
```
**+ T3 sidecar** (`meta_critic_inference.launch.py`)

### Run 2: VF Passive (IMITATION baseline)

```bash
ros2 launch vf_robot_bringup bringup_launch.py \
    robot:=virofighter \
    controller:=vf_passive \
    localization:=rtabmap_loc \
    camera:=dual \
    map_name:=<map_name>
```
**+ T3 sidecar** (`imitation_inference.launch.py`)

The C++ plugin in `vf_passive` mode hands `/cmd_vel` ownership to the
Python sidecar, which publishes velocities directly from the trained
imitation model.

---

## Phase 6 — Portability proof (TurtleBot3)

**Goal**: Show that the VF Robot Controller plugin is robot-agnostic by
running it on TurtleBot3 Waffle. One run is enough for the thesis claim.

**Prerequisite**: A TurtleBot3 Gazebo world. Use the standard nav2_bringup
TB3 world or any TB3-spawning world. **Do not use** `vf_my_world_xacro` —
that one spawns ViroFighter.

```bash
# T1: TB3 Gazebo (use whatever TB3 world you have)
ros2 launch vf_robot_gazebo tb3_world.launch.py
# (or whichever TB3 launch your workspace provides)

# T2: bringup with TB3 robot profile
ros2 launch vf_robot_bringup bringup_launch.py \
    robot:=turtlebot3_waffle \
    controller:=vf_inference \
    localization:=slam_toolbox \
    camera:=d435i \
    scan_method:=pc2scan \
    merge_scans:=false \
    use_sim_time:=true \
    rviz:=true
```

**Why slam_toolbox here**: TB3 has a real LiDAR — slam_toolbox is the
natural localization choice. Could also use `rtabmap_slam` if TB3 has a
depth camera in your sim.

**+ T3 sidecar** for vf_inference.

---

## Phase 7 — Localization sweep

**Goal**: Show the controller is localization-agnostic by running the
*same* controller across all 4 SLAM/localization modes.

### Mode 1 — RTAB-Map SLAM (build new map)
```bash
ros2 launch vf_robot_bringup bringup_launch.py \
    robot:=virofighter controller:=vf_inference \
    localization:=rtabmap_slam \
    map_name:=<map_name> new_map:=true \
    camera:=dual scan_method:=pc2scan merge_scans:=true
```

### Mode 2 — RTAB-Map Localization (use existing .db)
```bash
ros2 launch vf_robot_bringup bringup_launch.py \
    robot:=virofighter controller:=vf_inference \
    localization:=rtabmap_loc \
    map_name:=<map_name> \
    camera:=dual scan_method:=pc2scan merge_scans:=true
```

### Mode 3 — AMCL (use 2D .yaml map)
```bash
ros2 launch vf_robot_bringup bringup_launch.py \
    robot:=virofighter controller:=vf_inference \
    localization:=amcl \
    map:=$HOME/cogni-nav-x0/maps/<map_name>/<map_name>.yaml \
    camera:=dual scan_method:=pc2scan merge_scans:=true
```

After launch, **click "2D Pose Estimate" in RViz** and click roughly where
the robot is on the map. AMCL needs this initial pose hint.

### Mode 4 — SLAM Toolbox (build new 2D map)
```bash
ros2 launch vf_robot_bringup bringup_launch.py \
    robot:=virofighter controller:=vf_inference \
    localization:=slam_toolbox \
    camera:=dual scan_method:=pc2scan merge_scans:=true
```

**All four runs need T3 sidecar** for `vf_inference`.

---

## Reference tables

These tables are the "build your own command" cheat sheet. Pick one value
from each row that applies to you, plug them into the template at the bottom.

### Controllers

| `controller:=` | Plugin | Needs T3 sidecar? | Purpose |
|---|---|---|---|
| `vf_fixed` | VF, fixed weights | No | Ablation baseline (your plugin without ML) |
| `vf_inference` | VF + meta-critic ML | **Yes** (`meta_critic_inference.launch.py`) | THESIS MAIN RESULT |
| `vf_collect` | VF + data logger | No | Phase 2 — gather training data |
| `vf_passive` | VF in passthrough + ML cmd_vel | **Yes** (`imitation_inference.launch.py`) | IMITATION baseline |
| `mppi` | Stock Nav2 MPPI | No | Primary external baseline |
| `dwb` | Stock Nav2 DWB | No | Secondary external baseline |

### Robots

| `robot:=` | Description | Footprint type | Notes |
|---|---|---|---|
| `virofighter` | ViroFighter UVC-1 | Polygon (0.6×0.4 m) | Default. Has D435i + D455. |
| `turtlebot3_waffle` | TB3 Waffle Pi | Round (r=0.22 m) | Portability proof. Real LiDAR. |

### Localization modes

| `localization:=` | Source of `map→odom` | Needs prior map? | Map argument |
|---|---|---|---|
| `rtabmap_slam` | RTAB-Map (visual SLAM) | No (builds new) | `map_name:=<name>` + `new_map:=true` |
| `rtabmap_loc` | RTAB-Map (loaded .db) | Yes (.db file) | `map_name:=<name>` |
| `amcl` | AMCL particle filter | Yes (.pgm + .yaml) | `map:=/full/path/to/<name>.yaml` |
| `slam_toolbox` | SLAM Toolbox (laser SLAM) | No (builds new) | None — builds in memory |

### Cameras

| `camera:=` | What runs | When to use |
|---|---|---|
| `d435i` | Only D435i | Debugging the scan pipeline. Lower obstacles. |
| `d455` | Only D455 | Debugging. Better range, no blind zone. |
| `dual` | Both, merged via scan_merger | Production / thesis runs (default) |

### Scan method

| `scan_method:=` | Conversion | Trade-off |
|---|---|---|
| `dimg` | depthimage_to_laserscan | Fast but D435i has < 1.1 m blind zone |
| `pc2scan` | pointcloud_to_laserscan | Slower but no blind zone (default) |

### Other arguments

| Argument | Default | What it does |
|---|---|---|
| `merge_scans:=` | `true` | Merge dual cameras into single `/scan`. False for single-camera runs. |
| `map_name:=` | `default_map` | Folder name under `maps_dir` (RTAB-Map modes only) |
| `map:=` | `""` | Full path to .yaml (AMCL only) |
| `maps_dir:=` | `~/cogni-nav-x0/maps` | Base maps directory |
| `new_map:=` | `true` | RTAB-Map SLAM: delete existing .db. Set false to extend an existing map. |
| `use_sim_time:=` | `true` | True for Gazebo, false for real robot |
| `rviz:=` | `true` | Launch RViz with Nav2 panels |
| `rviz_config:=` | (auto) | Custom RViz config file path |
| `autostart_sidecar:=` | `false` | Auto-start T3 (don't use if T3 needs conda) |

### The full template

Substitute any values from the tables above into this skeleton:

```bash
ros2 launch vf_robot_bringup bringup_launch.py \
    robot:=<robot> \
    controller:=<controller> \
    localization:=<localization> \
    camera:=<camera> \
    scan_method:=<scan_method> \
    merge_scans:=<true|false> \
    map_name:=<map_name> \
    map:=<full_path_or_empty> \
    new_map:=<true|false> \
    use_sim_time:=true \
    rviz:=true
```

Not every argument matters in every mode:

| Mode | `map_name:=` | `map:=` | `new_map:=` |
|---|---|---|---|
| `rtabmap_slam` | **required** | ignored | **required** (true=fresh, false=continue) |
| `rtabmap_loc` | **required** | ignored | ignored |
| `amcl` | ignored | **required** | ignored |
| `slam_toolbox` | ignored | ignored | ignored |

---

## Sidecar terminal (T3) commands

When running `vf_inference`, `vf_collect`, or `vf_passive`, you need a
**third terminal** running the appropriate Python sidecar from
`vf_robot_controller`. The sidecar needs the conda `dl` environment for
PyTorch.

### Setup (every T3 session)

```bash
conda activate dl
source ~/cogni-nav-x0/install/setup.bash
```

### For `controller:=vf_inference` (META_CRITIC inference)

```bash
ros2 launch vf_robot_controller meta_critic_inference.launch.py
```

### For `controller:=vf_collect` (data logging)

```bash
# vf_collect doesn't strictly need a sidecar — the C++ plugin writes
# HDF5 directly. Confirm in your training/data folder if a new .h5
# file appears after the run starts.
ros2 launch vf_robot_controller meta_critic_collect.launch.py
# (only run this if your collect pipeline includes a Python logger)
```

### For `controller:=vf_passive` (IMITATION inference)

```bash
ros2 launch vf_robot_controller imitation_inference.launch.py
```

### Stopping T3

`Ctrl+C` in the T3 terminal. Always stop T3 **before** T2 to avoid the
inference node trying to publish to a dead topic.

---

## Quick diagnostics

Run these in any spare terminal while the system is up.

```bash
# Source first
source ~/cogni-nav-x0/install/setup.bash

# Is /scan publishing?
ros2 topic hz /scan
# Expected: ~10-30 Hz. If nothing, scan pipeline is broken.

# Is the merged scan getting through?
ros2 topic echo /scan --once | head -20

# Is /cmd_vel actually being commanded?
ros2 topic hz /cmd_vel
# Expected: ~20 Hz once a goal is set, 0 Hz when idle.

# Inspect the composed Nav2 yaml that this run produced
ls -lt /tmp/nav2_*.yaml | head -1
cat /tmp/nav2_<robot>_<controller>_<localization>_*.yaml | less

# Check Nav2 lifecycle states
ros2 service call /lifecycle_manager_navigation/is_active \
    nav2_msgs/srv/ManageLifecycleNodes "{command: 0}"
# (command 0 = STARTUP, returns success: true if everything's active)

# List active Nav2 nodes
ros2 node list | grep -E "controller|planner|amcl|map_server|slam_toolbox"

# Watch the meta-critic weights (vf_inference mode only)
ros2 topic echo /vf_controller/meta_weights
```

---

## Common errors and fixes

### `[map_server] yaml-filename parameter is empty`
**Cause**: You used `map_name:=` but `localization:=amcl`. AMCL needs `map:=`.
**Fix**: Use `map:=$HOME/cogni-nav-x0/maps/<name>/<name>.yaml`.

### `[amcl] Waiting for map....` (forever)
**Cause**: The map yaml path doesn't exist or is wrong.
**Fix**: `ls -la` the path. Build the map with rtabmap_slam first if missing.

### `Timed out waiting for transform from base_footprint to map`
**Cause**: No `map→odom` TF being published. Either AMCL/SLAM didn't
configure (see above two errors), or you forgot to set the initial pose
in RViz for AMCL.
**Fix**: For AMCL, click "2D Pose Estimate" in RViz at the robot's location.

### `New subscription discovered ... incompatible QoS`
**Cause**: A `/scan` publisher uses RELIABLE QoS but Nav2 expects
BEST_EFFORT (sensor data). Messages get silently dropped.
**Fix**: In `vf_robot_slam/scan_merger.py` and `pc_to_scan.py`, change
publishers to use `qos_profile_sensor_data`. (See QoS notes in the
project's known-issues file.)

### `Inconsistent configuration in collision checking`
**Cause**: MPPI's CostCritic uses circular collision check while local
costmap uses a polygon footprint. Cosmetic.
**Fix**: Ignore. Or set `consider_footprint: true` in `mppi.yaml` if
you want pixel-perfect agreement (slower).

### `PluginlibFactory: nav2_rviz_plugins/Selector failed to load`
**Cause**: The RViz config has panels that don't exist in Humble's
nav2_rviz_plugins (they're from Iron+).
**Fix**: Edit `rviz/vf_bringup.rviz` and remove the `Selector` and
`Docking` panel entries. Cosmetic, doesn't affect navigation.

### `ModuleNotFoundError: torch` (in T3)
**Cause**: T3 is not in the conda `dl` environment.
**Fix**: `conda activate dl` before sourcing the workspace.

### Robot moves but ignores goals
**Cause**: Local costmap has no obstacles → planner thinks everything is
free → MPPI samples diverge. Usually a /scan QoS or topic name issue.
**Fix**: `ros2 topic hz /scan` first. Then check the local_costmap shows
obstacles in RViz.

### `colcon build` fails after editing the composer
**Cause**: Probably a Python syntax error or missing import.
**Fix**: `python3 -c "import ast; ast.parse(open('vf_robot_bringup/launch_utils/compose_params.py').read())"` to check syntax.

### Two simultaneous launches collide on /tmp/ files
**Cause**: They don't — the composer uses random suffixes via `tempfile.mkstemp()`.
**Non-fix**: Don't worry about it.

### Want to inspect the exact Nav2 yaml that's being used right now
```bash
ls -lt /tmp/nav2_*.yaml | head -1   # most recent
# Or grep for the robot+controller+localization label:
ls /tmp/nav2_virofighter_vf_inference_rtabmap_loc_*.yaml
```

### Kill all ROS processes between runs
```bash
# Ctrl+C usually works, but if anything is stuck:
pkill -9 -f "ros2|gazebo|rviz2|controller_server|planner_server|amcl|map_server"
sleep 2
```

---

## Appendix — File paths quick reference

```
~/cogni-nav-x0/
├── install/                                     ← source this
├── src/
│   ├── vf_robot_bringup/
│   │   ├── config/
│   │   │   ├── nav2/
│   │   │   │   ├── nav2_base.yaml               ← base Nav2 skeleton
│   │   │   │   ├── controllers/                 ← 6 controller fragments
│   │   │   │   └── localization/                ← amcl.yaml, slam_toolbox.yaml
│   │   │   └── robots/                          ← virofighter.yaml, tb3.yaml
│   │   ├── launch/
│   │   │   ├── bringup_launch.py                ← THE entry point
│   │   │   ├── navigation_launch.py
│   │   │   ├── localization_launch.py
│   │   │   ├── slam_launch.py
│   │   │   └── rviz_launch.py
│   │   └── vf_robot_bringup/launch_utils/
│   │       └── compose_params.py                ← the composer
│   ├── vf_robot_controller/
│   │   ├── meta_critic/models/meta_critic.pt    ← deployed model
│   │   └── training/
│   │       ├── data/run_*.h5                    ← collected episodes
│   │       ├── checkpoints/meta_critic_best.pt  ← trained model
│   │       └── train.py                         ← training entry
│   └── vf_robot_slam/                           ← RTAB-Map + scan pipeline
└── maps/
    └── <map_name>/
        ├── rtabmap.db                           ← RTAB-Map binary map
        ├── <map_name>.pgm                       ← optional 2D occupancy
        └── <map_name>.yaml                      ← optional 2D yaml (for AMCL)
```
---

## Sidecar terminals (T3) for ML controllers

Three of the six controllers require a Python sidecar from
`vf_robot_controller` to be useful. The sidecar runs in a separate terminal
in the conda `dl` environment (not the system Python) so PyTorch is
available. Always start the sidecar **after** the bringup terminal (T2)
has fully come up — Nav2 lifecycle nodes ACTIVE, plugin loaded.

### `vf_inference` — META_CRITIC adaptive weights (thesis main result)

```bash
conda activate dl
source /opt/ros/humble/setup.bash
source ~/cogni-nav-x0/install/setup.bash

ros2 launch vf_robot_controller meta_critic_inference_launch.py
```

Loads `meta_critic/models/meta_critic.pt`, runs inference at ~20 Hz, publishes
critic weights on `/vf_controller/meta_weights`. The C++ plugin's
`WeightAdapter` consumes them every control cycle.

### `vf_collect` — META_CRITIC data collection

```bash
conda activate dl
source /opt/ros/humble/setup.bash
source ~/cogni-nav-x0/install/setup.bash

cd ~/cogni-nav-x0/src/vf_robot_controller
ros2 launch vf_robot_controller meta_critic_collect_launch.py
```

(The `cd` matters — the data logger writes HDF5 files to a relative path
`training/data/run_*.h5` so they land inside the source tree where
`dataset.py` can find them at training time.)

### `vf_passive` — IMITATION baseline deployment

```bash
conda activate dl
source /opt/ros/humble/setup.bash
source ~/cogni-nav-x0/install/setup.bash

ros2 launch vf_robot_controller imitation_inference_launch.py
```

Loads `meta_critic/models/imitation.pt`, runs inference at ~20 Hz, publishes
directly to `/cmd_vel`. The C++ plugin in PASSIVE mode early-returns zero
so the imitation node owns velocity commands.

### Shutdown order

Always Ctrl+C the sidecar (T3) **before** the bringup (T2). The sidecar
publishes to topics the controller_server subscribes to, and killing
controller_server first leaves the sidecar publishing into the void —
which for `meta_critic_collect` means HDF5 buffers may not flush cleanly.

### Sidecar autostart

`bringup_launch.py` accepts `autostart_sidecar:=true` which launches the
matching sidecar automatically (mapped via `SIDECAR_CONTROLLERS` in the
launch file). Default is `false` because most users want the sidecar in
its own terminal so logs are easy to read and so the conda env is
explicit. Use the autostart only for unattended runs.

```bash
# Manual (recommended for development)
ros2 launch vf_robot_bringup bringup_launch.py controller:=vf_inference ...
# then in a separate terminal:
ros2 launch vf_robot_controller meta_critic_inference_launch.py

# Auto (recommended for unattended runs)
ros2 launch vf_robot_bringup bringup_launch.py \
    controller:=vf_inference \
    autostart_sidecar:=true \
    ...
```
