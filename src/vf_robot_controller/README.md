# 🤖 `vf_robot_controller`

> **Adaptive Meta-Critic Navigation Plugin for Nav2**
> ROS 2 Humble · Nav2 · MPPI · PyTorch
> Thesis project — robust motion planning for disinfection robots in tight hospital spaces

[![ROS2 Humble](https://img.shields.io/badge/ROS2-Humble-blue)](https://docs.ros.org/en/humble/)
[![Nav2](https://img.shields.io/badge/Nav2-Local%20Controller-green)](https://navigation.ros.org/)
[![License](https://img.shields.io/badge/License-Apache%202.0-green)](LICENSE)

---

## What this package is

A self-contained ROS 2 package providing:

1. **A C++ Nav2 local controller plugin** (`vf_robot_controller::VFRobotController`)
   that implements MPPI trajectory optimisation over 10 modular behavioural critics,
   wrapped in a hard safety shell that vetoes any candidate trajectory entering
   `LETHAL_OBSTACLE` cells.

2. **Python ML sidecar nodes** that train a small MLP (the "meta-critic") to
   dynamically reweight those 10 critics every control cycle, conditioned on
   local costmap and sensor observations. This is the thesis main contribution.

3. **Offline training scripts** for both methods compared in the thesis:
   - **META_CRITIC**: hindsight ideal-weight recovery + MSELoss on weight vectors
   - **IMITATION**: behaviour cloning of a teacher controller's `cmd_vel`

## What this package is NOT

- **Not a runtime Nav2 stack.** It does not own `nav2_base.yaml`, robot profiles,
  AMCL/SLAM Toolbox/RTAB-Map config, planner config, BT config, costmap config,
  or any "full Nav2 params file". For launching the robot, use
  [`vf_robot_bringup`](../vf_robot_bringup), which composes the right Nav2
  params from per-robot, per-controller, and per-localisation fragments.

- **Not a sensor pipeline.** It does not own depth-to-scan, scan merging, or
  RTAB-Map nodes. Those live in [`vf_robot_slam`](../vf_robot_slam).

- **Not a robot description.** URDF, meshes, sensor TFs are in
  `vf_robot_description`.

This split exists so the plugin and ML pipeline can be developed, tested, and
versioned independently from the bringup orchestration.

## The four operating modes

The plugin reads one parameter — `controller_mode` — from its YAML config and
selects one of four runtime behaviours:

| Mode        | What the C++ plugin does                                              | Python sidecar required                  |
|-------------|------------------------------------------------------------------------|------------------------------------------|
| `fixed`     | Classical MPPI with YAML-declared critic weights. Thesis baseline.    | None                                     |
| `collect`   | Same as fixed, plus publishes the N×K critic-score matrix every cycle on `/vf_controller/critic_data` for training data collection. | `meta_critic_collect_launch.py`          |
| `inference` | `WeightAdapter` subscribes to `/vf_controller/meta_weights` and replaces YAML weights with the meta-critic NN output every cycle. **The thesis main result.** | `meta_critic_inference_launch.py`        |
| `passive`   | `computeVelocityCommands()` returns zero immediately. The Nav2 action server is satisfied, but the C++ plugin produces no `/cmd_vel`. The Python `imitation_inference_node` owns `/cmd_vel` directly. **The behaviour-cloning baseline.** | `imitation_inference_launch.py`          |

To switch modes you change ONE parameter in your Nav2 params file:
`controller_mode: "fixed" | "collect" | "inference" | "passive"`. Everything
else in the controller's parameter set is mode-agnostic.

## Quick start

This package is normally launched **via `vf_robot_bringup`**, not directly.

```bash
# Build
cd ~/cogni-nav-x0
colcon build --packages-select vf_robot_controller --symlink-install
source install/setup.bash

# Launch (the bringup orchestrates the full Nav2 stack + this plugin)
ros2 launch vf_robot_bringup bringup_launch.py \
    robot:=virofighter \
    controller:=vf_inference \
    localization:=rtabmap_loc

# In a second terminal (conda dl env): start the meta-critic sidecar
conda activate dl
ros2 launch vf_robot_controller meta_critic_inference_launch.py
```

For all four modes and combinations see
[`vf_robot_bringup/README_quick_Launch.md`](../vf_robot_bringup/README_quick_Launch.md).

## Package layout
vf_robot_controller/
├── CMakeLists.txt
├── package.xml
├── vf_robot_controller.xml      # Nav2 plugin description
├── critics.xml                  # Critic plugin descriptions
│
├── include/vf_robot_controller/ # C++ headers
│   ├── controller.hpp           # the Nav2 plugin entry point
│   ├── optimizer.hpp            # MPPI loop
│   ├── critic_manager.hpp       # holds the 10 critics, applies weights
│   ├── parameter_handler.hpp    # YAML → struct
│   ├── critics/                 # 10 critic .hpp files
│   ├── gcf/                     # geometric complexity field
│   ├── safety/                  # hard safety shell + swept volume checker
│   ├── models/                  # state.hpp, trajectory.hpp
│   └── tools/                   # WeightAdapter, DataRecorder, visualiser, etc.
│
├── src/                         # mirrors include/ structure
│
├── meta_critic/                 # Python ML sidecars
│   ├── feature_extractor.py     # builds the 410-dim observation vector
│   ├── meta_critic_inference_node.py
│   ├── meta_critic_data_logger.py
│   ├── imitation_inference_node.py
│   ├── imitation_data_logger.py
│   └── models/                  # meta_critic.pt + imitation.pt live here
│
├── training/                    # offline ML training
│   ├── README.md                # training pipeline guide
│   ├── train.py                 # entry point: --method META_CRITIC | IMITATION
│   ├── dataset.py               # both dataset classes + label generators
│   ├── model.py                 # MetaCriticMLP + ImitationMLP
│   ├── evaluate.py
│   ├── inspect_h5.py
│   ├── diagnose_h5.py
│   ├── rl_finetune.py           # optional PPO fine-tuning (not used in thesis)
│   ├── data/                    # HDF5 training files (gitignored)
│   ├── checkpoints/             # training checkpoints (gitignored)
│   └── figures/                 # loss curves
│
├── launch/                      # sidecar-only launches (no controller_server)
│   ├── meta_critic_inference_launch.py
│   ├── meta_critic_collect_launch.py
│   ├── imitation_inference_launch.py
│   └── imitation_collect_launch.py    # works with any teacher controller
│
├── config/                      # one example file (documentation)
│   └── vf_controller_example.yaml
│
└── test/                        # gtest unit tests
    ├── test_critics.cpp
    ├── test_optimizer.cpp
    ├── test_safety.cpp
    ├── test_gcf.cpp
    └── test_trajectory.cpp

## The 10 critics

Order matters — this defines the index used by the meta-critic NN output vector:

| Idx | Name                  | What it scores                                              |
|-----|-----------------------|-------------------------------------------------------------|
| 0   | ObstacleCritic        | Cost from local costmap inflation layers                    |
| 1   | VolumetricCritic      | 3D clearance from depth-camera pointcloud                   |
| 2   | DynamicObstacleCritic | Predicted collision with moving obstacles                   |
| 3   | PathFollowCritic      | Cross-track error vs global plan                            |
| 4   | SmoothnessCritic      | Curvature, jerk, and angular acceleration                   |
| 5   | GoalCritic            | Distance + heading alignment to current goal                |
| 6   | VelocityCritic        | Penalty for being far from preferred forward velocity       |
| 7   | CorridorCritic        | Corridor centering when in narrow passages                  |
| 8   | ClearanceCritic       | Lateral clearance to nearest obstacle on each side          |
| 9   | OscillationCritic     | Penalty for sign-flipping angular velocity                  |

## Training the meta-critic

See [`training/README.md`](training/README.md) for the full training pipeline.

TL;DR:
```bash
# 1. Collect data with the controller in COLLECT mode
ros2 launch vf_robot_bringup bringup_launch.py controller:=vf_collect ...
# in a second terminal:
ros2 launch vf_robot_controller meta_critic_collect_launch.py
# drive episodes in RViz, HDF5 files accumulate in training/data/

# 2. Train
cd ~/cogni-nav-x0/src/vf_robot_controller
conda activate dl
python training/train.py --method META_CRITIC

# 3. Deploy
cp training/checkpoints/meta_critic_best.pt meta_critic/models/meta_critic.pt

# 4. Run inference
ros2 launch vf_robot_bringup bringup_launch.py controller:=vf_inference ...
ros2 launch vf_robot_controller meta_critic_inference_launch.py
```

## Build

```bash
cd ~/cogni-nav-x0
colcon build --packages-select vf_robot_controller \
  --packages-ignore nav2_common nav2_msgs nav2_voxel_grid nav2_util \
    nav2_behavior_tree nav2_lifecycle_manager nav2_map_server \
    nav2_costmap_2d nav2_core nav2_ros_common \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
```

The `--packages-ignore` flags exist because cloning the full `navigation2/`
source tree alongside this package causes colcon to try to rebuild every
Nav2 component from source, which creates dependency conflicts. Either
use the system Nav2 packages from `apt` (recommended) and pass these
ignores, or remove the Nav2 source folder from your workspace.

## Dependencies

| Layer        | Required                                                                       |
|--------------|--------------------------------------------------------------------------------|
| ROS 2        | Humble                                                                         |
| Nav2         | `nav2_core`, `nav2_costmap_2d`, `nav2_util` (system install via apt)          |
| C++          | Eigen3, optionally PCL (for 3D volumetric clearance)                           |
| Python (ML)  | PyTorch, NumPy, h5py — install in a conda env named `dl`                      |
| Sensor stack | `vf_robot_slam` (for depth-to-scan and RTAB-Map), system `depthimage_to_laserscan` |

## Related packages in this workspace

- `vf_robot_bringup` — single entry point for launching navigation
- `vf_robot_slam` — depth-to-scan, RTAB-Map, scan merging
- `vf_robot_description` — URDF, meshes, sensor TFs
- `vf_robot_gazebo` — Gazebo worlds for simulation testing

## License

Apache-2.0
