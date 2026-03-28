# ViroFighter UVC-1 Package Architecture

## Single Source of Truth Design

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                         vf_robot_description                                 │
│                    (SINGLE SOURCE OF TRUTH)                                  │
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
│  meshes/                                                                    │
│  ├── bases/virofighter_base.dae, Cube.019.dae                              │
│  ├── sensors/camera_d435i.dae, ...                                         │
│  └── wheels/wheel_*.dae, wrist_*.dae                                       │
│                                                                             │
│  launch/                                                                    │
│  ├── display_robot_xacro.launch.py  → RViz only (xacro)                    │
│  └── display_robot_urdf.launch.py   → RViz only (plain URDF)               │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
                           │
                           │ xacro processed at runtime
                           │ via $(find vf_robot_description)
                           ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                          vf_robot_gazebo                                     │
│                     (SIMULATION PACKAGE)                                     │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│  TWO PARALLEL SIMULATION PIPELINES:                                         │
│                                                                             │
│  ┌─────────────────────────────────────────────────────────────────────┐   │
│  │  XACRO PIPELINE (from vf_robot_description)                         │   │
│  │                                                                     │   │
│  │  vf_*_world_xacro.launch.py                                         │   │
│  │       │                                                             │   │
│  │       ├── SetEnvironmentVariable (PREPEND paths)                    │   │
│  │       ├── gzserver (starts Gazebo physics)                          │   │
│  │       ├── gzclient (delayed 3s)                                     │   │
│  │       ├── vf_robot_state_publisher.launch.py                        │   │
│  │       │       └── processes xacro → /robot_description topic        │   │
│  │       └── vf_spawn_xacro.launch.py (delayed 5s)                     │   │
│  │               └── spawn_entity -topic robot_description             │   │
│  │                                                                     │   │
│  │  Mesh resolution: package://vf_robot_description/meshes/...         │   │
│  └─────────────────────────────────────────────────────────────────────┘   │
│                                                                             │
│  ┌─────────────────────────────────────────────────────────────────────┐   │
│  │  SDF PIPELINE (pre-converted model.sdf)                             │   │
│  │                                                                     │   │
│  │  vf_*_world_sdf.launch.py                                           │   │
│  │       │                                                             │   │
│  │       ├── SetEnvironmentVariable (GAZEBO_MODEL_PATH)                │   │
│  │       ├── gzserver + gzclient (no delay needed)                     │   │
│  │       ├── vf_spawn_sdf.launch.py                                    │   │
│  │       │       └── spawn_entity -file model.sdf                      │   │
│  │       └── vf_robot_state_publisher.launch.py                        │   │
│  │               └── still uses xacro for TF tree!                     │   │
│  │                                                                     │   │
│  │  Mesh resolution: model://uvc1_common/meshes/...                    │   │
│  └─────────────────────────────────────────────────────────────────────┘   │
│                                                                             │
│  models/                                                                    │
│  ├── uvc1_virofighter/model.sdf  ← AUTO-GENERATED by xacro_to_sdf.sh       │
│  ├── uvc1_common/meshes/         ← COPY of vf_robot_description meshes     │
│  ├── uvc1_hospital/              (world models)                            │
│  └── uvc1_corridors*/            (world models)                            │
│                                                                             │
│  worlds/                                                                    │
│  ├── empty_world.world                                                     │
│  ├── hospital.world                                                        │
│  └── my_world.world                                                        │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

## Key Design Decisions

### 1. Why Two Pipelines?

| Feature | XACRO Pipeline | SDF Pipeline |
|---------|---------------|--------------|
| Source | Live xacro processing | Pre-converted model.sdf |
| Mesh URIs | `package://` | `model://` |
| Edit workflow | Edit xacro → launch | Edit xacro → run xacro_to_sdf.sh → launch |
| Startup time | Slower (xacro processing) | Faster |
| Delays needed | Yes (3s client, 5s spawn) | No |
| Use case | Development, testing | Production, demos |

### 2. Environment Variable Rules

**CRITICAL: Never replace, always prepend!**

```python
# WRONG — destroys Gazebo's own paths
SetEnvironmentVariable(name='GAZEBO_RESOURCE_PATH', value='/my/path')

# CORRECT — preserves system paths
SetEnvironmentVariable(
    name='GAZEBO_RESOURCE_PATH',
    value=os.pathsep.join([
        '/my/path',
        '/usr/share/gazebo-11',
        '/opt/ros/humble/share',
    ]),
)
```

### 3. TF Tree Architecture

```
                    odom
                      │
                      │ (Gazebo diff_drive plugin)
                      ▼
               base_footprint
                      │
                      │ (robot_state_publisher from xacro)
                      ▼
                 base_link
                      │
        ┌─────────────┼─────────────┬─────────────┐
        │             │             │             │
        ▼             ▼             ▼             ▼
   wheel_*_link  camera_*_link  ultrasonic_*  uvc_lights_link
```

**Both pipelines use robot_state_publisher for TF!**
- Gazebo only publishes `odom → base_footprint`
- RSP publishes all other transforms

### 4. Timing Delays (XACRO pipeline only)

| Component | Delay | Reason |
|-----------|-------|--------|
| gzclient | 3s | Prevents "Assertion px != 0" GUI race crash |
| spawn_entity | 5s | Prevents mesh-load crash before rendering scene ready |
| rviz2 | 6s | Ensures robot is spawned before visualization |

SDF pipeline doesn't need delays because `model://` URIs are resolved differently.

## Launch Commands

```bash
# RViz only (vf_robot_description)
ros2 launch vf_robot_description display_robot_xacro.launch.py

# Gazebo + RViz — XACRO pipeline
ros2 launch vf_robot_gazebo vf_empty_world_xacro.launch.py
ros2 launch vf_robot_gazebo vf_hospital_world_xacro.launch.py
ros2 launch vf_robot_gazebo vf_my_world_xacro.launch.py

# Gazebo + RViz — SDF pipeline (faster)
ros2 launch vf_robot_gazebo vf_empty_world_sdf.launch.py
ros2 launch vf_robot_gazebo vf_hospital_world_sdf.launch.py
ros2 launch vf_robot_gazebo vf_my_world_sdf.launch.py
```

## Workflow: Editing the Robot

1. Edit any xacro file in `vf_robot_description/urdf/xacro/`
2. For XACRO pipeline: Just re-launch — changes are live
3. For SDF pipeline: Run `./xacro_to_sdf.sh uvc1_virofighter.xacro`
4. Never edit `model.sdf` manually!

## Future Packages

Any future package (navigation, SLAM, etc.) that needs the robot description:

```python
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.descriptions import ParameterValue

xacro_path = PathJoinSubstitution([
    FindPackageShare('vf_robot_description'),
    'urdf', 'xacro', 'uvc1_virofighter.xacro',
])

robot_description = ParameterValue(
    Command([FindExecutable(name='xacro'), ' ', xacro_path]),
    value_type=str,
)
```

This pattern works anywhere because:
- xacro is resolved via ament index (no environment setup needed)
- `package://` URIs are resolved automatically
- Single source of truth is always used
