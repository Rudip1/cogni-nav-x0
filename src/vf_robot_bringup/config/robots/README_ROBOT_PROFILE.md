# Robot Profile Format

Each `.yaml` file in this directory describes ONE robot. The file is **not**
loaded by Nav2 directly — it is processed by
`vf_robot_bringup/launch_utils/compose_params.py` at launch time, which
applies its rewrites on top of `config/nav2/nav2_base.yaml` and the chosen
controller fragment.

## File structure

```yaml
robot:
  name: <human-readable name>
  description: >-
    <short description>
  base_frame: base_footprint
  ... (other informational fields, ignored by composer)

rewrites:
  # Always applied, regardless of which controller is chosen.
  # One {dotted.key.path: value} per line.
  bt_navigator.ros__parameters.robot_base_frame: base_footprint
  velocity_smoother.ros__parameters.max_velocity: [0.30, 0.0, 1.0]
  ...

controller_overrides:
  # Applied ONLY when the chosen controller belongs to this family.
  vf:
    controller_server.ros__parameters.FollowPath.robot_length: 0.60
    ...
  mppi:
    controller_server.ros__parameters.FollowPath.vx_max: 0.30
    ...
  dwb:
    controller_server.ros__parameters.FollowPath.max_vel_x: 0.30
    ...
```

## Adding a new robot

1. Copy `virofighter.yaml` to `<your_robot>.yaml`.
2. Update the `robot:` metadata block.
3. Replace the values in `rewrites:` with your robot's frames, footprint
   (or `robot_radius`), velocity limits, sensor topics.
4. Update each `controller_overrides.<family>:` block with the right
   per-plugin velocity/dimension values.
5. Test:
```bash
   PYTHONPATH=. python3 -c "
   from vf_robot_bringup.launch_utils.compose_params import compose, load_robot_profile
   for family in ['vf', 'mppi', 'dwb']:
       rw = load_robot_profile('config/robots/<your_robot>.yaml',
                               controller_family=family)
       print(f'{family}: {len(rw)} rewrites OK')
   "
```
6. Launch:
```bash
   ros2 launch vf_robot_bringup bringup_launch.py \
       robot:=<your_robot> controller:=mppi localization:=rtabmap_loc
```

## Strict mode

The composer runs in **strict mode**: every key in `rewrites` and
`controller_overrides` MUST already exist in the merged Nav2 params. If you
add a key that doesn't exist (typo, wrong nesting, removed Nav2 feature),
the launch fails immediately with a "Did you mean ...?" suggestion.

To add a brand-new key, first add it (with a placeholder value) to
`config/nav2/nav2_base.yaml` so the composer knows it's a legal target.

## Why `rewrites` vs `controller_overrides`

Different controller plugins use different parameter names for the same
physical concept:

| Concept           | VF plugin       | nav2_mppi  | dwb_core      |
|-------------------|-----------------|------------|---------------|
| max forward speed | `max_vel_x`     | `vx_max`   | `max_vel_x`   |
| max angular speed | `max_vel_theta` | `wz_max`   | `max_vel_theta` |
| max accel         | `acc_lim_x`     | `ax_max`   | `acc_lim_x`   |

Putting all of them in the always-applied `rewrites:` block would fail
strict mode for whichever controller doesn't define those keys. The
`controller_overrides:` split lets each family declare its own param
names while sharing the always-applied costmap/frame/velocity-smoother
rewrites.
