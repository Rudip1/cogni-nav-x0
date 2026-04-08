# Future Work

Things deliberately left out of the Phase 1 bringup. Re-evaluate after the
experiment runner produces baseline numbers — only add what the data shows
is needed.

## Additional baseline controllers

- **`rpp.yaml`** — Regulated Pure Pursuit. Useful as a "geometric path
  follower" baseline. Filling in: copy the `FollowPath:` block from
  `nav2_bringup/params/nav2_params.yaml` (RPP section), drop into
  `config/nav2/controllers/rpp.yaml`, add `rpp:` block under
  `controller_overrides:` in both robot profiles.

- **`graceful.yaml`** — Graceful Controller. Designed for smooth final
  approach to a docking pose, not general corridor navigation. Only worth
  adding if you plan a "final approach to charging station" experiment.

## Additional planners

- **`smac_2d.yaml` / `smac_hybrid.yaml`** — Smac Planner family. Better
  than NavFn for non-holonomic robots in tight corridors. Only worth
  adding if a thesis chapter does a planner ablation. To wire these in:
  add `planner_path` argument to `compose_params.compose()`, remove
  `planner_server:` block from `nav2_base.yaml`, add `planner:=` arg to
  `bringup_launch.py`.

## When to revisit

After Phase 2 (experiment runner) is producing CSVs and you have data
showing whether the 6 existing controllers tell a complete enough story.
