# =============================================================================
# vf_robot_bringup / launch_utils / compose_params.py
# =============================================================================
# Composes a final Nav2 params YAML at launch time by merging:
#
#     base.yaml                          ← controller-agnostic Nav2 skeleton
#   ⊕ controllers/<controller>.yaml     ← FollowPath: block for the chosen controller
#   ⊕ localization/<localization>.yaml  ← amcl: block (only if localization == "amcl")
#   ⊕ robot rewrites                    ← per-robot footprint, frames, vel limits
#
# The merged file is written to /tmp/ with a descriptive filename and its
# path is returned. The launch file passes that path to every Nav2 node.
#
# Strict mode (default): if a robot profile tries to rewrite a key that does
# not exist in the merged params, raise a clear error with a "did you mean"
# suggestion. This catches typos at launch time instead of producing silent
# wrong behavior at runtime.
#
# Usage from a launch file:
#
#   from vf_robot_bringup.launch_utils.compose_params import compose, load_robot_profile
#
#   robot_rewrites = load_robot_profile(
#       "/.../config/robots/virofighter.yaml")
#
#   final_path = compose(
#       base_path        = "/.../config/nav2/nav2_base.yaml",
#       controller_path  = "/.../config/nav2/controllers/vf_inference.yaml",
#       localization_path= "/.../config/nav2/localization/amcl.yaml",  # or None
#       robot_rewrites   = robot_rewrites,
#       label            = "virofighter_vf_inference_rtabmap_loc",
#   )
#   # final_path is e.g. "/tmp/nav2_virofighter_vf_inference_rtabmap_loc_31415.yaml"
#
# =============================================================================

from __future__ import annotations

import difflib
import os
import sys
import tempfile
from typing import Any

import yaml


# -----------------------------------------------------------------------------
# Errors
# -----------------------------------------------------------------------------

class ComposeError(RuntimeError):
    """Raised when composition fails for a user-fixable reason."""
    pass


# -----------------------------------------------------------------------------
# YAML loading
# -----------------------------------------------------------------------------

def _load_yaml(path: str) -> dict:
    """Load a YAML file. Returns {} for empty files. Raises on parse errors."""
    if not os.path.isfile(path):
        raise ComposeError(f"YAML file not found: {path}")
    try:
        with open(path, "r") as f:
            data = yaml.safe_load(f)
    except yaml.YAMLError as e:
        raise ComposeError(f"Failed to parse YAML file '{path}':\n  {e}")
    if data is None:
        return {}
    if not isinstance(data, dict):
        raise ComposeError(
            f"YAML file '{path}' must contain a top-level dict, "
            f"got {type(data).__name__}"
        )
    return data


# -----------------------------------------------------------------------------
# Deep merge
# -----------------------------------------------------------------------------

def _deep_merge(base: dict, overlay: dict) -> dict:
    """
    Recursively merge `overlay` into `base`. `overlay` wins on conflict.

    Rules:
      - dict + dict  → recurse
      - list + list  → REPLACE wholesale (do not concatenate)
      - scalar + any → overlay value wins
      - missing key  → take overlay value

    Returns a new dict; does not mutate either input.
    """
    out = dict(base)
    for key, ov_val in overlay.items():
        if key in out and isinstance(out[key], dict) and isinstance(ov_val, dict):
            out[key] = _deep_merge(out[key], ov_val)
        else:
            out[key] = ov_val
    return out


# -----------------------------------------------------------------------------
# Dotted-path rewrite
# -----------------------------------------------------------------------------

def _collect_all_paths(d: dict, prefix: str = "") -> list[str]:
    """Walk a nested dict and return every leaf-or-branch dotted path."""
    paths: list[str] = []
    for key, val in d.items():
        path = f"{prefix}.{key}" if prefix else key
        paths.append(path)
        if isinstance(val, dict):
            paths.extend(_collect_all_paths(val, path))
    return paths


def _set_path(d: dict, dotted_path: str, value: Any, *, strict: bool) -> None:
    """
    Set d[a][b][c] = value for dotted_path "a.b.c".

    In strict mode (default), every intermediate key AND the final key
    must already exist in `d`. If not, raise ComposeError with a
    'did you mean' suggestion built from existing keys at the same level.

    In lenient mode, missing keys are created on the fly.
    """
    parts = dotted_path.split(".")
    if not parts or any(p == "" for p in parts):
        raise ComposeError(f"Invalid rewrite path: '{dotted_path}'")

    cursor: Any = d
    walked: list[str] = []

    for i, part in enumerate(parts[:-1]):
        walked.append(part)
        if not isinstance(cursor, dict):
            raise ComposeError(
                f"Rewrite path '{dotted_path}' tries to descend into "
                f"non-dict at '{'.'.join(walked[:-1])}' "
                f"(value is {type(cursor).__name__})"
            )
        if part not in cursor:
            if strict:
                _raise_unknown_key(dotted_path, walked, cursor)
            cursor[part] = {}
        cursor = cursor[part]

    if not isinstance(cursor, dict):
        raise ComposeError(
            f"Rewrite path '{dotted_path}' parent is not a dict "
            f"(parent path: '{'.'.join(walked)}')"
        )

    last = parts[-1]
    if strict and last not in cursor:
        walked.append(last)
        _raise_unknown_key(dotted_path, walked, cursor, leaf=True)

    cursor[last] = value


def _raise_unknown_key(
    full_path: str,
    walked: list[str],
    parent_dict: dict,
    *,
    leaf: bool = False,
) -> None:
    """Raise ComposeError with a 'did you mean' suggestion."""
    missing = walked[-1]
    siblings = [k for k in parent_dict.keys() if isinstance(k, str)]
    suggestions = difflib.get_close_matches(missing, siblings, n=3, cutoff=0.6)

    location = ".".join(walked[:-1]) if not leaf else ".".join(walked[:-1])
    where = location if location else "<root>"

    msg_lines = [
        f"Robot profile tries to rewrite key '{full_path}' but the segment "
        f"'{missing}' does not exist under '{where}' in the merged Nav2 params.",
    ]
    if suggestions:
        msg_lines.append(f"  Did you mean: {', '.join(suggestions)}?")
    if siblings:
        # Show up to 8 keys at this level so the user can scan
        shown = sorted(siblings)[:8]
        more = "" if len(siblings) <= 8 else f" (and {len(siblings) - 8} more)"
        msg_lines.append(f"  Available keys at '{where}': {', '.join(shown)}{more}")
    msg_lines.append(
        "  Fix: correct the key in the robot profile YAML, "
        "or set strict=False if you intentionally want to add a new key."
    )
    raise ComposeError("\n".join(msg_lines))


# -----------------------------------------------------------------------------
# Robot profile loading
# -----------------------------------------------------------------------------

def load_robot_profile(
    path: str,
    controller_family: str | None = None,
) -> dict[str, Any]:
    """
    Load a robot profile YAML and return its rewrite dict, optionally
    augmented with controller-family-specific overrides.

    A robot profile YAML must look like:

        robot:
          name: ViroFighter UVC-1
          description: ...

        rewrites:
          # ALWAYS applied (Nav2 base, costmaps, velocity_smoother, frames, ...)
          bt_navigator.ros__parameters.robot_base_frame: base_footprint
          local_costmap.local_costmap.ros__parameters.footprint: "[[0.1,0.2],...]"
          velocity_smoother.ros__parameters.max_velocity: [0.30, 0.0, 1.0]
          ...

        controller_overrides:
          # Applied ONLY when the chosen controller belongs to this family.
          # Family names are arbitrary; convention: "vf", "mppi", "dwb", "rpp".
          vf:
            controller_server.ros__parameters.FollowPath.robot_length: 0.60
            controller_server.ros__parameters.FollowPath.max_vel_x: 0.30
            ...
          mppi:
            controller_server.ros__parameters.FollowPath.vx_max: 0.30
            controller_server.ros__parameters.FollowPath.wz_max: 1.0
            ...

    Returns the merged dict (always-rewrites + family-specific overrides if
    `controller_family` is given). The `robot:` section is informational
    metadata for humans and is ignored by the composer.

    Args:
      path:               filesystem path to the profile YAML
      controller_family:  if given, also pull overrides from
                          controller_overrides[controller_family].
                          If the family is not present in the profile,
                          no overrides are applied (and no error raised).
    """
    data = _load_yaml(path)
    if "rewrites" not in data:
        raise ComposeError(
            f"Robot profile '{path}' is missing the 'rewrites:' section.\n"
            f"  See config/robots/README_ROBOT_PROFILE.md for the expected format."
        )
    rewrites = dict(data["rewrites"])  # copy so we don't mutate the loaded dict
    if not isinstance(rewrites, dict):
        raise ComposeError(
            f"Robot profile '{path}' has 'rewrites:' but it is not a dict "
            f"(got {type(rewrites).__name__})"
        )

    # Optionally merge controller-family-specific overrides on top
    if controller_family is not None:
        overrides_root = data.get("controller_overrides", {}) or {}
        if not isinstance(overrides_root, dict):
            raise ComposeError(
                f"Robot profile '{path}' has 'controller_overrides:' but "
                f"it is not a dict (got {type(overrides_root).__name__})"
            )
        family_overrides = overrides_root.get(controller_family, {}) or {}
        if not isinstance(family_overrides, dict):
            raise ComposeError(
                f"Robot profile '{path}' has 'controller_overrides.{controller_family}:' "
                f"but it is not a dict (got {type(family_overrides).__name__})"
            )
        # Family overrides win on conflict (later writes override earlier)
        for k, v in family_overrides.items():
            rewrites[k] = v

    # Validate all keys are dotted strings; values can be anything
    for k in rewrites.keys():
        if not isinstance(k, str) or "." not in k:
            raise ComposeError(
                f"Robot profile '{path}' has invalid rewrite key '{k}'. "
                f"Rewrite keys must be dotted paths like "
                f"'controller_server.ros__parameters.FollowPath.max_vel_x'."
            )
    return rewrites


# -----------------------------------------------------------------------------
# Public entry point
# -----------------------------------------------------------------------------

def compose(
    base_path: str,
    controller_path: str,
    localization_path: str | None,
    robot_rewrites: dict[str, Any],
    *,
    label: str = "composed",
    strict: bool = True,
    debug: bool = False,
) -> str:
    """
    Compose a final Nav2 params YAML and return its filesystem path.

    Args:
      base_path:         path to nav2_base.yaml (skeleton, controller-agnostic)
      controller_path:   path to controllers/<choice>.yaml (FollowPath fragment)
      localization_path: path to localization/amcl.yaml, or None to skip
      robot_rewrites:    dict of {dotted.key.path: value} from a robot profile
      label:             string used in the temp filename for debugging
                         e.g. "virofighter_vf_inference_rtabmap_loc"
      strict:            if True (default), raise on unknown rewrite keys
      debug:             if True, also print the output path to stderr

    Returns:
      Absolute path to a YAML file in /tmp/ that Nav2 nodes can load via
      `params_file=`.
    """
    # 1. Load all source files
    base = _load_yaml(base_path)
    controller = _load_yaml(controller_path)

    # 2. Merge in order: base ⊕ controller [⊕ localization]
    merged = _deep_merge(base, controller)
    if localization_path is not None:
        loc = _load_yaml(localization_path)
        merged = _deep_merge(merged, loc)

    # 3. Apply robot rewrites (strict by default)
    for dotted_key, value in robot_rewrites.items():
        _set_path(merged, dotted_key, value, strict=strict)

    # 4. Write to /tmp/ with a descriptive filename
    safe_label = "".join(c if c.isalnum() or c in "_-" else "_" for c in label)
    fd, out_path = tempfile.mkstemp(
        prefix=f"nav2_{safe_label}_",
        suffix=".yaml",
        dir=tempfile.gettempdir(),
        text=True,
    )
    with os.fdopen(fd, "w") as f:
        f.write(
            "# AUTO-GENERATED by vf_robot_bringup/launch_utils/compose_params.py\n"
            "# Do not edit — this file is overwritten on every launch.\n"
            f"# Source files:\n"
            f"#   base:         {base_path}\n"
            f"#   controller:   {controller_path}\n"
            f"#   localization: {localization_path or '(none)'}\n"
            f"#   robot:        {len(robot_rewrites)} rewrites applied\n"
            "#\n"
        )
        yaml.safe_dump(merged, f, default_flow_style=False, sort_keys=False)

    if debug:
        print(f"[compose_params] wrote {out_path}", file=sys.stderr)

    return out_path


# -----------------------------------------------------------------------------
# Self-test (run as script)
# -----------------------------------------------------------------------------

if __name__ == "__main__":
    # Quick self-test: compose a tiny example end-to-end and print the result.
    import json

    base = {
        "controller_server": {
            "ros__parameters": {
                "controller_frequency": 20.0,
                "FollowPath": {
                    "plugin": "PLACEHOLDER",
                    "max_vel_x": 0.22,
                },
            }
        },
        "local_costmap": {
            "local_costmap": {
                "ros__parameters": {
                    "robot_base_frame": "base_link",
                    "robot_radius": 0.22,
                }
            }
        },
    }
    controller = {
        "controller_server": {
            "ros__parameters": {
                "FollowPath": {
                    "plugin": "vf_robot_controller::VFRobotController",
                    "controller_mode": "inference",
                    "num_samples": 300,
                }
            }
        }
    }
    rewrites = {
        "controller_server.ros__parameters.FollowPath.max_vel_x": 0.30,
        "local_costmap.local_costmap.ros__parameters.robot_base_frame": "base_footprint",
    }

    # Write tmp inputs
    import tempfile as _t
    bd = _t.mkdtemp(prefix="compose_test_")
    bp = os.path.join(bd, "base.yaml")
    cp = os.path.join(bd, "ctrl.yaml")
    with open(bp, "w") as f:
        yaml.safe_dump(base, f)
    with open(cp, "w") as f:
        yaml.safe_dump(controller, f)

    out = compose(bp, cp, None, rewrites, label="self_test", debug=True)
    print(f"Composed file: {out}")
    print("Contents:")
    with open(out) as f:
        print(f.read())

    # Negative test: typo in rewrite key
    print("\n--- Negative test: typo in rewrite ---")
    bad_rewrites = {
        "controller_server.ros__parameters.FollowPath.max_vel_xx": 0.30,
    }
    try:
        compose(bp, cp, None, bad_rewrites, label="should_fail")
        print("ERROR: should have raised ComposeError")
    except ComposeError as e:
        print(f"OK — caught error:\n{e}")
