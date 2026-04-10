# Training the Meta-Critic and Imitation Models

This directory contains the offline training pipeline for the two learning
methods compared in the thesis:

| Method        | What it predicts                                  | Loss        | Output model path                  |
|---------------|---------------------------------------------------|-------------|------------------------------------|
| `META_CRITIC` | 10-dim per-critic weight vector (softmax)         | MSELoss     | `meta_critic/models/meta_critic.pt` |
| `IMITATION`   | 2-dim teacher cmd_vel `[linear.x, angular.z]`     | MSELoss     | `meta_critic/models/imitation.pt`   |

Both share the same 410-dim feature vector built by `feature_extractor.py`
(local costmap patch + odometry + global path features) and the same MLP
backbone (`Linear → LayerNorm → ReLU` × 3, then a 10-dim softmax head for
META_CRITIC or a 2-dim linear head for IMITATION).

---

## Quick start

```bash
cd ~/cogni-nav-x0/src/vf_robot_controller
conda activate dl

# META_CRITIC (the thesis main contribution)
python training/train.py --method META_CRITIC

# IMITATION (the behaviour-cloning baseline)
python training/train.py --method IMITATION
```

Both default to: `epochs=100`, `batch=128`, `lr=1e-3`, AdamW + cosine LR
schedule, early-stopping with patience=15.

---

## Data collection

### META_CRITIC data

You **must** use `vf_robot_controller` in COLLECT mode. The N×K critic-score
matrix that the training labels are derived from is only available from the
custom critics — DWB or stock Nav2 MPPI cannot produce it.

```bash
# Terminal 1: bringup with collect-mode controller
ros2 launch vf_robot_bringup bringup_launch.py \
    robot:=turtlebot3_waffle controller:=vf_collect localization:=slam_toolbox

# Terminal 2: sidecar (writes HDF5 to training/data/)
conda activate dl
cd ~/cogni-nav-x0/src/vf_robot_controller
ros2 launch vf_robot_controller meta_critic_collect_launch.py
```

Drive the robot in RViz with Nav2 Goal. Each episode writes a new
`run_YYYYMMDD_HHMMSS.h5` file in `training/data/`. Target: 300+ episodes
across varied map regions.

### IMITATION data

Works with **any** teacher controller (DWB, stock MPPI, RPP, your `vf_fixed`).
The data logger only needs the teacher's `/cmd_vel` and the 410-dim feature
vector from `feature_extractor.py`.

```bash
# Terminal 1: bringup with whatever teacher controller you want to clone
ros2 launch vf_robot_bringup bringup_launch.py \
    robot:=turtlebot3_waffle controller:=mppi localization:=slam_toolbox

# Terminal 2: imitation collection sidecar
ros2 launch vf_robot_controller imitation_collect_launch.py
```

HDF5 files written to `training/imitation/`.

---

## Training

```bash
# All defaults
python training/train.py --method META_CRITIC
python training/train.py --method IMITATION

# Override defaults
python training/train.py \
    --method META_CRITIC \
    --data-dir training/data \
    --epochs 200 \
    --batch-size 256 \
    --lr 5e-4
```

Training writes:
- Best checkpoint: `training/checkpoints/{meta_critic,imitation}_best.pt`
- Loss curve: `training/figures/loss_curve_{META_CRITIC,IMITATION}.png`

---

## Deploy a trained model

The C++ inference path loads from `meta_critic/models/`, NOT `training/checkpoints/`.
After a successful training run, copy the checkpoint over:

```bash
# META_CRITIC
cp training/checkpoints/meta_critic_best.pt meta_critic/models/meta_critic.pt

# IMITATION
cp training/checkpoints/imitation_best.pt meta_critic/models/imitation.pt
```

Then re-run the relevant inference launch and the new model is live.

---

## Inspect HDF5 files

```bash
# Quick summary of one file
python training/inspect_h5.py training/data/run_20260410_150049.h5

# Full diagnostic dump (per-critic stats, label distribution, frame counts)
python training/diagnose_h5.py training/data/run_20260410_150049.h5
```

---

## Evaluate a trained model offline

```bash
python training/evaluate.py \
    --method META_CRITIC \
    --model meta_critic/models/meta_critic.pt \
    --data-dir training/data
```

Reports: validation MSE, per-critic weight distribution histogram, cosine
similarity vs ideal weight labels, prediction-vs-label scatter plots.

---

## File reference

| File                | Purpose                                                          |
|---------------------|------------------------------------------------------------------|
| `train.py`          | Entry point. `--method` flag selects META_CRITIC or IMITATION.   |
| `dataset.py`        | `MetaCriticDataset`, `ImitationDataset`. Includes per-critic min-max normalization in `recover_ideal_weights()` for META_CRITIC labels. |
| `model.py`          | `MetaCriticMLP` (410 → 10 softmax), `ImitationMLP` (410 → 2 linear). Both ~148K params. |
| `evaluate.py`       | Offline metrics on a held-out dataset.                          |
| `inspect_h5.py`     | Quick tabular summary of a single HDF5 file.                    |
| `diagnose_h5.py`    | Detailed per-frame and per-critic statistics.                   |
| `rl_finetune.py`    | **Optional** PPO fine-tuning on top of META_CRITIC. Not used in the thesis main result. |
| `data/`             | META_CRITIC HDF5 files (with critic-score matrices)             |
| `checkpoints/`      | Best validation-loss checkpoints                                 |
| `figures/`          | Loss curves                                                      |

---

## Key learnings & gotchas

- **Training data must come from `vf_robot_controller` in COLLECT mode for META_CRITIC.** DWB and Nav2 MPPI do not produce the N×K per-critic score matrix.
- **The plugin ID must be `FollowPath`** (not `VFController`) to match Nav2's default Behavior Tree, which hardcodes that ID.
- **`conda activate dl` must precede ROS sourcing** in any terminal that runs Python ML nodes — otherwise PyTorch is not on the path.
- **Model paths in inference launches are absolute** (resolved at launch time via `get_package_share_directory`). After deploying a new model, you must re-launch — the inference node loads the model once at startup.
- **Per-critic min-max normalization in `dataset.py`** addresses a scale imbalance bug where SmoothnessCritic raw scores (0–586,000 range) dominate label generation. This fix lives at line 63 (`normalize_score_matrix`) and is applied inside `recover_ideal_weights()` at line 124.
- **Watch for `__pycache__` drift.** The training/ directory previously had a `__pycache__` of an old `dataset.py` that contained unfixed normalisation code, which silently masked changes during debugging. If you see weird behaviour after editing, `rm -rf training/__pycache__`.

## Known issues

- **META_CRITIC inference produces near-constant weight output** at runtime,
  with SmoothnessCritic ≈ 0.58 and PathFollowCritic ≈ 0.37 absorbing 95% of
  the weight regardless of the local scene. Sums to 1.0 (softmax intact)
  but vector barely changes between cycles. Most likely cause: input
  features are not normalised at training or inference time, only labels
  are. Fix path: z-score normalisation of the 410-dim feature vector with
  statistics computed over the training set, applied identically in
  `feature_extractor.py` (or inside the model's forward pass via a fixed
  pre-norm layer). Three diagnostic scripts and a fix design are documented
  in the project chat history; pick this up before the next paper-quality
  training run.

- **`imitation.pt` not yet trained.** To produce it: collect IMITATION data
  via `imitation_collect_launch.py` while running a teacher controller,
  then `python training/train.py --method IMITATION`, then deploy with `cp`.
