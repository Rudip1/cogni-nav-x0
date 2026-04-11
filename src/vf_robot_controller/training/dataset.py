"""
dataset.py — HDF5 DataLoader for meta-critic supervised training.

─────────────────────────────────────────────────────────────────────────────
TRAINING METHOD: META_CRITIC (novel contribution — ideal weight recovery)
─────────────────────────────────────────────────────────────────────────────

  Label : float32 vector w* ∈ R^K, w*≥0, sum=1
  Learns: given features s_t, output per-critic importance weights
  Loss  : MSELoss(predicted_weights, w*)

─────────────────────────────────────────────────────────────────────────────
HDF5 FILE LAYOUT (written by data_logger.py from DataRecorder C++ output)
─────────────────────────────────────────────────────────────────────────────
  features  (T, 410)    float32  — feature vector per timestep
  scalars   (T, 11)     float32  — [timestamp, rx, ry, rtheta, vx, vz,
                                     goal_dist, goal_heading, best_idx,
                                     n_candidates, n_critics]
  scores    (T, N*K)    float32  — per-critic score matrix, row-major

─────────────────────────────────────────────────────────────────────────────
FIX: Per-critic normalization to handle scale imbalance
─────────────────────────────────────────────────────────────────────────────
Problem: SmoothnessCritic can return values up to 500,000+ while other
critics return values < 1.0. This causes SmoothnessCritic to dominate
the ideal weight computation (99%+ weight).

Solution: Normalize each critic's scores to [0, 1] range before computing
margins. This ensures all critics have equal opportunity to influence
the weight vector.
─────────────────────────────────────────────────────────────────────────────
"""

import os
import sys
import glob
import argparse
import numpy as np
import torch
from torch.utils.data import Dataset

try:
    import h5py

    H5PY_AVAILABLE = True
except ImportError:
    H5PY_AVAILABLE = False


# ── Training method constants ─────────────────────────────────────────────────
META_CRITIC = "META_CRITIC"
VALID_METHODS = (META_CRITIC,)

# ── Scalars layout (matches data_recorder.cpp header) ─────────────────────────
IDX_BEST_IDX = 8
IDX_N_CANDIDATES = 9
IDX_N_CRITICS = 10


# ── Normalization helper ──────────────────────────────────────────────────────


def normalize_score_matrix(score_matrix: np.ndarray) -> np.ndarray:
    """
    Normalize each critic's scores to [0, 1] range.

    This handles the scale imbalance problem where some critics (e.g.,
    SmoothnessCritic) can return values 1000x larger than others.

    Args:
        score_matrix: (N, K) array of raw scores

    Returns:
        (N, K) array with each column normalized to [0, 1]
    """
    # Handle inf values first
    matrix = score_matrix.copy()
    matrix = np.nan_to_num(matrix, nan=0.0, posinf=1e6, neginf=0.0)

    # Normalize each critic column independently
    normalized = np.zeros_like(matrix)
    for k in range(matrix.shape[1]):
        col = matrix[:, k]
        col_min = col.min()
        col_max = col.max()

        if col_max - col_min > 1e-9:
            # Scale to [0, 1]
            normalized[:, k] = (col - col_min) / (col_max - col_min)
        else:
            # All values same — set to 0 (no discrimination)
            normalized[:, k] = 0.0

    return normalized


# ── Label generation functions ────────────────────────────────────────────────


def recover_ideal_weights(
    score_matrix: np.ndarray,
    best_idx: int,
) -> np.ndarray:
    """
    META_CRITIC label: ideal critic weight vector w* ∈ R^K.

    Given the (N, K) per-critic score matrix and the best trajectory index,
    recover the weight vector that best differentiates the best trajectory
    from all others.

    IMPORTANT: Normalizes scores per-critic before computing margins to
    handle scale imbalance between critics.

    Method: for each critic k, compute how much it helps select best_idx:
      margin_k = mean(scores[other_trajs, k]) - scores[best_idx, k]
    A large positive margin means critic k strongly prefers the best trajectory.
    Normalise to a probability simplex (sum=1, all≥0).

    Returns: float32 array of shape (K,), sums to 1.0, or None if invalid
    """
    n_cand, n_crit = score_matrix.shape

    # CRITICAL: Normalize scores to handle scale imbalance
    score_matrix = normalize_score_matrix(score_matrix)

    best_scores = score_matrix[best_idx]  # (K,)
    mask = np.ones(n_cand, dtype=bool)
    mask[best_idx] = False
    other_scores = score_matrix[mask]  # (N-1, K)

    if other_scores.shape[0] == 0:
        # Only one trajectory — uniform weights
        return np.ones(n_crit, dtype=np.float32) / n_crit

    # margin_k > 0 means critic k gives lower (better) normalized score to best
    margins = other_scores.mean(axis=0) - best_scores  # (K,)
    margins = np.clip(margins, 0.0, None)  # keep positives only

    # Handle case where margins are all zero or contain inf/nan
    if not np.isfinite(margins).all():
        return np.ones(n_crit, dtype=np.float32) / n_crit

    total = margins.sum()
    if total < 1e-9:
        # No critic differentiates — fall back to uniform
        return np.ones(n_crit, dtype=np.float32) / n_crit

    weights = (margins / total).astype(np.float32)

    # Final sanity check
    if not np.isfinite(weights).all() or abs(weights.sum() - 1.0) > 0.01:
        return np.ones(n_crit, dtype=np.float32) / n_crit

    return weights


def get_best_idx_from_scores(score_matrix: np.ndarray) -> int:
    """
    Compute best trajectory index from normalized score matrix.

    Args:
        score_matrix: (N, K) array of per-critic scores

    Returns:
        int: index of trajectory with lowest total normalized cost
    """
    # Normalize first to handle scale imbalance
    normalized = normalize_score_matrix(score_matrix)

    n_cand, n_crit = normalized.shape

    # Uniform weights for trajectory selection
    uniform = np.ones(n_crit, dtype=np.float32) / n_crit
    totals = normalized @ uniform

    return int(np.argmin(totals))


# ── Dataset class ─────────────────────────────────────────────────────────────


class MetaCriticDataset(Dataset):
    """
    HDF5 dataset for meta-critic training.

    Args:
        data_dir : directory containing run_*.h5 files
        n_critics: expected number of critics — must match K in HDF5 files
        normalize: whether to apply z-score normalization to features

    Returns per __getitem__:
        (features: float32[410], label: float32[K])
    """

    def __init__(
        self,
        data_dir: str = "training/data",
        method: str = META_CRITIC,  # kept for compatibility, ignored
        n_critics: int = 10,
        normalize: bool = True,
    ):
        if not H5PY_AVAILABLE:
            raise ImportError("h5py required: pip3 install h5py")

        self.method = META_CRITIC  # Always META_CRITIC
        self.n_critics = n_critics
        self.normalize = normalize
        self.feature_mean = None
        self.feature_std = None

        # samples stores (feature_np, label) where label is np.ndarray float32 (K,)
        self.samples: list = []

        # Track statistics
        self.skipped_inf = 0
        self.skipped_dim = 0
        self.skipped_nan_label = 0

        files = sorted(glob.glob(os.path.join(data_dir, "run_*.h5")))
        if not files:
            raise FileNotFoundError(
                f"No run_*.h5 files found in {data_dir}\n"
                f"Run the controller in COLLECT mode first."
            )

        # First pass: collect all features to compute normalization stats
        if self.normalize:
            all_features = []
            for fp in files:
                self._load_file(fp, collect_features=True, all_features=all_features)

            if all_features:
                features_stacked = np.vstack(all_features)
                self.feature_mean = features_stacked.mean(axis=0)
                self.feature_std = features_stacked.std(axis=0) + 1e-8

                # Save statistics for deployment
                stats_path = os.path.join(os.path.dirname(data_dir), "meta_critic_stats.npz")
                np.savez(stats_path, mean=self.feature_mean, std=self.feature_std)
                print(f"  Saved feature normalization stats to {stats_path}")
                print(f"  Feature mean range: [{self.feature_mean.min():.4f}, {self.feature_mean.max():.4f}]")
                print(f"  Feature std range:  [{self.feature_std.min():.4f}, {self.feature_std.max():.4f}]")

                # Apply normalization to all samples
                for i in range(len(self.samples)):
                    feat, label = self.samples[i]
                    feat_normalized = (feat - self.feature_mean) / self.feature_std
                    self.samples[i] = (feat_normalized.astype(np.float32), label)
        else:
            for fp in files:
                self._load_file(fp, collect_features=False)

        print(
            f"[{self.method}] MetaCriticDataset: {len(self.samples):,} samples "
            f"from {len(files)} file(s) in {data_dir}"
        )

        if self.skipped_inf > 0:
            print(f"  Skipped {self.skipped_inf} samples with excessive inf scores")
        if self.skipped_dim > 0:
            print(f"  Skipped {self.skipped_dim} samples with dimension mismatch")
        if self.skipped_nan_label > 0:
            print(f"  Skipped {self.skipped_nan_label} samples with NaN labels")

    def _load_file(self, filepath: str, collect_features: bool = False, all_features: list = None):
        with h5py.File(filepath, "r") as f:
            missing = [k for k in ("features", "scores", "scalars") if k not in f]
            if missing:
                print(f"  Skipping {os.path.basename(filepath)} — missing: {missing}")
                return

            features = f["features"][:]  # (T, 410)
            scalars = f["scalars"][:]  # (T, 11)
            scores = f["scores"][:]  # (T, N*K)

        T = features.shape[0]
        loaded = 0

        for t in range(T):
            feat_t = features[t]  # (410,)
            scalar_t = scalars[t]  # (11,)
            scores_t = scores[t]  # (N*K,)

            n_cand = int(scalar_t[IDX_N_CANDIDATES])
            n_crit = int(scalar_t[IDX_N_CRITICS])

            # Validate dimensions
            if n_cand <= 0 or n_crit <= 0:
                self.skipped_dim += 1
                continue
            if n_crit != self.n_critics:
                self.skipped_dim += 1
                continue
            expected = n_cand * n_crit
            if len(scores_t) < expected:
                self.skipped_dim += 1
                continue

            # Reshape to (N, K) matrix
            score_matrix = scores_t[:expected].reshape(n_cand, n_crit)

            # Check for excessive inf values (skip if >50% are inf)
            inf_ratio = np.isinf(score_matrix).sum() / score_matrix.size
            if inf_ratio > 0.5:
                self.skipped_inf += 1
                continue

            # Use recorded best_idx if valid, otherwise compute from scores
            recorded_best = int(scalar_t[IDX_BEST_IDX])
            if 0 <= recorded_best < n_cand:
                best_idx = recorded_best
            else:
                best_idx = get_best_idx_from_scores(score_matrix)

            # Generate ideal weight label (with normalization)
            label = recover_ideal_weights(score_matrix, best_idx)

            # Skip if label contains NaN
            if not np.isfinite(label).all():
                self.skipped_nan_label += 1
                continue

            if collect_features:
                all_features.append(feat_t.astype(np.float32))
            self.samples.append((feat_t.astype(np.float32), label))
            loaded += 1

        print(f"  {os.path.basename(filepath)}: loaded {loaded}/{T} timesteps")

    def __len__(self) -> int:
        return len(self.samples)

    def __getitem__(self, idx: int):
        features, label = self.samples[idx]
        feat_tensor = torch.from_numpy(features)
        label_tensor = torch.from_numpy(label)
        return feat_tensor, label_tensor

    def label_info(self) -> str:
        """Human-readable label summary for logging."""
        if not self.samples:
            return "empty dataset"
        weights = np.stack([s[1] for s in self.samples])
        # Compute entropy safely
        weights_safe = np.clip(weights, 1e-9, 1.0)
        entropy = -np.sum(weights_safe * np.log(weights_safe), axis=1).mean()

        # Also show mean weight per critic
        mean_weights = weights.mean(axis=0)
        critics = [
            "Obst",
            "Vol",
            "Dyn",
            "Path",
            "Smooth",
            "Goal",
            "Vel",
            "Corr",
            "Clear",
            "Osc",
        ]
        weight_str = ", ".join([f"{c}={w:.2f}" for c, w in zip(critics, mean_weights)])

        return (
            f"META_CRITIC labels — float32[{self.n_critics}], "
            f"mean_entropy={float(entropy):.3f}\n"
            f"  Mean weights: {weight_str}"
        )


class DataStats:
    """Compute dataset statistics."""

    @staticmethod
    def compute(dataset: MetaCriticDataset) -> dict:
        if not dataset.samples:
            return {}
        features = np.stack([s[0] for s in dataset.samples])
        return {
            "method": dataset.method,
            "n_samples": len(dataset),
            "feature_mean": float(features.mean()),
            "feature_std": float(features.std()),
            "label_info": dataset.label_info(),
        }


# ── CLI entry point ───────────────────────────────────────────────────────────

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Inspect training dataset")
    parser.add_argument(
        "--method",
        default=META_CRITIC,
        help="Training method (only META_CRITIC supported)",
    )
    parser.add_argument(
        "--data-dir",
        default="training/data",
        help="Directory containing run_*.h5 files",
    )
    parser.add_argument("--n-critics", type=int, default=10)
    args = parser.parse_args()

    try:
        ds = MetaCriticDataset(
            data_dir=args.data_dir, method=args.method, n_critics=args.n_critics
        )
        stats = DataStats.compute(ds)
        print(f'Method:     {stats["method"]}')
        print(f'Samples:    {stats["n_samples"]:,}')
        print(
            f'Features:   mean={stats["feature_mean"]:.4f} std={stats["feature_std"]:.4f}'
        )
        print(f'Labels:     {stats["label_info"]}')
        if len(ds) > 0:
            feat, label = ds[0]
            print(
                f"Sample[0]:  feat={feat.shape} {feat.dtype}  label={label.shape} {label.dtype}"
            )
            print(f"            label values: {label.numpy()}")
    except FileNotFoundError as e:
        print(f"Error: {e}")


# ── Imitation Dataset ─────────────────────────────────────────────────────────

IMITATION = "IMITATION"


class ImitationDataset(Dataset):
    """
    HDF5 dataset for IMITATION (behaviour cloning) training.

    Reads from imitation/run_*.h5 files written by imitation_data_logger.py.
    Does not require or read critic score matrices.

    Returns per __getitem__:
        (features: float32[410], label: float32[2])
        label = [linear_x, angular_z] from teacher controller /cmd_vel
    """

    def __init__(self, data_dir: str = 'training/imitation'):
        if not H5PY_AVAILABLE:
            raise ImportError('h5py required: pip3 install h5py')

        self.method = IMITATION
        self.samples: list = []

        files = sorted(glob.glob(os.path.join(data_dir, 'run_*.h5')))
        if not files:
            raise FileNotFoundError(
                f'No run_*.h5 files found in {data_dir}\n'
                f'Run imitation_collect_launch.py first.')

        for fp in files:
            self._load_file(fp)

        print(f'[IMITATION] ImitationDataset: {len(self.samples):,} samples '
              f'from {len(files)} file(s) in {data_dir}')

    def _load_file(self, filepath: str):
        with h5py.File(filepath, 'r') as f:
            missing = [k for k in ('features', 'cmd_vel') if k not in f]
            if missing:
                print(f'  Skipping {os.path.basename(filepath)} — missing: {missing}')
                return

            features = f['features'][:]   # (T, 410)
            cmd_vel  = f['cmd_vel'][:]    # (T, 2)

        T = features.shape[0]
        loaded = 0
        for t in range(T):
            feat = features[t].astype(np.float32)
            vel  = cmd_vel[t].astype(np.float32)
            if np.isfinite(feat).all() and np.isfinite(vel).all():
                self.samples.append((feat, vel))
                loaded += 1

        print(f'  {os.path.basename(filepath)}: loaded {loaded}/{T} timesteps')

    def __len__(self) -> int:
        return len(self.samples)

    def __getitem__(self, idx: int):
        features, label = self.samples[idx]
        return torch.from_numpy(features), torch.from_numpy(label)
