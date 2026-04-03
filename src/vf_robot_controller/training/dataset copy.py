"""
dataset.py — HDF5 DataLoader for meta-critic supervised training.

─────────────────────────────────────────────────────────────────────────────
TRAINING METHODS
─────────────────────────────────────────────────────────────────────────────

META_CRITIC (novel contribution — ideal weight recovery):
  Label : float32 vector w* ∈ R^K, w*≥0, sum=1
  Learns: given features s_t, output per-critic importance weights
  Loss  : MSELoss(predicted_weights, w*)
  Note  : interprets WHY each critic matters, produces the thesis key figure

─────────────────────────────────────────────────────────────────────────────
HDF5 FILE LAYOUT (written by data_logger.py from DataRecorder C++ output)
─────────────────────────────────────────────────────────────────────────────
  features  (T, 410)    float32  — feature vector per timestep
  scalars   (T, 11)     float32  — [timestamp, rx, ry, rtheta, vx, vz,
                                     goal_dist, goal_heading, best_idx,
                                     n_candidates, n_critics]
  scores    (T, N*K)    float32  — per-critic score matrix, row-major
                                   [traj0_c0, traj0_c1...traj0_cK, traj1_c0...]

Scalars index constants (must match data_recorder.cpp HEADER_SIZE layout):
  IDX_BEST_IDX     = 8   ← real argmin from optimizer (Phase 1 fix)
  IDX_N_CANDIDATES = 9
  IDX_N_CRITICS    = 10
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
META_CRITIC = "META_CRITIC"  # novel — ideal weight recovery
VALID_METHODS = (META_CRITIC,)

# ── Scalars layout (matches data_recorder.cpp header) ─────────────────────────
IDX_BEST_IDX = 8
IDX_N_CANDIDATES = 9
IDX_N_CRITICS = 10


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

    Method: for each critic k, compute how much it helps select best_idx:
      margin_k = mean(scores[other_trajs, k]) - scores[best_idx, k]
    A large positive margin means critic k strongly prefers the best trajectory.
    Normalise to a probability simplex (sum=1, all≥0).

    IMPORTANT: Handles inf values by replacing them with a large finite value
    before computing margins. This prevents NaN in the output.

    Returns: float32 array of shape (K,), sums to 1.0, or None if invalid
    """
    n_cand, n_crit = score_matrix.shape

    # Check for any inf or nan in the score matrix
    if not np.isfinite(score_matrix).all():
        # Replace inf with large finite value, nan with 0
        score_matrix = score_matrix.copy()
        score_matrix = np.nan_to_num(score_matrix, nan=0.0, posinf=1e6, neginf=-1e6)

    best_scores = score_matrix[best_idx]  # (K,)
    mask = np.ones(n_cand, dtype=bool)
    mask[best_idx] = False
    other_scores = score_matrix[mask]  # (N-1, K)

    if other_scores.shape[0] == 0:
        # Only one trajectory — uniform weights
        return np.ones(n_crit, dtype=np.float32) / n_crit

    # margin_k > 0 means critic k gives lower cost to best than to others
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
    Compute best trajectory index from score matrix using uniform critic weights.

    Args:
        score_matrix: (N, K) array of per-critic scores

    Returns:
        int: index of trajectory with lowest total cost
    """
    n_cand, n_crit = score_matrix.shape

    # Handle inf values
    safe_scores = np.nan_to_num(score_matrix, nan=1e9, posinf=1e9, neginf=-1e9)

    # Uniform weights
    uniform = np.ones(n_crit, dtype=np.float32) / n_crit
    totals = safe_scores @ uniform

    return int(np.argmin(totals))


# ── Dataset class ─────────────────────────────────────────────────────────────


class MetaCriticDataset(Dataset):
    """
    HDF5 dataset for meta-critic training.

    Args:
        data_dir : directory containing run_*.h5 files
        n_critics: expected number of critics — must match K in HDF5 files

    Returns per __getitem__:
        (features: float32[410], label: float32[K])
    """

    def __init__(
        self,
        data_dir: str = "training/data",
        method: str = META_CRITIC,  # kept for compatibility, ignored
        n_critics: int = 10,
    ):
        if not H5PY_AVAILABLE:
            raise ImportError("h5py required: pip3 install h5py")

        self.method = META_CRITIC  # Always META_CRITIC
        self.n_critics = n_critics

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

        for fp in files:
            self._load_file(fp)

        print(
            f"[{self.method}] MetaCriticDataset: {len(self.samples):,} samples "
            f"from {len(files)} file(s) in {data_dir}"
        )

        if self.skipped_inf > 0:
            print(f"  Skipped {self.skipped_inf} samples with inf scores")
        if self.skipped_dim > 0:
            print(f"  Skipped {self.skipped_dim} samples with dimension mismatch")
        if self.skipped_nan_label > 0:
            print(f"  Skipped {self.skipped_nan_label} samples with NaN labels")

    def _load_file(self, filepath: str):
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

            # Compute best_idx from scores (more robust than using recorded value)
            best_idx = get_best_idx_from_scores(score_matrix)

            # Generate ideal weight label
            label = recover_ideal_weights(score_matrix, best_idx)

            # Skip if label contains NaN
            if not np.isfinite(label).all():
                self.skipped_nan_label += 1
                continue

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
        return (
            f"META_CRITIC labels — float32[{self.n_critics}], "
            f"mean_entropy={float(entropy):.3f}"
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
