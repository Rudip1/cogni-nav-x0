"""
dataset.py — HDF5 DataLoader for meta-critic supervised training.

─────────────────────────────────────────────────────────────────────────────
TRAINING METHODS
─────────────────────────────────────────────────────────────────────────────

IMITATION (baseline — behaviour cloning):
  Label : integer best_idx ∈ {0 .. N-1}
  Learns: given features s_t, predict which trajectory index is best
  Loss  : CrossEntropyLoss(logits, best_idx)
  Note  : cannot exceed performance of classical MPPI that generated the data

META_CRITIC (novel contribution — ideal weight recovery):
  Label : float32 vector w* ∈ R^K, w*≥0, sum=1
  Learns: given features s_t, output per-critic importance weights
  Loss  : MSELoss(predicted_weights, w*)
  Note  : interprets WHY each critic matters, produces the thesis key figure

Both methods:
  - Read the exact same HDF5 files (data collection is method-agnostic)
  - Use the same MetaCriticMLP network (identical architecture + runtime)
  - Deploy the same meta_critic.pt (C++ side never sees the difference)

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
IMITATION   = 'IMITATION'    # baseline — behaviour cloning
META_CRITIC = 'META_CRITIC'  # novel    — ideal weight recovery
VALID_METHODS = (IMITATION, META_CRITIC)

# ── Scalars layout (matches data_recorder.cpp header) ─────────────────────────
IDX_BEST_IDX      = 8
IDX_N_CANDIDATES  = 9
IDX_N_CRITICS     = 10


# ── Label generation functions ────────────────────────────────────────────────

def get_best_idx(scalar_t: np.ndarray, score_matrix: np.ndarray) -> int:
    """
    IMITATION label: index of the best trajectory.

    Uses the real best_idx recorded by the optimizer (argmin of weighted costs).
    Falls back to recomputing from the score matrix with uniform weights if
    the recorded value is out of range (e.g. old data collected before Phase 1 fix).

    Returns: int in [0, n_candidates-1]
    """
    recorded = int(scalar_t[IDX_BEST_IDX])
    n_cand   = score_matrix.shape[0]

    if 0 <= recorded < n_cand:
        return recorded

    # Fallback: recompute with uniform weights (for pre-Phase-1 data)
    n_crit   = score_matrix.shape[1]
    uniform  = np.ones(n_crit, dtype=np.float32) / n_crit
    totals   = score_matrix @ uniform
    return int(np.argmin(totals))


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

    This is a closed-form approximation of the full linear programme from the
    thesis (Chapter 6.4.2). It is O(N*K) and computes in microseconds.

    Returns: float32 array of shape (K,), sums to 1.0
    """
    n_cand, n_crit = score_matrix.shape

    best_scores  = score_matrix[best_idx]                        # (K,)
    mask         = np.ones(n_cand, dtype=bool)
    mask[best_idx] = False
    other_scores = score_matrix[mask]                            # (N-1, K)

    if other_scores.shape[0] == 0:
        # Only one trajectory — uniform weights
        return np.ones(n_crit, dtype=np.float32) / n_crit

    # margin_k > 0 means critic k gives lower cost to best than to others
    margins = other_scores.mean(axis=0) - best_scores            # (K,)
    margins = np.clip(margins, 0.0, None)                        # keep positives only

    total = margins.sum()
    if total < 1e-9:
        # No critic differentiates — fall back to uniform
        return np.ones(n_crit, dtype=np.float32) / n_crit

    return (margins / total).astype(np.float32)


# ── Dataset class ─────────────────────────────────────────────────────────────

class MetaCriticDataset(Dataset):
    """
    HDF5 dataset for meta-critic training.

    Args:
        data_dir : directory containing run_*.h5 files
        method   : IMITATION or META_CRITIC (use module constants)
        n_critics: expected number of critics — must match K in HDF5 files

    Returns per __getitem__:
        IMITATION   → (features: float32[410], label: int64 scalar)
        META_CRITIC → (features: float32[410], label: float32[K])
    """

    def __init__(
        self,
        data_dir:  str = 'training/data',
        method:    str = IMITATION,
        n_critics: int = 10,
    ):
        if not H5PY_AVAILABLE:
            raise ImportError('h5py required: pip3 install h5py')

        if method not in VALID_METHODS:
            raise ValueError(
                f'method must be one of {VALID_METHODS}, got: {method}')

        self.method    = method
        self.n_critics = n_critics

        # samples stores (feature_np, label) where label type depends on method:
        #   IMITATION   → int
        #   META_CRITIC → np.ndarray float32 (K,)
        self.samples: list = []

        files = sorted(glob.glob(os.path.join(data_dir, 'run_*.h5')))
        if not files:
            raise FileNotFoundError(
                f'No run_*.h5 files found in {data_dir}\n'
                f'Run the controller in COLLECT mode first.')

        for fp in files:
            self._load_file(fp)

        print(
            f'[{self.method}] MetaCriticDataset: {len(self.samples):,} samples '
            f'from {len(files)} file(s) in {data_dir}')

    def _load_file(self, filepath: str):
        with h5py.File(filepath, 'r') as f:
            missing = [k for k in ('features', 'scores', 'scalars') if k not in f]
            if missing:
                print(f'  Skipping {os.path.basename(filepath)} — missing: {missing}')
                return

            features = f['features'][:]   # (T, 410)
            scalars  = f['scalars'][:]    # (T, 11)
            scores   = f['scores'][:]     # (T, N*K)

        T = features.shape[0]
        skipped = 0

        for t in range(T):
            feat_t   = features[t]                    # (410,)
            scalar_t = scalars[t]                     # (11,)
            scores_t = scores[t]                      # (N*K,)

            n_cand = int(scalar_t[IDX_N_CANDIDATES])
            n_crit = int(scalar_t[IDX_N_CRITICS])

            # Validate dimensions
            if n_cand <= 0 or n_crit <= 0:
                skipped += 1
                continue
            if n_crit != self.n_critics:
                skipped += 1
                continue
            expected = n_cand * n_crit
            if len(scores_t) < expected:
                skipped += 1
                continue

            # Reshape to (N, K) matrix
            score_matrix = scores_t[:expected].reshape(n_cand, n_crit)

            # ── Label generation — branches by method ────────────────────────
            best_idx = get_best_idx(scalar_t, score_matrix)

            if self.method == IMITATION:
                # IMITATION: integer class label
                label = best_idx

            else:  # META_CRITIC
                # META_CRITIC: ideal weight vector
                label = recover_ideal_weights(score_matrix, best_idx)

            self.samples.append((feat_t.astype(np.float32), label))

        if skipped:
            print(f'  {os.path.basename(filepath)}: skipped {skipped}/{T} timesteps')

    def __len__(self) -> int:
        return len(self.samples)

    def __getitem__(self, idx: int):
        features, label = self.samples[idx]
        feat_tensor = torch.from_numpy(features)

        if self.method == IMITATION:
            # Long tensor for CrossEntropyLoss
            return feat_tensor, torch.tensor(label, dtype=torch.long)
        else:
            # Float tensor for MSELoss
            return feat_tensor, torch.from_numpy(label)

    def label_info(self) -> str:
        """Human-readable label summary for logging."""
        if not self.samples:
            return 'empty dataset'
        if self.method == IMITATION:
            labels = [s[1] for s in self.samples]
            return (f'IMITATION labels — int, range [{min(labels)}, {max(labels)}], '
                    f'mean={sum(labels)/len(labels):.1f}')
        else:
            weights = np.stack([s[1] for s in self.samples])
            return (f'META_CRITIC labels — float32[{self.n_critics}], '
                    f'mean_entropy={float(-np.sum(weights * np.log(weights + 1e-9), axis=1).mean()):.3f}')


class DataStats:
    """Compute dataset statistics — works for both methods."""

    @staticmethod
    def compute(dataset: MetaCriticDataset) -> dict:
        if not dataset.samples:
            return {}
        features = np.stack([s[0] for s in dataset.samples])
        return {
            'method':        dataset.method,
            'n_samples':     len(dataset),
            'feature_mean':  float(features.mean()),
            'feature_std':   float(features.std()),
            'label_info':    dataset.label_info(),
        }


# ── CLI entry point ───────────────────────────────────────────────────────────

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Inspect training dataset')
    parser.add_argument('--method',   default=IMITATION,
                        choices=VALID_METHODS,
                        help='Training method: IMITATION or META_CRITIC')
    parser.add_argument('--data-dir', default='training/data',
                        help='Directory containing run_*.h5 files')
    parser.add_argument('--n-critics', type=int, default=10)
    args = parser.parse_args()

    try:
        ds    = MetaCriticDataset(
            data_dir=args.data_dir,
            method=args.method,
            n_critics=args.n_critics)
        stats = DataStats.compute(ds)
        print(f'Method:     {stats["method"]}')
        print(f'Samples:    {stats["n_samples"]:,}')
        print(f'Features:   mean={stats["feature_mean"]:.4f} std={stats["feature_std"]:.4f}')
        print(f'Labels:     {stats["label_info"]}')
        feat, label = ds[0]
        print(f'Sample[0]:  feat={feat.shape} {feat.dtype}  label={label.shape} {label.dtype}')
    except FileNotFoundError as e:
        print(f'Error: {e}')
