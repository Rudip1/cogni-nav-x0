"""
dataset.py — HDF5 DataLoader for meta-critic supervised training.

Each HDF5 file contains:
  features  (N, 410)   float32  — feature vectors from feature_extractor.py
  scalars   (N, 11)    float32  — [timestamp, rx, ry, rtheta, vx, vz,
                                    goal_dist, goal_heading, best_idx,
                                    n_candidates, n_critics]
  scores    (N, M)     float32  — flat critic score matrix (n_candidates * n_critics)

Label generation (Approach A — hindsight trajectory ranking):
  For each timestep t, look at the critic score matrix.
  Reshape to (n_candidates, n_critics).
  Apply uniform weights to get total score per candidate.
  best_idx = argmin(total_scores)  ← lowest cost = best trajectory
  Label = best_idx  (integer class, 0..n_candidates-1)

The network learns: given features s_t, predict which trajectory would be best.
At inference time the network outputs weights, not trajectory indices directly —
the weights are used by CriticManager to re-score trajectories adaptively.
"""

import os
import glob
import numpy as np
import torch
from torch.utils.data import Dataset

try:
    import h5py
    H5PY_AVAILABLE = True
except ImportError:
    H5PY_AVAILABLE = False


# Scalars layout indices (matches data_recorder.cpp header)
IDX_BEST_IDX      = 8
IDX_N_CANDIDATES  = 9
IDX_N_CRITICS     = 10


class MetaCriticDataset(Dataset):
    """
    Loads all HDF5 files from data_dir and generates training samples.

    Each sample: (feature_vector, best_trajectory_idx)
      feature_vector : torch.Tensor (410,)  float32
      best_idx       : torch.Tensor ()      int64

    Args:
        data_dir  : directory containing run_*.h5 files
        horizon   : look-ahead steps for hindsight quality (not used in
                    Approach A — kept for future Approach B extension)
        n_critics : expected number of critics (used to reshape score matrix)
    """

    def __init__(
        self,
        data_dir:  str = 'training/data',
        horizon:   int = 10,
        n_critics: int = 10,
    ):
        if not H5PY_AVAILABLE:
            raise ImportError('h5py required: pip3 install h5py')

        self.horizon   = horizon
        self.n_critics = n_critics
        self.samples: list[tuple[np.ndarray, int]] = []

        files = sorted(glob.glob(os.path.join(data_dir, 'run_*.h5')))
        if not files:
            raise FileNotFoundError(f'No run_*.h5 files found in {data_dir}')

        for fp in files:
            self._load_file(fp)

        print(f'MetaCriticDataset: loaded {len(self.samples):,} samples '
              f'from {len(files)} file(s) in {data_dir}')

    def _load_file(self, filepath: str):
        with h5py.File(filepath, 'r') as f:
            if 'features' not in f or 'scores' not in f or 'scalars' not in f:
                print(f'  Skipping {filepath} — missing datasets')
                return

            features = f['features'][:]   # (N, 410)
            scalars  = f['scalars'][:]    # (N, 11)
            scores   = f['scores'][:]     # (N, M)

        T = features.shape[0]

        for t in range(T):
            feat_t    = features[t]                         # (410,)
            scalar_t  = scalars[t]                          # (11,)
            scores_t  = scores[t]                           # (M,)

            n_cand    = int(scalar_t[IDX_N_CANDIDATES])
            n_crit    = int(scalar_t[IDX_N_CRITICS])

            if n_cand <= 0 or n_crit <= 0:
                continue

            expected_len = n_cand * n_crit
            if len(scores_t) < expected_len:
                continue

            # Reshape score matrix: (n_candidates, n_critics)
            score_matrix = scores_t[:expected_len].reshape(n_cand, n_crit)

            # Approach A: uniform weighting to find best trajectory
            uniform_w = np.ones(n_crit, dtype=np.float32) / n_crit
            total_scores = score_matrix @ uniform_w          # (n_candidates,)
            best_idx = int(np.argmin(total_scores))          # lowest cost = best

            # Clip to valid range
            best_idx = min(best_idx, n_cand - 1)

            self.samples.append((
                feat_t.astype(np.float32),
                best_idx
            ))

    def __len__(self) -> int:
        return len(self.samples)

    def __getitem__(self, idx: int):
        features, label = self.samples[idx]
        return (
            torch.from_numpy(features),
            torch.tensor(label, dtype=torch.long)
        )

    def num_candidates(self) -> int:
        """Max trajectory index seen — used to set output dim for classification."""
        if not self.samples:
            return 200
        return max(label for _, label in self.samples) + 1


class DataStats:
    """Compute dataset statistics for normalisation diagnostics."""

    @staticmethod
    def compute(dataset: MetaCriticDataset) -> dict:
        if not dataset.samples:
            return {}
        features = np.stack([s[0] for s in dataset.samples])
        labels   = np.array([s[1] for s in dataset.samples])
        return {
            'n_samples':     len(dataset),
            'feature_mean':  features.mean(axis=0),
            'feature_std':   features.std(axis=0),
            'label_min':     int(labels.min()),
            'label_max':     int(labels.max()),
            'label_mean':    float(labels.mean()),
        }


if __name__ == '__main__':
    import sys
    data_dir = sys.argv[1] if len(sys.argv) > 1 else 'training/data'
    try:
        ds = MetaCriticDataset(data_dir=data_dir)
        stats = DataStats.compute(ds)
        print(f'Samples:    {stats["n_samples"]:,}')
        print(f'Label range: {stats["label_min"]} .. {stats["label_max"]}')
        feat, label = ds[0]
        print(f'Feature shape: {feat.shape}, dtype: {feat.dtype}')
        print(f'Label: {label}')
    except FileNotFoundError as e:
        print(f'Error: {e}')
        print('Run the controller in COLLECT mode first to generate training data.')
