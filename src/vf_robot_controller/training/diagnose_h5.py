#!/usr/bin/env python3
"""
Diagnostic script to inspect HDF5 training data.

Run this on your machine:
  python3 diagnose_h5.py training/data/run_20260331_121635.h5

It will show:
  1. Dataset shapes and dtypes
  2. Sample of scalar values (including n_candidates, n_critics)
  3. Why samples are being filtered out
"""

import sys
import numpy as np

try:
    import h5py
except ImportError:
    print("ERROR: h5py not installed. Run: pip3 install h5py")
    sys.exit(1)

# Scalars layout (from data_recorder.cpp)
SCALAR_NAMES = [
    "timestamp",      # 0
    "rx",             # 1
    "ry",             # 2
    "rtheta",         # 3
    "vx",             # 4
    "vz",             # 5
    "goal_dist",      # 6
    "goal_heading",   # 7
    "best_idx",       # 8
    "n_candidates",   # 9
    "n_critics",      # 10
]

IDX_N_CANDIDATES = 9
IDX_N_CRITICS = 10


def diagnose(filepath: str):
    print(f"\n{'='*60}")
    print(f"Diagnosing: {filepath}")
    print(f"{'='*60}\n")
    
    with h5py.File(filepath, 'r') as f:
        # 1. Show available datasets
        print("1. DATASETS IN FILE:")
        for name in f.keys():
            ds = f[name]
            print(f"   {name}: shape={ds.shape}, dtype={ds.dtype}")
        print()
        
        # Check required datasets exist
        if 'scalars' not in f:
            print("ERROR: 'scalars' dataset not found!")
            return
        if 'scores' not in f:
            print("ERROR: 'scores' dataset not found!")
            return
        if 'features' not in f:
            print("ERROR: 'features' dataset not found!")
            return
            
        scalars = f['scalars'][:]
        scores = f['scores'][:]
        features = f['features'][:]
        
        T = scalars.shape[0]
        print(f"Total timesteps: {T}")
        print()
        
        # 2. Show first few scalar rows
        print("2. SAMPLE SCALAR VALUES (first 5 rows):")
        print(f"   {'idx':<4} " + " ".join(f"{n:<12}" for n in SCALAR_NAMES))
        print("   " + "-" * 150)
        for t in range(min(5, T)):
            row = scalars[t]
            print(f"   {t:<4} " + " ".join(f"{v:<12.4f}" for v in row))
        print()
        
        # 3. Analyze n_candidates and n_critics distribution
        print("3. n_candidates AND n_critics DISTRIBUTION:")
        n_cands = scalars[:, IDX_N_CANDIDATES]
        n_crits = scalars[:, IDX_N_CRITICS]
        
        print(f"   n_candidates: min={n_cands.min():.0f}, max={n_cands.max():.0f}, "
              f"mean={n_cands.mean():.2f}, unique={np.unique(n_cands)[:10]}...")
        print(f"   n_critics:    min={n_crits.min():.0f}, max={n_crits.max():.0f}, "
              f"mean={n_crits.mean():.2f}, unique={np.unique(n_crits)[:10]}...")
        print()
        
        # 4. Count filtering reasons
        print("4. SAMPLE FILTERING ANALYSIS:")
        skip_n_cand_zero = 0
        skip_n_crit_zero = 0
        skip_scores_short = 0
        valid = 0
        
        for t in range(T):
            n_cand = int(scalars[t, IDX_N_CANDIDATES])
            n_crit = int(scalars[t, IDX_N_CRITICS])
            scores_t = scores[t]
            
            if n_cand <= 0:
                skip_n_cand_zero += 1
                continue
            if n_crit <= 0:
                skip_n_crit_zero += 1
                continue
            
            expected_len = n_cand * n_crit
            actual_len = len(scores_t)
            
            # Handle padded zeros
            non_zero = np.count_nonzero(scores_t)
            
            if actual_len < expected_len:
                skip_scores_short += 1
                continue
                
            valid += 1
        
        print(f"   Skipped (n_candidates <= 0): {skip_n_cand_zero}")
        print(f"   Skipped (n_critics <= 0):    {skip_n_crit_zero}")
        print(f"   Skipped (scores too short):  {skip_scores_short}")
        print(f"   Valid samples:               {valid}")
        print()
        
        # 5. Show score array details
        print("5. SCORES ARRAY DETAILS:")
        print(f"   Shape: {scores.shape}")
        print(f"   First row (scores[0]):")
        print(f"     Length: {len(scores[0])}")
        print(f"     Non-zero count: {np.count_nonzero(scores[0])}")
        print(f"     First 20 values: {scores[0][:20]}")
        print()
        
        # 6. Show what the expected vs actual score lengths are
        print("6. EXPECTED vs ACTUAL SCORE LENGTHS (first 10 rows):")
        print(f"   {'t':<4} {'n_cand':<8} {'n_crit':<8} {'expected':<10} {'actual':<10} {'status':<10}")
        print("   " + "-" * 60)
        for t in range(min(10, T)):
            n_cand = int(scalars[t, IDX_N_CANDIDATES])
            n_crit = int(scalars[t, IDX_N_CRITICS])
            expected = n_cand * n_crit
            actual = len(scores[t])
            non_zero = np.count_nonzero(scores[t])
            
            status = "OK" if actual >= expected and n_cand > 0 and n_crit > 0 else "SKIP"
            print(f"   {t:<4} {n_cand:<8} {n_crit:<8} {expected:<10} {actual:<10} {status:<10} (non-zero: {non_zero})")


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Usage: python3 diagnose_h5.py <path_to_h5_file>")
        print("Example: python3 diagnose_h5.py training/data/run_20260331_121635.h5")
        sys.exit(1)
    
    diagnose(sys.argv[1])
