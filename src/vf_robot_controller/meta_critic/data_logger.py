#!/usr/bin/env python3
"""
data_logger.py — subscribes to critic data and writes HDF5 training files.

Active during Mode 2 (COLLECT). Run alongside the C++ controller.

Subscribes:
  /vf_controller/features    (Float32MultiArray)  410-dim feature vector
  /vf_controller/critic_data (Float32MultiArray)  critic score matrix + metadata

HDF5 file layout (one file per run):
  features   (N, 410)   float32
  scalars    (N, 11)    float32  — [timestamp, rx, ry, rtheta, vx, vz,
                                     goal_dist, goal_heading, best_idx,
                                     n_candidates, n_critics]
  scores     (N, M)     float32  — flat critic score matrix per timestep
                                   M = n_candidates * n_critics

Files saved to: training/data/run_YYYYMMDD_HHMMSS.h5
Buffer flushed to disk every 10 seconds.
"""

import os
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from datetime import datetime

try:
    import h5py
    H5PY_AVAILABLE = True
except ImportError:
    H5PY_AVAILABLE = False


FEATURE_DIM   = 410
FLUSH_INTERVAL = 10.0   # seconds between disk flushes
HEADER_SIZE   = 11      # scalars at start of critic_data message


class DataLogger(Node):

    def __init__(self):
        super().__init__('data_logger')

        if not H5PY_AVAILABLE:
            self.get_logger().error('h5py not installed — DataLogger cannot write files')
            self.get_logger().error('Install with: pip3 install h5py')

        # ── Output file ───────────────────────────────────────────────────────
        self.declare_parameter('data_dir', 'training/data')
        data_dir = self.get_parameter('data_dir').value
        os.makedirs(data_dir, exist_ok=True)
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.filepath = os.path.join(data_dir, f'run_{timestamp}.h5')
        self.get_logger().info(f'DataLogger: writing to {self.filepath}')

        # ── State ─────────────────────────────────────────────────────────────
        self._latest_features  = None   # np.ndarray (410,)
        self._latest_critic    = None   # np.ndarray (HEADER_SIZE + N*K,)

        # In-memory buffers — flushed every FLUSH_INTERVAL seconds
        self._buf_features : list[np.ndarray] = []
        self._buf_scalars  : list[np.ndarray] = []
        self._buf_scores   : list[np.ndarray] = []

        self._total_written = 0

        # ── Subscriptions ─────────────────────────────────────────────────────
        self.create_subscription(
            Float32MultiArray,
            '/vf_controller/features',
            self._on_features,
            10)

        self.create_subscription(
            Float32MultiArray,
            '/vf_controller/critic_data',
            self._on_critic_data,
            10)

        # ── Flush timer ───────────────────────────────────────────────────────
        self.create_timer(FLUSH_INTERVAL, self._flush)

        self.get_logger().info('DataLogger ready — waiting for data...')

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _on_features(self, msg: Float32MultiArray):
        arr = np.array(msg.data, dtype=np.float32)
        if len(arr) == FEATURE_DIM:
            self._latest_features = arr
        else:
            self.get_logger().warn(
                f'Feature dim mismatch: {len(arr)} != {FEATURE_DIM}',
                throttle_duration_sec=10.0)

    def _on_critic_data(self, msg: Float32MultiArray):
        arr = np.array(msg.data, dtype=np.float32)
        if len(arr) < HEADER_SIZE:
            return
        self._latest_critic = arr
        self._try_record()

    # ── Record one timestep ───────────────────────────────────────────────────

    def _try_record(self):
        """Pair latest features with latest critic data and buffer the row."""
        if self._latest_features is None or self._latest_critic is None:
            return

        features = self._latest_features.copy()
        critic   = self._latest_critic.copy()

        # Parse header
        scalars = critic[:HEADER_SIZE]          # (11,)
        scores  = critic[HEADER_SIZE:]          # (N*K,) flat

        self._buf_features.append(features)
        self._buf_scalars.append(scalars)
        self._buf_scores.append(scores)

        # Reset so we don't double-record
        self._latest_critic = None

    # ── HDF5 flush ────────────────────────────────────────────────────────────

    def _flush(self):
        if not self._buf_features:
            return
        if not H5PY_AVAILABLE:
            self.get_logger().warn('h5py unavailable — skipping flush')
            self._buf_features.clear()
            self._buf_scalars.clear()
            self._buf_scores.clear()
            return

        n = len(self._buf_features)
        feat_arr   = np.stack(self._buf_features)   # (n, 410)
        scalar_arr = np.stack(self._buf_scalars)    # (n, 11)
        scores_arr = self._pad_scores(self._buf_scores)  # (n, M)

        try:
            with h5py.File(self.filepath, 'a') as f:
                self._append_dataset(f, 'features', feat_arr)
                self._append_dataset(f, 'scalars',  scalar_arr)
                self._append_dataset(f, 'scores',   scores_arr)

            self._total_written += n
            self.get_logger().info(
                f'Flushed {n} samples to {self.filepath} '
                f'(total: {self._total_written})')
        except Exception as e:
            self.get_logger().error(f'HDF5 write error: {e}')

        self._buf_features.clear()
        self._buf_scalars.clear()
        self._buf_scores.clear()

    def _append_dataset(self, f: 'h5py.File', name: str, data: np.ndarray):
        """Append data to a resizable HDF5 dataset, creating it if needed."""
        if name not in f:
            maxshape = (None,) + data.shape[1:]
            f.create_dataset(name, data=data, maxshape=maxshape,
                             chunks=True, compression='gzip', compression_opts=1)
        else:
            existing = f[name].shape[0]
            f[name].resize(existing + data.shape[0], axis=0)
            f[name][existing:] = data

    def _pad_scores(self, scores_list: list) -> np.ndarray:
        """Pad score arrays to same length (handles variable n_candidates)."""
        max_len = max(len(s) for s in scores_list)
        padded = np.zeros((len(scores_list), max_len), dtype=np.float32)
        for i, s in enumerate(scores_list):
            padded[i, :len(s)] = s
        return padded

    def _flush_on_shutdown(self):
        self.get_logger().info('DataLogger shutting down — final flush...')
        self._flush()


def main(args=None):
    rclpy.init(args=args)
    node = DataLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._flush_on_shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
