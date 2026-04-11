#!/usr/bin/env python3
"""
inference_node.py — runs the trained meta-critic network at 20 Hz.

Subscribes:
  /vf_controller/features     (std_msgs/Float32MultiArray)  410-dim vector

Publishes:
  /vf_controller/meta_weights (std_msgs/Float32MultiArray)  10 softmax weights

Parameters:
  model_path   (string)  path to meta_critic.pt TorchScript file
  num_critics  (int)     number of critics — must match network output dim (10)
  feature_dim  (int)     expected input dimension (410)

Behaviour:
  - If model_path exists: runs forward pass, publishes softmax weights
  - If model_path missing or load fails: publishes uniform weights (1/K each)
    and logs a warning every 5 seconds — controller falls back gracefully
  - Feature vector size mismatch: logs error, publishes uniform weights
"""

import os
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from ament_index_python.packages import get_package_share_directory

try:
    import torch
    TORCH_AVAILABLE = True
except ImportError:
    TORCH_AVAILABLE = False


NUM_CRITICS  = 10
FEATURE_DIM  = 410


class InferenceNode(Node):

    def __init__(self):
        super().__init__('meta_critic_inference')

        # ── Parameters ───────────────────────────────────────────────────────
        self.declare_parameter('model_path',  'meta_critic/models/meta_critic.pt')
        self.declare_parameter('num_critics', NUM_CRITICS)
        self.declare_parameter('feature_dim', FEATURE_DIM)

        model_path       = self.get_parameter('model_path').value
        self.num_critics = self.get_parameter('num_critics').value
        self.feature_dim = self.get_parameter('feature_dim').value

        # ── Load normalization statistics ─────────────────────────────────────
        self.feature_mean = None
        self.feature_std = None
        self._load_normalization_stats()

        # ── Load model ───────────────────────────────────────────────────────
        self.model = None
        self._load_model(model_path)

        # ── Uniform fallback (used when model absent or stale) ────────────────
        self._uniform = np.full(
            self.num_critics,
            1.0 / self.num_critics,
            dtype=np.float32)

        # ── ROS interface ─────────────────────────────────────────────────────
        self.sub = self.create_subscription(
            Float32MultiArray,
            '/vf_controller/features',
            self._on_features,
            10)

        self.pub = self.create_publisher(
            Float32MultiArray,
            '/vf_controller/meta_weights',
            10)

        # Periodic status log
        self._status_timer = self.create_timer(5.0, self._log_status)
        self._inference_count = 0

        self.get_logger().info(
            f'InferenceNode ready — model: {"loaded" if self.model else "ABSENT (uniform fallback)"}'
            f', num_critics={self.num_critics}')

    # ── Model loading ─────────────────────────────────────────────────────────

    def _load_model(self, model_path: str):
        if not TORCH_AVAILABLE:
            self.get_logger().warn('torch not available — running uniform fallback')
            return
        try:
            self.model = torch.jit.load(model_path, map_location='cpu')
            self.model.eval()
            self.get_logger().info(f'Meta-critic model loaded from: {model_path}')
        except Exception as e:
            self.model = None
            self.get_logger().warn(
                f'Could not load model from {model_path}: {e}\n'
                f'Running uniform fallback weights until model is available.')

    # ── Normalization statistics loading ──────────────────────────────────────

    def _load_normalization_stats(self):
        """Load feature normalization statistics from training."""
        if not TORCH_AVAILABLE:
            return

        # Try to load from package share directory
        try:
            pkg_share = get_package_share_directory('vf_robot_controller')
            stats_path = os.path.join(pkg_share, 'training', 'meta_critic_stats.npz')

            if os.path.exists(stats_path):
                stats = np.load(stats_path)
                self.feature_mean = stats['mean']
                self.feature_std = stats['std']
                self.get_logger().info(
                    f'Loaded normalization stats from {stats_path}')
            else:
                self.get_logger().warn(
                    f'Normalization stats not found at {stats_path} — '
                    f'running without feature normalization')
        except Exception as e:
            self.get_logger().warn(
                f'Could not load normalization stats: {e} — '
                f'running without feature normalization')

    # ── Subscription callback ─────────────────────────────────────────────────

    def _on_features(self, msg: Float32MultiArray):
        features = np.array(msg.data, dtype=np.float32)

        # Sanity check
        if len(features) != self.feature_dim:
            self.get_logger().error(
                f'Feature dim mismatch: got {len(features)}, expected {self.feature_dim}'
                f' — publishing uniform weights',
                throttle_duration_sec=5.0)
            self._publish(self._uniform)
            return

        weights = self._infer(features)
        self._publish(weights)
        self._inference_count += 1

    # ── Inference ─────────────────────────────────────────────────────────────

    def _infer(self, features: np.ndarray) -> np.ndarray:
        """Run forward pass. Returns (num_critics,) float32 array summing to 1."""
        if self.model is None:
            return self._uniform

        try:
            # Apply z-score normalization if statistics are available
            if self.feature_mean is not None and self.feature_std is not None:
                features = (features - self.feature_mean) / self.feature_std

            with torch.no_grad():
                x = torch.from_numpy(features).unsqueeze(0)   # (1, 410)
                w = self.model(x).squeeze(0).numpy()           # (10,)

            # Defensive: ensure valid probability vector
            w = np.clip(w, 0.0, None)
            total = w.sum()
            if total > 1e-6:
                w /= total
            else:
                w = self._uniform.copy()

            return w.astype(np.float32)

        except Exception as e:
            self.get_logger().error(
                f'Inference error: {e} — falling back to uniform',
                throttle_duration_sec=5.0)
            return self._uniform

    # ── Publisher ─────────────────────────────────────────────────────────────

    def _publish(self, weights: np.ndarray):
        msg = Float32MultiArray()
        msg.data = weights.tolist()
        self.pub.publish(msg)

    # ── Status log ────────────────────────────────────────────────────────────

    def _log_status(self):
        mode = 'INFERENCE' if self.model is not None else 'UNIFORM FALLBACK'
        norm_status = 'NORMALIZED' if self.feature_mean is not None else 'RAW FEATURES'
        self.get_logger().info(
            f'InferenceNode [{mode}, {norm_status}] — {self._inference_count} inferences since last log')
        self._inference_count = 0


def main(args=None):
    rclpy.init(args=args)
    node = InferenceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
