#!/usr/bin/env python3
"""
imitation_data_logger.py — records features + teacher cmd_vel for IMITATION training.

Works alongside ANY Nav2 controller (DWB, MPPI, RPP, etc.).
Does not require vf_robot_controller to be running.

Subscribes:
  /vf_controller/features    (Float32MultiArray)  410-dim feature vector
  /cmd_vel                   (geometry_msgs/Twist) teacher controller output

HDF5 file layout (training/imitation/run_*.h5):
  features   (T, 410)   float32
  cmd_vel    (T, 2)     float32  — [linear.x, angular.z]

Files saved to: training/imitation/run_YYYYMMDD_HHMMSS.h5
"""

import os
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist
from datetime import datetime

try:
    import h5py
    H5PY_AVAILABLE = True
except ImportError:
    H5PY_AVAILABLE = False

FEATURE_DIM    = 410
FLUSH_INTERVAL = 10.0


class ImitationDataLogger(Node):

    def __init__(self):
        super().__init__('imitation_data_logger')

        if not H5PY_AVAILABLE:
            self.get_logger().error('h5py not installed: pip3 install h5py')

        self.declare_parameter('data_dir', 'training/imitation')
        data_dir = self.get_parameter('data_dir').value
        os.makedirs(data_dir, exist_ok=True)

        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.filepath = os.path.join(data_dir, f'run_{timestamp}.h5')
        self.get_logger().info(f'ImitationDataLogger: writing to {self.filepath}')

        self._latest_features = None   # np.ndarray (410,)
        self._latest_cmd_vel  = None   # np.ndarray (2,)  [vx, vz]

        self._buf_features: list[np.ndarray] = []
        self._buf_cmd_vel:  list[np.ndarray] = []
        self._total_written = 0

        self.create_subscription(
            Float32MultiArray,
            '/vf_controller/features',
            self._on_features,
            10)

        self.create_subscription(
            Twist,
            '/cmd_vel',
            self._on_cmd_vel,
            10)

        self.create_timer(FLUSH_INTERVAL, self._flush)
        self.get_logger().info('ImitationDataLogger ready — waiting for data...')

    def _on_features(self, msg: Float32MultiArray):
        arr = np.array(msg.data, dtype=np.float32)
        if len(arr) == FEATURE_DIM:
            self._latest_features = arr
        else:
            self.get_logger().warn(
                f'Feature dim mismatch: {len(arr)} != {FEATURE_DIM}',
                throttle_duration_sec=10.0)

    def _on_cmd_vel(self, msg: Twist):
        self._latest_cmd_vel = np.array(
            [msg.linear.x, msg.angular.z], dtype=np.float32)
        self._try_record()

    def _try_record(self):
        if self._latest_features is None or self._latest_cmd_vel is None:
            return
        self._buf_features.append(self._latest_features.copy())
        self._buf_cmd_vel.append(self._latest_cmd_vel.copy())
        # reset cmd_vel so we don't double-record on same teacher command
        self._latest_cmd_vel = None

    def _flush(self):
        if not self._buf_features:
            return
        if not H5PY_AVAILABLE:
            self._buf_features.clear()
            self._buf_cmd_vel.clear()
            return

        n = len(self._buf_features)
        feat_arr = np.stack(self._buf_features)   # (n, 410)
        vel_arr  = np.stack(self._buf_cmd_vel)    # (n, 2)

        try:
            with h5py.File(self.filepath, 'a') as f:
                self._append_dataset(f, 'features', feat_arr)
                self._append_dataset(f, 'cmd_vel',  vel_arr)
            self._total_written += n
            self.get_logger().info(
                f'Flushed {n} samples (total: {self._total_written})')
        except Exception as e:
            self.get_logger().error(f'HDF5 write error: {e}')

        self._buf_features.clear()
        self._buf_cmd_vel.clear()

    def _append_dataset(self, f, name: str, data: np.ndarray):
        if name not in f:
            maxshape = (None,) + data.shape[1:]
            f.create_dataset(name, data=data, maxshape=maxshape,
                             chunks=True, compression='gzip', compression_opts=1)
        else:
            existing = f[name].shape[0]
            f[name].resize(existing + data.shape[0], axis=0)
            f[name][existing:] = data

    def _flush_on_shutdown(self):
        self.get_logger().info('ImitationDataLogger shutting down — final flush...')
        self._flush()


def main(args=None):
    rclpy.init(args=args)
    node = ImitationDataLogger()
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
