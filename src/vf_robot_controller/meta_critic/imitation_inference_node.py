#!/usr/bin/env python3
"""
imitation_inference_node.py — runs imitation_best.pt and publishes /cmd_vel.

Requires vf_robot_controller running in PASSIVE mode (satisfies Nav2
action server but publishes zero velocity — this node owns /cmd_vel).

Subscribes:  /vf_controller/features  (Float32MultiArray)
Publishes:   /cmd_vel                 (geometry_msgs/Twist)
"""

import os
import numpy as np
import torch
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist

FEATURE_DIM = 410


class ImitationInferenceNode(Node):

    def __init__(self):
        super().__init__("imitation_inference_node")

        self.declare_parameter("model_path", "")
        self.declare_parameter("feature_dim", FEATURE_DIM)

        model_path = self.get_parameter("model_path").value
        feature_dim = self.get_parameter("feature_dim").value

        self._model = None
        self._device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

        if model_path and os.path.exists(model_path):
            try:
                self._model = torch.jit.load(model_path, map_location=self._device)
                self._model.eval()
                self.get_logger().info(f"Imitation model loaded from {model_path}")
            except Exception as e:
                self.get_logger().error(f"Failed to load model: {e}")
        else:
            self.get_logger().warn(
                f"Model not found at {model_path} — publishing zero velocity"
            )

        self._cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        self.create_subscription(
            Float32MultiArray, "/vf_controller/features", self._on_features, 10
        )

        self.get_logger().info("ImitationInferenceNode ready")

    def _on_features(self, msg: Float32MultiArray):
        arr = np.array(msg.data, dtype=np.float32)
        if len(arr) != FEATURE_DIM:
            return

        twist = Twist()

        if self._model is not None:
            try:
                x = torch.from_numpy(arr).unsqueeze(0).to(self._device)
                with torch.no_grad():
                    vel = self._model(x).squeeze(0).cpu().numpy()
                twist.linear.x = float(np.clip(vel[0], -1.0, 1.0))
                twist.angular.z = float(np.clip(vel[1], -2.0, 2.0))
            except Exception as e:
                self.get_logger().error(
                    f"Inference error: {e}", throttle_duration_sec=5.0
                )

        self._cmd_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = ImitationInferenceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
