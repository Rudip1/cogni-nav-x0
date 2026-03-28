"""
rl_finetune.py — Phase 2: PPO fine-tuning of the meta-critic network.

Run AFTER supervised pre-training (train.py) has produced meta_critic.pt.

Usage:
  cd ~/cogni-nav-x0/src/vf_robot_controller
  # Terminal 1: start Gazebo + Nav2 in INFERENCE mode
  ros2 launch vf_robot_controller inference_launch.py
  # Terminal 2: start feature extractor
  ros2 run vf_robot_controller feature_extractor.py
  # Terminal 3: run RL fine-tuning
  python3 training/rl_finetune.py

Requirements:
  pip3 install stable-baselines3 gymnasium

What it does:
  1. Wraps Gazebo+Nav2 in a Gymnasium environment
  2. Loads the pre-trained MetaCriticMLP as PPO policy initialisation
  3. Runs PPO for 500k timesteps with composite reward
  4. Saves checkpoints every 10k steps to training/checkpoints/
  5. Final model saved to meta_critic/models/meta_critic_rl.pt

Reward function:
  r = 1.0 * progress_to_goal
    - 100.0 * collision_flag
    + 0.5  * max(0, clearance - 0.3m)
    - 0.3  * |angular_velocity|
    + 200.0 * goal_reached_flag
"""

import os
import sys
import math
import time
import numpy as np

# ROS 2
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Bool
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty

# Gymnasium
try:
    import gymnasium as gym
    from gymnasium import spaces
    GYM_AVAILABLE = True
except ImportError:
    GYM_AVAILABLE = False
    print('WARNING: gymnasium not installed. Run: pip3 install gymnasium stable-baselines3')

# Stable-Baselines3
try:
    from stable_baselines3 import PPO
    from stable_baselines3.common.callbacks import CheckpointCallback
    SB3_AVAILABLE = True
except ImportError:
    SB3_AVAILABLE = False
    print('WARNING: stable-baselines3 not installed. Run: pip3 install stable-baselines3')

import torch
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


FEATURE_DIM      = 410
NUM_CRITICS      = 10
TOTAL_TIMESTEPS  = 500_000
CHECKPOINT_FREQ  = 10_000
PRETRAINED_PATH  = 'meta_critic/models/meta_critic.pt'
CHECKPOINT_DIR   = 'training/checkpoints'
OUTPUT_PATH      = 'meta_critic/models/meta_critic_rl'
TB_LOG_DIR       = 'training/tb_logs'


# ── Gymnasium Environment ─────────────────────────────────────────────────────

class MetaCriticEnv(gym.Env if GYM_AVAILABLE else object):
    """
    Gymnasium wrapper around the live Gazebo + Nav2 simulation.

    Observation: 410-dim feature vector from feature_extractor.py
    Action:      10-dim raw vector (softmax applied internally before publishing)
    Reward:      composite safety + progress + smoothness signal
    """

    metadata = {'render_modes': []}

    def __init__(self, ros_node: Node):
        if not GYM_AVAILABLE:
            raise ImportError('gymnasium required')

        super().__init__()
        self.node = ros_node

        self.observation_space = spaces.Box(
            low=0.0, high=1.0, shape=(FEATURE_DIM,), dtype=np.float32)
        self.action_space = spaces.Box(
            low=-3.0, high=3.0, shape=(NUM_CRITICS,), dtype=np.float32)

        # Publishers
        self.weight_pub = self.node.create_publisher(
            Float32MultiArray, '/vf_controller/meta_weights', 10)

        # Subscribers
        self._obs_buffer     = None
        self._odom           = None
        self._collision_flag = False
        self._goal_reached   = False
        self._prev_goal_dist = None

        self.node.create_subscription(
            Float32MultiArray, '/vf_controller/features',
            lambda msg: setattr(self, '_obs_buffer',
                                np.array(msg.data, dtype=np.float32)), 10)

        self.node.create_subscription(
            Odometry, '/odom', self._odom_cb, 10)

        # Reset service (calls Nav2 navigation reset)
        self._reset_client = self.node.create_client(Empty, '/reinitialize_global_localization')

        self.node.get_logger().info('MetaCriticEnv ready')

    # ── Gymnasium interface ───────────────────────────────────────────────────

    def reset(self, seed=None, options=None):
        self._collision_flag = False
        self._goal_reached   = False
        self._prev_goal_dist = None
        self._obs_buffer     = None

        # Try to reset simulation (non-blocking if service unavailable)
        self._try_reset_sim()

        obs = self._wait_for_obs()
        return obs, {}

    def step(self, action: np.ndarray):
        # Convert raw action to valid weight simplex via softmax
        exp_a = np.exp(action - action.max())
        weights = exp_a / exp_a.sum()
        weights = weights.astype(np.float32)

        # Publish weights to C++ controller
        msg = Float32MultiArray()
        msg.data = weights.tolist()
        self.weight_pub.publish(msg)

        # Spin once to process incoming callbacks
        rclpy.spin_once(self.node, timeout_sec=0.05)

        obs    = self._wait_for_obs()
        reward = self._compute_reward()
        done   = self._collision_flag or self._goal_reached
        info   = {
            'collision':    self._collision_flag,
            'goal_reached': self._goal_reached,
        }
        return obs, reward, done, False, info

    def close(self):
        pass

    # ── Reward ────────────────────────────────────────────────────────────────

    def _compute_reward(self) -> float:
        r_progress   = self._get_progress()
        r_clearance  = self._get_clearance()
        r_smooth     = self._get_smoothness()
        r_collision  = -100.0 if self._collision_flag else 0.0
        r_goal       = +200.0 if self._goal_reached   else 0.0

        return (
            1.0 * r_progress
            + r_collision
            + 0.5 * max(0.0, r_clearance - 0.3)
            - 0.3 * r_smooth
            + r_goal
        )

    def _get_progress(self) -> float:
        """Progress toward goal = reduction in goal distance."""
        if self._odom is None or self._obs_buffer is None:
            return 0.0
        # goal_distance is at feature index 407, normalised [0,1] / 20m
        goal_dist = float(self._obs_buffer[407]) * 20.0
        if self._prev_goal_dist is None:
            self._prev_goal_dist = goal_dist
            return 0.0
        progress = self._prev_goal_dist - goal_dist
        self._prev_goal_dist = goal_dist
        if goal_dist < 0.3:
            self._goal_reached = True
        return float(progress)

    def _get_clearance(self) -> float:
        """Closest obstacle distance from feature vector (index 404)."""
        if self._obs_buffer is None:
            return 1.0
        return float(self._obs_buffer[404]) * 5.0  # de-normalise

    def _get_smoothness(self) -> float:
        """Angular velocity magnitude from feature vector (index 406)."""
        if self._obs_buffer is None:
            return 0.0
        return abs(float(self._obs_buffer[406]))

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _odom_cb(self, msg: Odometry):
        self._odom = msg

    def _wait_for_obs(self, timeout: float = 1.0) -> np.ndarray:
        t0 = time.time()
        while self._obs_buffer is None:
            rclpy.spin_once(self.node, timeout_sec=0.01)
            if time.time() - t0 > timeout:
                return np.zeros(FEATURE_DIM, dtype=np.float32)
        obs = self._obs_buffer.copy()
        self._obs_buffer = None
        return obs

    def _try_reset_sim(self):
        """Non-blocking attempt to reset Nav2 localisation."""
        if self._reset_client.service_is_ready():
            self._reset_client.call_async(Empty.Request())
        time.sleep(0.5)  # give sim time to reset


# ── PPO Training ──────────────────────────────────────────────────────────────

def train_rl(
    pretrained_path: str = PRETRAINED_PATH,
    total_timesteps: int = TOTAL_TIMESTEPS,
):
    if not GYM_AVAILABLE or not SB3_AVAILABLE:
        print('ERROR: gymnasium and stable-baselines3 required.')
        print('Install: pip3 install gymnasium stable-baselines3')
        return

    os.makedirs(CHECKPOINT_DIR, exist_ok=True)
    os.makedirs(TB_LOG_DIR,     exist_ok=True)

    rclpy.init()
    ros_node = rclpy.create_node('rl_trainer')

    env = MetaCriticEnv(ros_node)

    # PPO with MlpPolicy (matches MetaCriticMLP architecture)
    model = PPO(
        policy          = 'MlpPolicy',
        env             = env,
        learning_rate   = 3e-4,
        n_steps         = 2048,
        batch_size      = 64,
        n_epochs        = 10,
        gamma           = 0.99,
        gae_lambda      = 0.95,
        clip_range      = 0.2,
        ent_coef        = 0.01,   # encourages exploration
        verbose         = 1,
        tensorboard_log = TB_LOG_DIR,
        policy_kwargs   = dict(net_arch=[256, 128, 64]),
    )

    # Load pre-trained weights into PPO policy network if available
    if os.path.exists(pretrained_path):
        try:
            _load_pretrained_into_ppo(model, pretrained_path)
            print(f'Loaded pre-trained weights from {pretrained_path}')
        except Exception as e:
            print(f'Could not load pre-trained weights: {e} — starting from scratch')
    else:
        print(f'No pre-trained model at {pretrained_path} — starting from scratch')
        print('Run training/train.py first for better convergence.')

    checkpoint_cb = CheckpointCallback(
        save_freq   = CHECKPOINT_FREQ,
        save_path   = CHECKPOINT_DIR,
        name_prefix = 'meta_critic_rl',
    )

    print(f'\nStarting PPO fine-tuning for {total_timesteps:,} timesteps...')
    model.learn(
        total_timesteps = total_timesteps,
        callback        = checkpoint_cb,
        progress_bar    = True,
    )

    model.save(OUTPUT_PATH)
    print(f'\nRL fine-tuning complete. Model saved to {OUTPUT_PATH}')

    rclpy.shutdown()


def _load_pretrained_into_ppo(ppo_model, pt_path: str):
    """
    Copy weights from TorchScript meta_critic.pt into PPO's MlpPolicy network.
    Best-effort: layer shapes must match.
    """
    scripted = torch.jit.load(pt_path, map_location='cpu')
    pt_state = {k: v for k, v in scripted.named_parameters()}

    ppo_state = ppo_model.policy.mlp_extractor.policy_net.state_dict()
    matched = 0
    for key in ppo_state:
        if key in pt_state and ppo_state[key].shape == pt_state[key].shape:
            ppo_state[key] = pt_state[key].detach().clone()
            matched += 1
    ppo_model.policy.mlp_extractor.policy_net.load_state_dict(ppo_state)
    print(f'  Matched {matched}/{len(ppo_state)} layers from pre-trained model')


if __name__ == '__main__':
    train_rl()
