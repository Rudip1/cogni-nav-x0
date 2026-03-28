#!/usr/bin/env python3
"""
feature_extractor.py — builds the 410-dim input vector for the meta-critic network.

Subscribes:
  /local_costmap/costmap   (nav_msgs/OccupancyGrid)
  /odom                    (nav_msgs/Odometry)
  /plan                    (nav_msgs/Path)  — global plan for path deviation

Publishes:
  /vf_controller/features  (std_msgs/Float32MultiArray)  at 20 Hz

Feature vector layout (410 floats):
  [0:400]   costmap patch  20x20 grid centred on robot, normalised [0,1]
  [400]     corridor_width  clipped [0,5] / 5
  [401]     gcf_local_mean  placeholder (0.5) — computed in C++ GCF
  [402]     gcf_local_max   placeholder (0.5)
  [403]     num_dynamic_obs clipped [0,10] / 10
  [404]     closest_obs_dist clipped [0,5] / 5
  [405]     robot_linear_vel  / 1.0 (m/s)
  [406]     robot_angular_vel / 1.0 (rad/s)
  [407]     goal_distance   clipped [0,20] / 20
  [408]     goal_heading_error / pi
  [409]     path_deviation  clipped [0,2] / 2
"""

import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from nav_msgs.msg import OccupancyGrid, Odometry, Path
from std_msgs.msg import Float32MultiArray


PATCH_SIZE   = 20     # 20x20 costmap grid centred on robot
FEATURE_DIM  = 410
TIMER_HZ     = 20.0   # publish rate


class FeatureExtractor(Node):

    def __init__(self):
        super().__init__('feature_extractor')

        # QoS for costmap — use best-effort to avoid blocking on slow maps
        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE)

        self.costmap_sub = self.create_subscription(
            OccupancyGrid, '/local_costmap/costmap',
            self._costmap_cb, map_qos)

        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self._odom_cb, 10)

        self.plan_sub = self.create_subscription(
            Path, '/plan', self._plan_cb, 10)

        self.pub = self.create_publisher(
            Float32MultiArray, '/vf_controller/features', 10)

        self.timer = self.create_timer(1.0 / TIMER_HZ, self._publish)

        self._costmap  = None
        self._odom     = None
        self._plan     = None

        self.get_logger().info('FeatureExtractor started — publishing to /vf_controller/features')

    # ── Callbacks ────────────────────────────────────────────────────────────

    def _costmap_cb(self, msg: OccupancyGrid):
        self._costmap = msg

    def _odom_cb(self, msg: Odometry):
        self._odom = msg

    def _plan_cb(self, msg: Path):
        self._plan = msg

    # ── Main publish loop ─────────────────────────────────────────────────────

    def _publish(self):
        if self._costmap is None or self._odom is None:
            return  # not ready yet

        vec = self._build_feature_vector()
        msg = Float32MultiArray()
        msg.data = vec.tolist()
        self.pub.publish(msg)

    # ── Feature construction ──────────────────────────────────────────────────

    def _build_feature_vector(self) -> np.ndarray:
        odom  = self._odom
        cmap  = self._costmap

        robot_x = odom.pose.pose.position.x
        robot_y = odom.pose.pose.position.y
        q = odom.pose.pose.orientation
        robot_yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z))

        robot_vx  = odom.twist.twist.linear.x
        robot_vz  = odom.twist.twist.angular.z

        # 1. Costmap patch (400 values)
        patch = self._extract_patch(cmap, robot_x, robot_y)  # (400,)

        # 2. Corridor width from costmap ray cast (1 value)
        corridor_w = self._estimate_corridor_width(cmap, robot_x, robot_y, robot_yaw)
        corridor_w_norm = np.clip(corridor_w, 0.0, 5.0) / 5.0

        # 3. GCF stats — placeholder (C++ GCF is the authoritative source;
        #    these are reasonable priors until a separate GCF topic is added)
        gcf_mean = 0.5
        gcf_max  = 0.5

        # 4. Dynamic obstacles and closest obstacle from costmap
        num_dyn, closest_dist = self._scan_obstacles(cmap, robot_x, robot_y)
        num_dyn_norm    = np.clip(num_dyn,    0.0, 10.0) / 10.0
        closest_norm    = np.clip(closest_dist, 0.0, 5.0) / 5.0

        # 5. Goal distance and heading
        goal_dist, goal_heading = self._goal_info(robot_x, robot_y, robot_yaw)
        goal_dist_norm    = np.clip(goal_dist,    0.0, 20.0) / 20.0
        goal_heading_norm = np.clip(goal_heading, -math.pi, math.pi) / math.pi

        # 6. Path deviation
        path_dev = self._path_deviation(robot_x, robot_y)
        path_dev_norm = np.clip(path_dev, 0.0, 2.0) / 2.0

        # 7. Velocity normalisation
        vx_norm = np.clip(robot_vx / 1.0, -1.0, 1.0)
        vz_norm = np.clip(robot_vz / 1.0, -1.0, 1.0)

        scalars = np.array([
            corridor_w_norm,
            gcf_mean,
            gcf_max,
            num_dyn_norm,
            closest_norm,
            vx_norm,
            vz_norm,
            goal_dist_norm,
            goal_heading_norm,
            path_dev_norm,
        ], dtype=np.float32)

        return np.concatenate([patch, scalars]).astype(np.float32)

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _extract_patch(self, cmap: OccupancyGrid,
                       robot_x: float, robot_y: float) -> np.ndarray:
        """Extract 20x20 costmap patch centred on robot, normalised [0,1]."""
        res   = cmap.info.resolution
        ox    = cmap.info.origin.position.x
        oy    = cmap.info.origin.position.y
        w     = cmap.info.width
        h     = cmap.info.height
        half  = PATCH_SIZE // 2

        cx = int((robot_x - ox) / res)
        cy = int((robot_y - oy) / res)

        data = np.array(cmap.data, dtype=np.float32).reshape(h, w)
        patch = np.zeros((PATCH_SIZE, PATCH_SIZE), dtype=np.float32)

        for i in range(PATCH_SIZE):
            for j in range(PATCH_SIZE):
                py = cy - half + i
                px = cx - half + j
                if 0 <= py < h and 0 <= px < w:
                    val = data[py, px]
                    patch[i, j] = np.clip(val, 0.0, 100.0) / 100.0

        return patch.flatten()

    def _estimate_corridor_width(self, cmap: OccupancyGrid,
                                  robot_x: float, robot_y: float,
                                  robot_yaw: float) -> float:
        """Estimate corridor width by raycasting perpendicular to heading."""
        res  = cmap.info.resolution
        ox   = cmap.info.origin.position.x
        oy   = cmap.info.origin.position.y
        w    = cmap.info.width
        h    = cmap.info.height
        data = np.array(cmap.data, dtype=np.int16).reshape(h, w)

        LETHAL = 90
        MAX_RAY = 3.0  # metres

        def raycast(dx: float, dy: float) -> float:
            dist = 0.0
            step = res
            x, y = robot_x, robot_y
            while dist < MAX_RAY:
                x += dx * step
                y += dy * step
                dist += step
                px = int((x - ox) / res)
                py = int((y - oy) / res)
                if px < 0 or px >= w or py < 0 or py >= h:
                    break
                if data[py, px] >= LETHAL:
                    break
            return dist

        # Perpendicular directions
        perp_yaw = robot_yaw + math.pi / 2.0
        left  = raycast(math.cos(perp_yaw), math.sin(perp_yaw))
        right = raycast(-math.cos(perp_yaw), -math.sin(perp_yaw))
        return left + right

    def _scan_obstacles(self, cmap: OccupancyGrid,
                        robot_x: float, robot_y: float):
        """Count obstacle cells within 5m and find closest distance."""
        res  = cmap.info.resolution
        ox   = cmap.info.origin.position.x
        oy   = cmap.info.origin.position.y
        w    = cmap.info.width
        h    = cmap.info.height

        LETHAL   = 90
        RADIUS   = 5.0   # metres
        cells    = int(RADIUS / res)

        cx = int((robot_x - ox) / res)
        cy = int((robot_y - oy) / res)

        data  = np.array(cmap.data, dtype=np.int16).reshape(h, w)
        count = 0
        closest = RADIUS

        for i in range(-cells, cells + 1):
            for j in range(-cells, cells + 1):
                py, px = cy + i, cx + j
                if 0 <= py < h and 0 <= px < w:
                    dist = math.hypot(i * res, j * res)
                    if dist <= RADIUS and data[py, px] >= LETHAL:
                        count += 1
                        closest = min(closest, dist)

        return float(count), closest

    def _goal_info(self, robot_x: float, robot_y: float,
                   robot_yaw: float):
        """Distance and heading error to final goal pose."""
        if self._plan is None or len(self._plan.poses) == 0:
            return 5.0, 0.0
        goal = self._plan.poses[-1].pose.position
        dx   = goal.x - robot_x
        dy   = goal.y - robot_y
        dist = math.hypot(dx, dy)
        heading_to_goal = math.atan2(dy, dx)
        heading_err = heading_to_goal - robot_yaw
        # Normalise to [-pi, pi]
        heading_err = math.atan2(math.sin(heading_err), math.cos(heading_err))
        return dist, heading_err

    def _path_deviation(self, robot_x: float, robot_y: float) -> float:
        """Minimum cross-track distance from global path."""
        if self._plan is None or len(self._plan.poses) < 2:
            return 0.0
        min_dist = float('inf')
        for pose in self._plan.poses:
            dx = pose.pose.position.x - robot_x
            dy = pose.pose.position.y - robot_y
            min_dist = min(min_dist, math.hypot(dx, dy))
        return min_dist if min_dist < float('inf') else 0.0


def main(args=None):
    rclpy.init(args=args)
    node = FeatureExtractor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
