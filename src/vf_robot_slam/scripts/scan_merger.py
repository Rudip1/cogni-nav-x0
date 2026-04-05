#!/usr/bin/env python3
"""
LaserScan Merger Node — ViroFighter UVC-1
═════════════════════════════════════════

Merges /scan_d435i and /scan_d455 into a single /scan topic.
Replaces ira_laser_tools — zero external dependencies.

How it works:
    1. Subscribes to two LaserScan topics
    2. Transforms each scan's rays into base_footprint frame
    3. For each angular bin, keeps the closest reading from either scan
    4. Publishes combined LaserScan on /scan

Usage:
    ros2 run vf_robot_slam scan_merger.py

Parameters:
    scan_topics:        space-separated input topics (default: "/scan_d435i /scan_d455")
    output_topic:       merged output topic (default: "/scan")
    output_frame:       frame_id for merged scan (default: "base_footprint")
    angle_min:          minimum angle in radians (default: -pi)
    angle_max:          maximum angle in radians (default: +pi)
    angle_increment:    angular resolution (default: 0.00581 ~ 0.33°)
    range_min:          minimum range in meters (default: 0.1)
    range_max:          maximum range in meters (default: 6.0)
"""

import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener
from rclpy.time import Time


class ScanMerger(Node):
    def __init__(self):
        super().__init__("scan_merger")

        # Declare parameters
        self.declare_parameter("scan_topics", "/scan_d435i /scan_d455")
        self.declare_parameter("output_topic", "/scan")
        self.declare_parameter("output_frame", "base_footprint")
        self.declare_parameter("angle_min", -math.pi)
        self.declare_parameter("angle_max", math.pi)
        self.declare_parameter("angle_increment", 0.00581)  # ~0.33°
        self.declare_parameter("range_min", 0.1)
        self.declare_parameter("range_max", 6.0)

        # Read parameters
        topics_str = self.get_parameter("scan_topics").value
        self.scan_topics = topics_str.split()
        self.output_topic = self.get_parameter("output_topic").value
        self.output_frame = self.get_parameter("output_frame").value
        self.angle_min = self.get_parameter("angle_min").value
        self.angle_max = self.get_parameter("angle_max").value
        self.angle_increment = self.get_parameter("angle_increment").value
        self.range_min = self.get_parameter("range_min").value
        self.range_max = self.get_parameter("range_max").value

        # Number of bins in merged scan
        self.num_bins = int((self.angle_max - self.angle_min) / self.angle_increment)

        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # QoS — match typical LaserScan publishers (best effort)
        scan_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        # Subscribe to each input scan
        self.latest_scans = {}
        for topic in self.scan_topics:
            self.latest_scans[topic] = None
            self.create_subscription(
                LaserScan,
                topic,
                lambda msg, t=topic: self._scan_callback(t, msg),
                scan_qos,
            )
            self.get_logger().info(f"Subscribed to {topic}")

        # Publisher
        self.pub = self.create_publisher(LaserScan, self.output_topic, scan_qos)

        # Merge timer — 30 Hz
        self.create_timer(0.033, self._merge_and_publish)

        self.get_logger().info(
            f"ScanMerger: {self.scan_topics} → {self.output_topic} "
            f"in {self.output_frame}"
        )

    def _scan_callback(self, topic: str, msg: LaserScan):
        self.latest_scans[topic] = msg

    def _merge_and_publish(self):
        now = self.get_clock().now()

        # Collect scans, drop any older than 0.5 seconds (stale data)
        scans = []
        for s in self.latest_scans.values():
            if s is None:
                continue
            try:
                age = (now - rclpy.time.Time.from_msg(s.header.stamp)).nanoseconds / 1e9
                if age < 0.5:
                    scans.append(s)
            except Exception:
                # Keep scan if age check fails (e.g., sim time clock not yet available)
                scans.append(s)

        if not scans:
            return

        # Initialize merged ranges to inf (no reading)
        merged = np.full(self.num_bins, float("inf"))

        for scan in scans:
            # Look up transform from scan frame to output frame
            try:
                tf = self.tf_buffer.lookup_transform(
                    self.output_frame,
                    scan.header.frame_id,
                    Time(),  # latest available
                    timeout=rclpy.duration.Duration(seconds=0.05),
                )
            except Exception:
                continue

            # Extract translation and yaw from transform
            t = tf.transform.translation
            q = tf.transform.rotation
            # Yaw from quaternion (2D only)
            siny = 2.0 * (q.w * q.z + q.x * q.y)
            cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            yaw = math.atan2(siny, cosy)

            # Project each ray into output frame
            for i, r in enumerate(scan.ranges):
                if r < scan.range_min or r > scan.range_max:
                    continue
                if math.isinf(r) or math.isnan(r):
                    continue

                # Angle of this ray in the scan's own frame
                ray_angle = scan.angle_min + i * scan.angle_increment

                # Ray endpoint in scan frame
                x_local = r * math.cos(ray_angle)
                y_local = r * math.sin(ray_angle)

                # Transform to output frame
                x_out = t.x + x_local * math.cos(yaw) - y_local * math.sin(yaw)
                y_out = t.y + x_local * math.sin(yaw) + y_local * math.cos(yaw)

                # Range and angle in output frame
                range_out = math.sqrt(x_out * x_out + y_out * y_out)
                angle_out = math.atan2(y_out, x_out)

                if range_out < self.range_min or range_out > self.range_max:
                    continue

                # Map to bin index
                bin_idx = int((angle_out - self.angle_min) / self.angle_increment)
                if 0 <= bin_idx < self.num_bins:
                    # Keep closest reading
                    if range_out < merged[bin_idx]:
                        merged[bin_idx] = range_out

        # Build output message
        msg = LaserScan()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = self.output_frame
        msg.angle_min = self.angle_min
        msg.angle_max = self.angle_max
        msg.angle_increment = self.angle_increment
        msg.time_increment = 0.0
        msg.scan_time = 0.033
        msg.range_min = self.range_min
        msg.range_max = self.range_max

        # inf = no reading (correct for Nav2/AMCL)
        # DO NOT replace inf with 0.0 — that means "obstacle at zero distance"
        msg.ranges = merged.tolist()

        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ScanMerger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
