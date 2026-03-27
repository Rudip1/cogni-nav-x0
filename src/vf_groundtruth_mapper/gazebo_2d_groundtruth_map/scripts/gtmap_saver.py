#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import numpy as np
from PIL import Image
import yaml
import os
from ament_index_python.packages import get_package_share_directory


class GTMapSaver(Node):
    def __init__(self):
        super().__init__("gtmap_saver")

        # Flags
        self.saved = False

        # Load config
        pkg_share = get_package_share_directory("gazebo_2d_groundtruth_map")
        config_file = os.path.join(pkg_share, "config", "gtmapper_params.yaml")
        with open(config_file, "r") as f:
            config = yaml.safe_load(f)

        self.output_dir = config.get("output_dir", "2d_gtmap")
        self.map_name = config.get("map_name", "gt_map")
        os.makedirs(self.output_dir, exist_ok=True)

        # Map subscriber
        self.subscription = self.create_subscription(
            OccupancyGrid, "/map", self.map_callback, 10
        )

        self.get_logger().info("GTMapSaver ready, waiting for /map messages...")

    def map_callback(self, msg):
        """Process the received map message and save files."""
        if self.saved:
            return  # Prevent multiple saves

        self.get_logger().info("Map received. Saving files...")

        # Save PGM and YAML
        self.save_pgm(msg)
        self.save_yaml(msg)

        self.get_logger().info("Map saved successfully. Exiting.")
        self.saved = True

        # Clean shutdown
        self.destroy_node()
        rclpy.shutdown()

    def save_pgm(self, msg):
        """Save the OccupancyGrid as a PGM file (Nav2-compatible)."""
        data = np.array(msg.data, dtype=np.int8).reshape(
            msg.info.height, msg.info.width
        )

        img = np.zeros_like(data, dtype=np.uint8)
        img[data == 100] = 0  # Occupied → black
        img[data == 0] = 254  # Free → white
        img[data == -1] = 205  # Unknown → gray

        # Flip vertically for Nav2 bottom-left origin
        image = Image.fromarray(np.flipud(img))

        pgm_path = os.path.join(self.output_dir, f"{self.map_name}.pgm")
        image.save(pgm_path)
        self.get_logger().info(f"PGM saved: {pgm_path}")

    def save_yaml(self, msg):
        """Save the YAML metadata for the occupancy map."""
        yaml_path = os.path.join(self.output_dir, f"{self.map_name}.yaml")

        map_yaml = {
            "image": f"{self.map_name}.pgm",
            "mode": "trinary",
            "resolution": msg.info.resolution,
            "origin": [
                msg.info.origin.position.x,
                msg.info.origin.position.y,
                msg.info.origin.position.z,
            ],
            "negate": 0,
            "occupied_thresh": 0.65,
            "free_thresh": 0.196,
        }

        with open(yaml_path, "w") as f:
            yaml.dump(map_yaml, f)

        self.get_logger().info(f"YAML saved: {yaml_path}")


def main(args=None):
    rclpy.init(args=args)
    node = GTMapSaver()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
