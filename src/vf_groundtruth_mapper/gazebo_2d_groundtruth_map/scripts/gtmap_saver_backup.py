#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty
from nav_msgs.msg import OccupancyGrid
import numpy as np
from PIL import Image
import yaml
import os
import time


class GTMapSaver(Node):
    def __init__(self):
        super().__init__("gtmap_saver")

        # Load parameters from YAML
        config_file = os.path.join(
            os.path.dirname(os.path.realpath(__file__)),
            "../../config/gtmapper_params.yaml",
        )
        with open(config_file, "r") as f:
            self.config = yaml.safe_load(f)

        self.output_dir = self.config.get("output_dir", "2d_gtmap")
        os.makedirs(self.output_dir, exist_ok=True)
        self.map_name = self.config.get("map_name", "gt_map")
        self.map_resolution = self.config.get("map_resolution", 0.05)

        # Service to trigger Gazebo map plugin
        self.srv_name = "gazebo_2d_groundtruth_map/generate_map"
        self.cli = self.create_client(Empty, self.srv_name)
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f"Waiting for service {self.srv_name}...")

        # Subscription to map topic
        self.map_sub = self.create_subscription(
            OccupancyGrid, "map", self.map_callback, 10
        )
        self.map_received = False

        self.get_logger().info(f"Calling service {self.srv_name} to generate map...")
        self.call_service()

        # Wait until map is actually received
        while not self.map_received:
            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(0.01)

    def call_service(self):
        req = Empty.Request()
        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info("Map generation triggered successfully!")
        else:
            self.get_logger().error("Failed to call map generation service.")

    def map_callback(self, msg: OccupancyGrid):
        if self.map_received:
            return  # only save once
        self.map_received = True
        self.get_logger().info("OccupancyGrid received, saving PGM and YAML...")

        # Save map as PGM
        self.save_map_as_pgm(msg)
        # Save map YAML
        self.save_yaml(msg)

    def save_map_as_pgm(self, msg: OccupancyGrid):
        data = np.array(msg.data, dtype=np.int8).reshape(
            msg.info.height, msg.info.width
        )

        # Convert OccupancyGrid to PGM values
        img_data = np.zeros_like(data, dtype=np.uint8)
        img_data[data == 100] = 0  # occupied → black
        img_data[data == 0] = 254  # free → white
        img_data[data == -1] = 205  # unknown → gray

        img = Image.fromarray(img_data)
        pgm_file = os.path.join(self.output_dir, f"{self.map_name}.pgm")
        img.save(pgm_file)
        self.get_logger().info(f"Map PGM saved to: {pgm_file}")

    def save_yaml(self, msg: OccupancyGrid):
        yaml_file = os.path.join(self.output_dir, f"{self.map_name}.yaml")

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

        with open(yaml_file, "w") as f:
            yaml.dump(map_yaml, f, default_flow_style=False)

        self.get_logger().info(f"Map YAML saved to: {yaml_file}")


def main(args=None):
    rclpy.init(args=args)
    node = GTMapSaver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
