#pragma once

#include <vector>
#include <Eigen/Dense>
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "vf_robot_controller/parameter_handler.hpp"

namespace vf_robot_controller::gcf
{

/**
 * @brief VolumetricClearance — 3D height-aware obstacle query.
 *
 * This is what sees chair legs (0.05–0.35m height), bed frames (0.15–0.25m),
 * IV stand bases (0.0–0.1m), and wheelchair footrests (0.0–0.12m).
 * A standard 2D LiDAR at 0.3m scan height misses all of these.
 *
 * Input: sensor_msgs/PointCloud2 (from 3D LiDAR, RGBD camera, or structured light)
 *
 * For each (x,y) in the local window, it answers:
 *   "At this position, what is the minimum vertical clearance that the robot
 *    body can pass through, given robot_height_min to robot_height_max?"
 *
 * If no 3D data available: falls back to 2D with conservative inflation.
 *
 * The result is a 2D field of volumetric clearance values used as the
 * third component of the GCF.
 */
class VolumetricClearance
{
public:
  explicit VolumetricClearance(const Parameters & params);

  /**
   * @brief Build internal voxel structure from new pointcloud.
   * Call whenever a new pointcloud arrives (typically 5-15 Hz).
   */
  void updateFromPointcloud(
    const sensor_msgs::msg::PointCloud2::SharedPtr & cloud,
    double robot_x, double robot_y);

  /**
   * @brief Compute volumetric clearance field for local window.
   * Returns flat grid (row-major) of clearance values in [0,1].
   * 0 = blocked by obstacle in robot height range
   * 1 = fully clear
   */
  std::vector<double> computeField(
    double origin_x, double origin_y,
    int width, int height, double resolution) const;

  /**
   * @brief Query clearance at single world point.
   * Returns minimum horizontal distance to any obstacle in the
   * height range [robot_height_min, robot_height_max].
   */
  double queryAt(double x, double y) const;

  /**
   * @brief Check if robot can traverse a cell.
   * Checks all height slices from robot_height_min to robot_height_max.
   */
  bool isTraversable(double x, double y, double robot_half_width) const;

  bool hasData() const { return has_data_; }

private:
  struct VoxelPoint { float x, y, z; };

  const Parameters & params_;
  bool has_data_{false};

  // Sparse voxel storage — only points in robot height range stored
  std::vector<VoxelPoint> relevant_points_;
  double data_origin_x_{0.0}, data_origin_y_{0.0};

  // Grid-based spatial index for fast range queries
  std::vector<std::vector<size_t>> spatial_index_;
  int index_width_{0}, index_height_{0};
  double index_resolution_{0.1};
  double index_origin_x_{0.0}, index_origin_y_{0.0};

  void buildSpatialIndex();
  std::vector<size_t> queryCell(int cx, int cy) const;
};

}  // namespace vf_robot_controller::gcf
