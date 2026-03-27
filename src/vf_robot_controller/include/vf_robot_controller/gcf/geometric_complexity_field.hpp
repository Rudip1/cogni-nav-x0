#pragma once

#include <memory>
#include <vector>
#include <Eigen/Dense>

#include "nav2_costmap_2d/costmap_2d.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "vf_robot_controller/parameter_handler.hpp"

namespace vf_robot_controller::gcf
{

/**
 * @brief GCF cell — complexity at one grid point in the local window.
 */
struct GCFCell
{
  double complexity{0.0};       // 0 = open, 1 = maximum complexity
  double clearance_2d{1e6};     // distance to nearest 2D obstacle (m)
  double clearance_3d{1e6};     // volumetric clearance (m)
  double clutter_density{0.0};  // number of distinct obstacle clusters nearby
  bool   traversable{true};     // false = hard-blocked
};

/**
 * @brief GeometricComplexityField — the primary novel contribution.
 *
 * Computes a 2D field over the local window where each cell stores a
 * composite complexity score C(x,y) derived from:
 *   1. 2D costmap clearance (what all existing controllers use)
 *   2. Local clutter density (number of distinct obstacle clusters)
 *   3. Volumetric 3D clearance (chair legs, bed frames, under-bed space)
 *
 * High C → allocate more spline knots, reduce speed, increase obstacle weight.
 * Low C  → fewer knots, full speed, follow path efficiently.
 *
 * Updated at gcf_update_rate Hz (lower than control loop to save CPU).
 * Query is O(1) per (x,y) via bilinear interpolation.
 */
class GeometricComplexityField
{
public:
  explicit GeometricComplexityField(const Parameters & params);

  /**
   * @brief Recompute the full GCF for the current robot pose.
   * Call at gcf_update_rate Hz from the controller.
   */
  void update(
    const nav2_costmap_2d::Costmap2D * costmap,
    const sensor_msgs::msg::PointCloud2::SharedPtr & pointcloud,  // may be nullptr
    double robot_x, double robot_y, double robot_yaw);

  /**
   * @brief Query complexity at world coordinates (x, y).
   * Returns GCFCell with all components. O(1) bilinear interpolation.
   */
  GCFCell query(double x, double y) const;

  /**
   * @brief Query mean complexity along a trajectory waypoint sequence.
   * Used by DOF allocator and velocity scheduler.
   */
  double meanComplexityAlongPath(
    const std::vector<Eigen::Vector2d> & waypoints) const;

  /**
   * @brief Per-segment complexity for knot density allocation.
   * Returns one value per segment between consecutive waypoints.
   */
  std::vector<double> complexityPerSegment(
    const std::vector<Eigen::Vector2d> & waypoints) const;

  /**
   * @brief Flat grid for visualization (row-major, resolution = costmap res).
   */
  const std::vector<GCFCell> & getGrid() const { return grid_; }
  double getOriginX() const { return origin_x_; }
  double getOriginY() const { return origin_y_; }
  int getWidth() const { return width_; }
  int getHeight() const { return height_; }
  double getResolution() const { return resolution_; }

private:
  void compute2DClearance(const nav2_costmap_2d::Costmap2D * costmap);
  void computeClutterDensity(const nav2_costmap_2d::Costmap2D * costmap);
  void computeVolumetricClearance(
    const sensor_msgs::msg::PointCloud2::SharedPtr & cloud);
  void combineComponents();

  const Parameters & params_;

  std::vector<GCFCell> grid_;
  int width_{0}, height_{0};
  double origin_x_{0.0}, origin_y_{0.0};
  double resolution_{0.05};

  // Separate component arrays (combined into grid_ by combineComponents)
  std::vector<double> clearance_2d_field_;
  std::vector<double> clutter_field_;
  std::vector<double> volumetric_field_;

  double robot_x_{0.0}, robot_y_{0.0}, robot_yaw_{0.0};
};

}  // namespace vf_robot_controller::gcf
