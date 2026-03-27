#pragma once

#include <vector>
#include <Eigen/Dense>
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "vf_robot_controller/parameter_handler.hpp"

namespace vf_robot_controller::gcf
{

/**
 * @brief ClutterDetector — counts distinct obstacle clusters near each cell.
 *
 * The key insight: a corridor with one wall is NOT cluttered.
 * A room with a bed, chair, IV stand, and wheelchair IS cluttered.
 * Distance-to-nearest-obstacle can't tell these apart.
 * Clutter density counts how many SEPARATE obstacle clusters exist
 * within a radius gcf_clutter_radius of each query point.
 *
 * Algorithm:
 *   1. Extract binary obstacle mask from costmap
 *   2. Run connected-component labeling on obstacle mask
 *   3. For each query point, count how many distinct component IDs
 *      fall within radius gcf_clutter_radius
 *
 * This is the component that detects "4 chair legs = 4 clusters = cluttered"
 * vs "1 wall = 1 cluster = not cluttered".
 */
class ClutterDetector
{
public:
  explicit ClutterDetector(const Parameters & params);

  /**
   * @brief Compute clutter density field for entire costmap window.
   * Returns flat grid (row-major) of clutter values in [0,1].
   * Called once per GCF update cycle.
   */
  std::vector<double> computeField(
    const nav2_costmap_2d::Costmap2D * costmap) const;

  /**
   * @brief Single-point clutter density query.
   * Returns normalized cluster count near (x,y).
   */
  double queryAt(
    double x, double y,
    const nav2_costmap_2d::Costmap2D * costmap) const;

private:
  std::vector<int> labelConnectedComponents(
    const std::vector<bool> & obstacle_mask,
    int width, int height) const;

  int countClustersInRadius(
    int cx, int cy,
    const std::vector<int> & labels,
    int width, int height,
    int radius_cells) const;

  const Parameters & params_;
  double max_expected_clusters_{8.0};  // for normalization
};

}  // namespace vf_robot_controller::gcf
