#pragma once

#include <vector>
#include <Eigen/Dense>
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "vf_robot_controller/parameter_handler.hpp"

namespace vf_robot_controller::gcf
{

/**
 * @brief CorridorAnalyzer — measures traversable width perpendicular to path.
 *
 * For each waypoint on the upcoming path, shoots rays perpendicular to the
 * path heading and finds the first occupied cell in each direction.
 * The width = left_clearance + right_clearance.
 *
 * Uses hysteresis to avoid oscillation from noisy costmap readings.
 * Width estimate used by:
 *   - DOF allocator (tight width → more knots)
 *   - VelocityScheduler (tight width → lower speed limit)
 *   - NarrowPassageCritic (tight width → higher penalty weight)
 */
class CorridorAnalyzer
{
public:
  struct CorridorInfo
  {
    double width{1e6};           // traversable width (m)
    double left_clearance{1e6};  // clearance to left obstacle (m)
    double right_clearance{1e6}; // clearance to right obstacle (m)
    bool   is_narrow{false};     // width < robot_width * narrow_factor
    bool   is_doorway{false};    // very narrow transition (width < 1.3 * robot_width)
  };

  explicit CorridorAnalyzer(const Parameters & params);

  /**
   * @brief Compute corridor info for all waypoints along the look-ahead.
   * @param waypoints  sequence of (x,y) points on the global path ahead
   * @param headings   corresponding headings (rad)
   * @param costmap    current local costmap
   * @return one CorridorInfo per waypoint
   */
  std::vector<CorridorInfo> analyze(
    const std::vector<Eigen::Vector2d> & waypoints,
    const std::vector<double> & headings,
    const nav2_costmap_2d::Costmap2D * costmap) const;

  /**
   * @brief Single-point corridor query (for GCF 2D component).
   */
  CorridorInfo analyzePoint(
    double x, double y, double heading,
    const nav2_costmap_2d::Costmap2D * costmap) const;

private:
  double raycastDistance(
    double x, double y,
    double dir_x, double dir_y,
    double max_dist,
    const nav2_costmap_2d::Costmap2D * costmap) const;

  const Parameters & params_;
  double hysteresis_width_{0.0};  // smoothed width from last call
};

}  // namespace vf_robot_controller::gcf
