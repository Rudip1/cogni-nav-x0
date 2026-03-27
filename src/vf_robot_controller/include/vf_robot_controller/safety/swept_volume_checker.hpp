#pragma once

#include <vector>
#include <Eigen/Dense>
#include "vf_robot_controller/parameter_handler.hpp"

namespace vf_robot_controller::safety
{

/**
 * @brief SweptVolumeChecker — computes cells covered by robot motion.
 *
 * Given current pose and velocity, computes all (x,y) cells that the
 * robot's rectangular footprint will occupy over the next dt seconds.
 *
 * The robot footprint is a rectangle: length × width.
 * At each of N orientation steps (simulating the arc), we compute
 * the 4 corners of the footprint and rasterize the filled rectangle
 * into grid cells. The union of all cells = swept volume.
 *
 * This is more accurate than a circle approximation — critical for
 * tight passages where the difference between 0.38m and 0.42m matters.
 */
class SweptVolumeChecker
{
public:
  explicit SweptVolumeChecker(const Parameters & params);

  struct FootprintCell { int mx, my; };  // map coordinates

  /**
   * @brief Compute all map cells in swept volume.
   * @param x0,y0,yaw0  current robot pose
   * @param v,omega      commanded velocity
   * @param dt           time horizon (seconds)
   * @param resolution   costmap resolution (m/cell)
   * @param origin_x/y   costmap origin in world frame
   * @param n_steps      number of pose interpolation steps
   */
  std::vector<FootprintCell> computeSweptCells(
    double x0, double y0, double yaw0,
    double v, double omega,
    double dt, double resolution,
    double origin_x, double origin_y,
    int n_steps = 8) const;

  /**
   * @brief Compute 3D swept bounding box for pointcloud query.
   * Returns list of (x,y) world points covering the swept area.
   */
  std::vector<Eigen::Vector2d> computeSweptWorldPoints(
    double x0, double y0, double yaw0,
    double v, double omega, double dt,
    int n_steps = 8) const;

private:
  std::vector<Eigen::Vector2d> rectangleCorners(
    double cx, double cy, double yaw,
    double half_length, double half_width) const;

  std::vector<FootprintCell> rasterizeFootprint(
    const std::vector<Eigen::Vector2d> & corners,
    double resolution, double origin_x, double origin_y) const;

  const Parameters & params_;
};

}  // namespace vf_robot_controller::safety
