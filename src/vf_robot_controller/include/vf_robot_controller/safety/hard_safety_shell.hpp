#pragma once

#include <memory>
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "vf_robot_controller/parameter_handler.hpp"

namespace vf_robot_controller::safety
{

/**
 * @brief HardSafetyShell — decoupled collision guard.
 *
 * This runs AFTER the optimizer produces a cmd_vel.
 * It is completely independent of the optimizer, critics, and GCF.
 *
 * Algorithm every control cycle:
 *   1. Take proposed (v, ω) from optimizer output
 *   2. Compute swept volume of robot rectangular footprint over
 *      next check_horizon_s seconds at that velocity
 *   3. Query swept volume against:
 *      a. 2D costmap (lethal cells)
 *      b. 3D voxel grid (chair legs etc.) if available
 *   4. If ANY occupied cell/voxel is inside swept volume → cmd_vel = ZERO
 *   5. Otherwise → pass cmd_vel through unchanged
 *
 * This provides a HARD guarantee: the robot never moves into a space
 * where a collision is imminent, regardless of optimizer quality.
 *
 * Safety-critical design choices:
 *   - No dynamic parameters that can accidentally disable it
 *   - Runs synchronously in computeVelocityCommands() after optimizer
 *   - Has its own independent costmap query (not shared with optimizer)
 *   - Logs every veto with WARN so operator can detect issues
 */
class HardSafetyShell
{
public:
  explicit HardSafetyShell(const Parameters & params);

  /**
   * @brief Check proposed cmd_vel and return safe version.
   * Returns cmd_vel unchanged if safe, returns zero twist if collision detected.
   * @param proposed   velocity from optimizer
   * @param pose       current robot pose
   * @param costmap    current local costmap (queried independently)
   * @param pointcloud 3D data, may be nullptr
   * @return safe cmd_vel (zero if veto triggered)
   */
  geometry_msgs::msg::Twist check(
    const geometry_msgs::msg::Twist & proposed,
    const geometry_msgs::msg::PoseStamped & pose,
    const nav2_costmap_2d::Costmap2D * costmap,
    const sensor_msgs::msg::PointCloud2::SharedPtr & pointcloud) const;

  /** @brief True if last check() triggered a veto. */
  bool lastCheckVetoed() const { return last_vetoed_; }

  /** @brief Number of consecutive vetoes (used for stuck detection). */
  int consecutiveVetoes() const { return consecutive_vetoes_; }

private:
  bool checkSweptVolume2D(
    const geometry_msgs::msg::Twist & vel,
    const geometry_msgs::msg::PoseStamped & pose,
    const nav2_costmap_2d::Costmap2D * costmap) const;

  bool checkSweptVolume3D(
    const geometry_msgs::msg::Twist & vel,
    const geometry_msgs::msg::PoseStamped & pose,
    const sensor_msgs::msg::PointCloud2::SharedPtr & cloud) const;

  const Parameters & params_;
  double check_horizon_s_{0.3};   // seconds of motion to check ahead
  int    footprint_samples_{8};   // number of orientation steps in swept volume

  mutable bool last_vetoed_{false};
  mutable int  consecutive_vetoes_{0};
};

}  // namespace vf_robot_controller::safety
