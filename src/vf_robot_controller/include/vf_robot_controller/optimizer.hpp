#pragma once

#include <memory>
#include <vector>

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include "vf_robot_controller/parameter_handler.hpp"
#include "vf_robot_controller/critic_manager.hpp"
#include "vf_robot_controller/gcf/geometric_complexity_field.hpp"
#include "vf_robot_controller/models/trajectory.hpp"
#include "vf_robot_controller/models/state.hpp"
#include "vf_robot_controller/tools/noise_generator.hpp"
#include "vf_robot_controller/tools/velocity_scheduler.hpp"
#include "vf_robot_controller/tools/utils.hpp"

namespace vf_robot_controller
{

/**
 * @brief Optimizer — the core adaptive B-spline MPPI loop.
 *
 * One cycle of optimize() does:
 *   1. Extract look-ahead waypoints from global plan
 *   2. Ask GCF for per-segment complexity
 *   3. Build SegmentMap (knot density allocation)
 *   4. Fit base B-spline to look-ahead waypoints with allocated knots
 *   5. Ask VelocityScheduler for v_max at current timestep
 *   6. Generate N perturbed trajectories via NoiseGenerator
 *   7. Roll out all N trajectories through kinematic model
 *   8. Score all N trajectories via CriticManager
 *   9. Compute MPPI importance weights
 *  10. Update base trajectory with weighted mean perturbation
 *  11. Repeat steps 6-10 for num_iterations
 *  12. Convert first step of best trajectory to (v, omega) command
 *  13. Clamp command to kinematic limits and v_max from VelocityScheduler
 */
class Optimizer
{
public:
  Optimizer() = default;

  void initialize(
    rclcpp_lifecycle::LifecycleNode::SharedPtr node,
    const Parameters & params,
    std::shared_ptr<gcf::GeometricComplexityField> gcf);

  void activate();
  void deactivate();

  /**
   * @brief Main optimization call. Returns best (v, omega).
   * Called every control cycle from controller.cpp.
   */
  geometry_msgs::msg::Twist optimize(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & current_vel,
    const nav_msgs::msg::Path & global_plan,
    const nav2_costmap_2d::Costmap2D * costmap,
    const sensor_msgs::msg::PointCloud2::SharedPtr & pointcloud);

  /** @brief Access best trajectory for visualization. */
  const models::BSplineTrajectory & bestTrajectory() const { return best_trajectory_; }

  /** @brief Access all sampled trajectories for visualization. */
  const std::vector<models::BSplineTrajectory> & sampledTrajectories() const
  { return sampled_trajectories_; }

  /** @brief Access costs for visualization. */
  const std::vector<double> & sampleCosts() const { return sample_costs_; }

private:
  // ── Internal helpers ─────────────────────────────────────────────────────
  models::BSplineTrajectory buildBaseTrajectory(
    const std::vector<Eigen::Vector2d> & waypoints,
    const models::SegmentMap & seg_map) const;

  models::StateSequence rollout(
    const models::BSplineTrajectory & traj,
    const geometry_msgs::msg::PoseStamped & start_pose,
    const geometry_msgs::msg::Twist & start_vel) const;

  geometry_msgs::msg::Twist trajectoryToCmd(
    const models::BSplineTrajectory & traj,
    const geometry_msgs::msg::PoseStamped & pose,
    double v_max) const;

  // ── Owned components ─────────────────────────────────────────────────────
  std::unique_ptr<CriticManager>             critic_manager_;
  std::unique_ptr<tools::NoiseGenerator>     noise_gen_;
  std::unique_ptr<tools::VelocityScheduler>  vel_scheduler_;
  std::shared_ptr<gcf::GeometricComplexityField> gcf_;

  // ── State ─────────────────────────────────────────────────────────────────
  models::BSplineTrajectory              best_trajectory_;
  std::vector<models::BSplineTrajectory> sampled_trajectories_;
  std::vector<double>                    sample_costs_;

  const Parameters * params_{nullptr};
  rclcpp::Logger logger_{rclcpp::get_logger("VFOptimizer")};
};

}  // namespace vf_robot_controller
