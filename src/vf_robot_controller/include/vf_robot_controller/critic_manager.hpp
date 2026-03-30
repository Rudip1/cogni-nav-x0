#pragma once

#include <memory>
#include <string>
#include <vector>
#include <mutex>

#include "pluginlib/class_loader.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav_msgs/msg/path.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include "vf_robot_controller/critics/critic_function.hpp"
#include "vf_robot_controller/gcf/geometric_complexity_field.hpp"
#include "vf_robot_controller/models/trajectory.hpp"
#include "vf_robot_controller/parameter_handler.hpp"

namespace vf_robot_controller
{

class CriticManager
{
public:
  CriticManager() = default;

  void initialize(
    rclcpp_lifecycle::LifecycleNode::SharedPtr node,
    const Parameters & params);

  /**
   * @brief Set dynamic weights from meta-critic network.
   * Called every control cycle by the Optimizer with values from WeightAdapter.
   * Size must match number of loaded critics exactly.
   * Falls back to uniform if size mismatches.
   */
  void setWeights(const std::vector<float> & weights);

  /**
   * @brief Score all N trajectory samples. Returns cost vector length N.
   * Applies dynamic_weights_[i] to each critic's raw score.
   * GCF-driven scaling acts as a sub-multiplier on top.
   */
  std::vector<double> scoreAll(
    const std::vector<models::BSplineTrajectory> & trajectories,
    const std::vector<models::StateSequence> & rollouts,
    const nav2_costmap_2d::Costmap2D * costmap,
    const std::shared_ptr<gcf::GeometricComplexityField> & gcf,
    const nav_msgs::msg::Path * global_plan,
    const geometry_msgs::msg::PoseStamped * goal,
    const sensor_msgs::msg::PointCloud2::SharedPtr & pointcloud) const;

  /**
   * @brief Per-critic score matrix for COLLECT mode data recording.
   *
   * Returns flat vector size N*K, row-major: [traj0_c0, traj0_c1...traj0_cK, traj1_c0...]
   * Dynamic weights are applied (same as scoreSingle).
   * Called ONCE after the MPPI loop — NOT inside it. Zero impact on runtime performance.
   * Used by Optimizer::perCriticScores() which is consumed by DataRecorder.
   * Both IMITATION and META_CRITIC training methods read this matrix.
   */
  std::vector<double> scoreAllPerCritic(
    const std::vector<models::BSplineTrajectory> & trajectories,
    const std::vector<models::StateSequence> & rollouts,
    const nav2_costmap_2d::Costmap2D * costmap,
    const std::shared_ptr<gcf::GeometricComplexityField> & gcf,
    const nav_msgs::msg::Path * global_plan,
    const geometry_msgs::msg::PoseStamped * goal,
    const sensor_msgs::msg::PointCloud2::SharedPtr & pointcloud) const;

  /**
   * @brief Score a single trajectory (used by safety shell check).
   * Uses current dynamic_weights_.
   */
  double scoreSingle(
    const models::BSplineTrajectory & traj,
    const models::StateSequence & rollout,
    const nav2_costmap_2d::Costmap2D * costmap,
    const std::shared_ptr<gcf::GeometricComplexityField> & gcf,
    const nav_msgs::msg::Path * global_plan,
    const geometry_msgs::msg::PoseStamped * goal,
    const sensor_msgs::msg::PointCloud2::SharedPtr & pointcloud) const;

  const std::vector<std::shared_ptr<critics::CriticFunction>> & critics() const
  { return critics_; }

  /** @brief Returns current dynamic weight for critic at slot index. */
  float dynamicWeight(int slot) const;

private:
  critics::CriticData buildData(
    const models::BSplineTrajectory & traj,
    const models::StateSequence & rollout,
    const nav2_costmap_2d::Costmap2D * costmap,
    const std::shared_ptr<gcf::GeometricComplexityField> & gcf,
    const nav_msgs::msg::Path * global_plan,
    const geometry_msgs::msg::PoseStamped * goal,
    const sensor_msgs::msg::PointCloud2::SharedPtr & pointcloud) const;

  std::vector<std::shared_ptr<critics::CriticFunction>> critics_;
  std::shared_ptr<pluginlib::ClassLoader<critics::CriticFunction>> loader_;
  const Parameters * params_{nullptr};

  // Dynamic weights from meta-critic — one per critic, same slot order as critics_
  mutable std::mutex weights_mutex_;
  std::vector<float> dynamic_weights_;  // set by setWeights(), applied in scoreAll()

  rclcpp::Logger logger_{rclcpp::get_logger("CriticManager")};
};

}  // namespace vf_robot_controller
