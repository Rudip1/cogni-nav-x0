#pragma once

#include <memory>
#include <string>
#include <vector>

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

/**
 * @brief CriticManager — loads, owns, and calls all critics.
 *
 * Uses pluginlib to load critics at runtime.
 * Applies GCF-driven weight scaling before each scoring call:
 *   - obstacle_critic weight  → scales UP when gcf_mean > tight_threshold
 *   - path_follow_critic weight → scales DOWN when gcf_mean > tight_threshold
 * All other critics keep their base weights.
 */
class CriticManager
{
public:
  CriticManager() = default;

  void initialize(
    rclcpp_lifecycle::LifecycleNode::SharedPtr node,
    const Parameters & params);

  /**
   * @brief Score all N trajectory samples. Returns cost vector length N.
   * Parallelised with std::for_each + execution::par_unseq if available.
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
   * @brief Score a single trajectory (used by safety shell check).
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
  rclcpp::Logger logger_{rclcpp::get_logger("CriticManager")};
};

}  // namespace vf_robot_controller
