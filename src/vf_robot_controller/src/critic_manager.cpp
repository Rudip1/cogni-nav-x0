#include "vf_robot_controller/critic_manager.hpp"
#include "vf_robot_controller/critics/obstacle_critic.hpp"
#include "vf_robot_controller/critics/volumetric_critic.hpp"
#include "vf_robot_controller/critics/dynamic_obstacle_critic.hpp"
#include "vf_robot_controller/critics/path_follow_critic.hpp"
#include "vf_robot_controller/critics/smoothness_critic.hpp"
#include "vf_robot_controller/critics/goal_critic.hpp"
#include "vf_robot_controller/critics/velocity_critic.hpp"
#include <algorithm>
#include <execution>

namespace vf_robot_controller
{

void CriticManager::initialize(
  rclcpp_lifecycle::LifecycleNode::SharedPtr node,
  const Parameters & params)
{
  params_ = &params;

  // Instantiate critics directly (no pluginlib overhead for built-ins)
  // This can be extended with pluginlib for custom critics later.

  for (const auto & name : params.critics) {
    if (name == "ObstacleCritic") {
      auto c = std::make_shared<critics::ObstacleCritic>();
      c->initialize(node, name, params);
      critics_.push_back(c);
    } else if (name == "VolumetricCritic") {
      auto c = std::make_shared<critics::VolumetricCritic>();
      c->initialize(node, name, params);
      critics_.push_back(c);
    } else if (name == "DynamicObstacleCritic") {
      auto c = std::make_shared<critics::DynamicObstacleCritic>();
      c->initialize(node, name, params);
      critics_.push_back(c);
    } else if (name == "PathFollowCritic") {
      auto c = std::make_shared<critics::PathFollowCritic>();
      c->initialize(node, name, params);
      critics_.push_back(c);
    } else if (name == "SmoothnessCritic") {
      auto c = std::make_shared<critics::SmoothnessCritic>();
      c->initialize(node, name, params);
      critics_.push_back(c);
    } else if (name == "GoalCritic") {
      auto c = std::make_shared<critics::GoalCritic>();
      c->initialize(node, name, params);
      critics_.push_back(c);
    } else if (name == "VelocityCritic") {
      auto c = std::make_shared<critics::VelocityCritic>();
      c->initialize(node, name, params);
      critics_.push_back(c);
    } else {
      RCLCPP_WARN(logger_, "Unknown critic: %s — skipping", name.c_str());
    }
  }
  RCLCPP_INFO(logger_, "CriticManager: loaded %zu critics", critics_.size());
}

critics::CriticData CriticManager::buildData(
  const models::BSplineTrajectory & traj,
  const models::StateSequence & rollout,
  const nav2_costmap_2d::Costmap2D * costmap,
  const std::shared_ptr<gcf::GeometricComplexityField> & gcf,
  const nav_msgs::msg::Path * global_plan,
  const geometry_msgs::msg::PoseStamped * goal,
  const sensor_msgs::msg::PointCloud2::SharedPtr & pointcloud) const
{
  // Compute GCF-driven weight scales once per trajectory
  double gcf_mean = 0.0;
  if (gcf) {
    const auto pts = traj.sample(10);
    for (const auto & p : pts) gcf_mean += gcf->query(p.x(), p.y()).complexity;
    gcf_mean /= 10.0;
  }

  const double tight   = std::clamp(
    (gcf_mean - params_->tight_gcf_threshold) /
    (1.0 - params_->tight_gcf_threshold), 0.0, 1.0);

  const double obs_scale = params_->obstacle_weight_open +
    tight * (params_->obstacle_weight_tight - params_->obstacle_weight_open);
  const double pf_scale  = params_->path_follow_weight_open +
    tight * (params_->path_follow_weight_tight - params_->path_follow_weight_open);

  return critics::CriticData{
    traj, rollout, costmap, gcf, global_plan, goal, pointcloud,
    gcf_mean,
    obs_scale / std::max(params_->obstacle_weight_open, 1e-6),
    pf_scale  / std::max(params_->path_follow_weight_open, 1e-6)
  };
}

std::vector<double> CriticManager::scoreAll(
  const std::vector<models::BSplineTrajectory> & trajectories,
  const std::vector<models::StateSequence> & rollouts,
  const nav2_costmap_2d::Costmap2D * costmap,
  const std::shared_ptr<gcf::GeometricComplexityField> & gcf,
  const nav_msgs::msg::Path * global_plan,
  const geometry_msgs::msg::PoseStamped * goal,
  const sensor_msgs::msg::PointCloud2::SharedPtr & pointcloud) const
{
  const int N = static_cast<int>(trajectories.size());
  std::vector<double> costs(N, 0.0);

  for (int i = 0; i < N; ++i) {
    const auto & rollout = (i < static_cast<int>(rollouts.size())) ? rollouts[i] : models::StateSequence{};
    costs[i] = scoreSingle(trajectories[i], rollout, costmap, gcf, global_plan, goal, pointcloud);
  }
  return costs;
}

double CriticManager::scoreSingle(
  const models::BSplineTrajectory & traj,
  const models::StateSequence & rollout,
  const nav2_costmap_2d::Costmap2D * costmap,
  const std::shared_ptr<gcf::GeometricComplexityField> & gcf,
  const nav_msgs::msg::Path * global_plan,
  const geometry_msgs::msg::PoseStamped * goal,
  const sensor_msgs::msg::PointCloud2::SharedPtr & pointcloud) const
{
  const auto data = buildData(traj, rollout, costmap, gcf, global_plan, goal, pointcloud);
  double total = 0.0;
  for (const auto & critic : critics_) {
    total += critic->score(data);
  }
  return total;
}

}  // namespace vf_robot_controller
