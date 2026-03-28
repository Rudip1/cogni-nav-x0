#include "vf_robot_controller/critic_manager.hpp"
#include "vf_robot_controller/critics/obstacle_critic.hpp"
#include "vf_robot_controller/critics/volumetric_critic.hpp"
#include "vf_robot_controller/critics/dynamic_obstacle_critic.hpp"
#include "vf_robot_controller/critics/path_follow_critic.hpp"
#include "vf_robot_controller/critics/smoothness_critic.hpp"
#include "vf_robot_controller/critics/goal_critic.hpp"
#include "vf_robot_controller/critics/velocity_critic.hpp"
#include "vf_robot_controller/critics/corridor_critic.hpp"
#include "vf_robot_controller/critics/clearance_critic.hpp"
#include "vf_robot_controller/critics/oscillation_critic.hpp"
#include <algorithm>

namespace vf_robot_controller
{

void CriticManager::initialize(
  rclcpp_lifecycle::LifecycleNode::SharedPtr node,
  const Parameters & params)
{
  params_ = &params;

  // Slot order must match critics.xml and the meta-critic network output:
  //   0 ObstacleCritic  1 VolumetricCritic  2 DynamicObstacleCritic
  //   3 PathFollowCritic  4 SmoothnessCritic  5 GoalCritic
  //   6 VelocityCritic  7 CorridorCritic  8 ClearanceCritic  9 OscillationCritic

  auto make = [&](auto critic_ptr, const std::string & name) {
    critic_ptr->initialize(node, name, params);
    critics_.push_back(critic_ptr);
  };

  for (const auto & name : params.critics) {
    if      (name == "ObstacleCritic")        make(std::make_shared<critics::ObstacleCritic>(), name);
    else if (name == "VolumetricCritic")      make(std::make_shared<critics::VolumetricCritic>(), name);
    else if (name == "DynamicObstacleCritic") make(std::make_shared<critics::DynamicObstacleCritic>(), name);
    else if (name == "PathFollowCritic")      make(std::make_shared<critics::PathFollowCritic>(), name);
    else if (name == "SmoothnessCritic")      make(std::make_shared<critics::SmoothnessCritic>(), name);
    else if (name == "GoalCritic")            make(std::make_shared<critics::GoalCritic>(), name);
    else if (name == "VelocityCritic")        make(std::make_shared<critics::VelocityCritic>(), name);
    else if (name == "CorridorCritic")        make(std::make_shared<critics::CorridorCritic>(), name);
    else if (name == "ClearanceCritic")       make(std::make_shared<critics::ClearanceCritic>(), name);
    else if (name == "OscillationCritic")     make(std::make_shared<critics::OscillationCritic>(), name);
    else RCLCPP_WARN(logger_, "Unknown critic: %s — skipping", name.c_str());
  }

  // Initialise dynamic weights to uniform
  const float uniform = 1.0f / static_cast<float>(critics_.size());
  dynamic_weights_.assign(critics_.size(), uniform);

  RCLCPP_INFO(logger_, "CriticManager: loaded %zu critics", critics_.size());
}

void CriticManager::setWeights(const std::vector<float> & weights)
{
  std::lock_guard<std::mutex> lock(weights_mutex_);
  if (weights.size() != critics_.size()) {
    RCLCPP_WARN(logger_,
      "setWeights: size mismatch (%zu vs %zu) — keeping previous weights",
      weights.size(), critics_.size());
    return;
  }
  dynamic_weights_ = weights;
}

float CriticManager::dynamicWeight(int slot) const
{
  std::lock_guard<std::mutex> lock(weights_mutex_);
  if (slot < 0 || slot >= static_cast<int>(dynamic_weights_.size())) {
    return 1.0f;
  }
  return dynamic_weights_[slot];
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
  double gcf_mean = 0.0;
  if (gcf) {
    const auto pts = traj.sample(10);
    for (const auto & p : pts) gcf_mean += gcf->query(p.x(), p.y()).complexity;
    gcf_mean /= 10.0;
  }

  const double tight = std::clamp(
    (gcf_mean - params_->tight_gcf_threshold) /
    (1.0 - params_->tight_gcf_threshold), 0.0, 1.0);

  const double obs_scale = params_->obstacle_weight_open +
    tight * (params_->obstacle_weight_tight - params_->obstacle_weight_open);
  const double pf_scale = params_->path_follow_weight_open +
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
    const auto & rollout = (i < static_cast<int>(rollouts.size())) ?
      rollouts[i] : models::StateSequence{};
    costs[i] = scoreSingle(
      trajectories[i], rollout, costmap, gcf, global_plan, goal, pointcloud);
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
  const auto data = buildData(
    traj, rollout, costmap, gcf, global_plan, goal, pointcloud);

  // Snapshot weights once for this trajectory
  std::vector<float> w;
  {
    std::lock_guard<std::mutex> lock(weights_mutex_);
    w = dynamic_weights_;
  }

  double total = 0.0;
  for (int i = 0; i < static_cast<int>(critics_.size()); ++i) {
    // dynamic weight from meta-critic (or uniform fallback)
    const double dyn_w = (i < static_cast<int>(w.size())) ?
      static_cast<double>(w[i]) : 1.0;
    total += dyn_w * critics_[i]->score(data);
  }
  return total;
}

}  // namespace vf_robot_controller
