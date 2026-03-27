// ─────────────────────────────────────────────────────────────────────────────
// obstacle_critic.hpp
// ─────────────────────────────────────────────────────────────────────────────
#pragma once
#include "vf_robot_controller/critics/critic_function.hpp"

namespace vf_robot_controller::critics
{
/** @brief Penalizes proximity to 2D costmap obstacles. */
class ObstacleCritic : public CriticFunction
{
public:
  void initialize(rclcpp_lifecycle::LifecycleNode::SharedPtr n,
                  const std::string & nm, const Parameters & p) override;
  double score(const CriticData & d) const override;
private:
  double critical_cost_{253.0};   // costmap lethal threshold
  double repulsion_radius_{0.5};  // m — start penalizing at this distance
};
}  // namespace vf_robot_controller::critics
