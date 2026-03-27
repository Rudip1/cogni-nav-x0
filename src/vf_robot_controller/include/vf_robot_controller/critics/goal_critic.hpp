#pragma once
#include "vf_robot_controller/critics/critic_function.hpp"

namespace vf_robot_controller::critics
{
/**
 * @brief Penalizes terminal distance and heading error to goal.
 * Only active when goal is within the local horizon.
 */
class GoalCritic : public CriticFunction
{
public:
  void initialize(rclcpp_lifecycle::LifecycleNode::SharedPtr n,
                  const std::string & nm, const Parameters & p) override;
  double score(const CriticData & d) const override;
private:
  double goal_activation_dist_{1.5};  // m — activate when goal is closer than this
  double heading_weight_{0.5};
};
}  // namespace vf_robot_controller::critics
