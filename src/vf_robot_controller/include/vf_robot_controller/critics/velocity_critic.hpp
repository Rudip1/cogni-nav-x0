#pragma once
#include "vf_robot_controller/critics/critic_function.hpp"

namespace vf_robot_controller::critics
{
/**
 * @brief Enforces GCF-driven velocity limits and penalizes excessive speed.
 *
 * In tight/cluttered regions: penalizes any trajectory exceeding
 * max_speed_in_tight (default 0.15 m/s).
 * In open regions: penalizes very slow trajectories (encourages efficiency).
 *
 * This is separate from VelocityScheduler — VelocityScheduler sets the
 * hard limit, this critic scores samples that approach the limit too
 * aggressively within the optimizer.
 */
class VelocityCritic : public CriticFunction
{
public:
  void initialize(rclcpp_lifecycle::LifecycleNode::SharedPtr n,
                  const std::string & nm, const Parameters & p) override;
  double score(const CriticData & d) const override;
private:
  double slowness_penalty_weight_{0.1};  // penalize unnecessary slowness
};
}  // namespace vf_robot_controller::critics
