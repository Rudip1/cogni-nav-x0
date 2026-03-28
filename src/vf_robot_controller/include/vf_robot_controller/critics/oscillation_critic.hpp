#pragma once
#include "vf_robot_controller/critics/critic_function.hpp"

namespace vf_robot_controller::critics
{
/**
 * @brief OscillationCritic — penalises angular velocity sign reversals.
 *
 * Reynolds alignment group. Slot 9.
 *
 * Trajectories that zigzag or oscillate in heading produce uncomfortable
 * motion and can disturb patients. This critic counts sign changes in the
 * angular velocity sequence derived from the rollout and penalises them.
 *
 * Cost = weight_ * (reversal_count / max_reversals)
 */
class OscillationCritic : public CriticFunction
{
public:
  void initialize(rclcpp_lifecycle::LifecycleNode::SharedPtr n,
                  const std::string & nm, const Parameters & p) override;
  double score(const CriticData & d) const override;

private:
  int    max_reversals_{2};      // reversals above this give cost = weight_
  double min_omega_{0.05};       // rad/s — below this is treated as zero (deadband)
};
}  // namespace vf_robot_controller::critics
