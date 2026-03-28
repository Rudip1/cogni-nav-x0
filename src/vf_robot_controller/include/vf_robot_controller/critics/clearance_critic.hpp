#pragma once
#include "vf_robot_controller/critics/critic_function.hpp"

namespace vf_robot_controller::critics
{
/**
 * @brief ClearanceCritic — continuous minimum clearance margin along trajectory.
 *
 * Reynolds separation group. Slot 8.
 *
 * Unlike ObstacleCritic which uses costmap cell costs (binary-ish),
 * this critic reads actual metric clearance from the GCF and applies
 * a continuous inverse penalty. This gives the optimizer a smooth
 * gradient to move trajectories away from walls even when they are
 * not yet in the lethal zone.
 *
 * Cost = weight_ * (1/N) * sum_i( max(0, desired_clearance - clearance_i)^2 )
 */
class ClearanceCritic : public CriticFunction
{
public:
  void initialize(rclcpp_lifecycle::LifecycleNode::SharedPtr n,
                  const std::string & nm, const Parameters & p) override;
  double score(const CriticData & d) const override;

private:
  int    samples_{10};              // evaluation points along trajectory
  double desired_clearance_{0.4};   // m — target minimum clearance
};
}  // namespace vf_robot_controller::critics
