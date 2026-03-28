#pragma once
#include "vf_robot_controller/critics/critic_function.hpp"

namespace vf_robot_controller::critics
{
/**
 * @brief CorridorCritic — penalises lateral offset from corridor centreline.
 *
 * Reynolds cohesion group. Slot 7.
 *
 * Uses corridor_analyzer via the GCF to find the centreline at each sampled
 * trajectory point, then sums squared lateral deviations. Encourages the
 * robot to stay centred in the passage, which maximises clearance margin
 * on both sides simultaneously without the obstacle critic needing to
 * explicitly reason about left vs right walls.
 *
 * Cost = weight_ * (1/N) * sum_i( lateral_offset_i^2 )
 */
class CorridorCritic : public CriticFunction
{
public:
  void initialize(rclcpp_lifecycle::LifecycleNode::SharedPtr n,
                  const std::string & nm, const Parameters & p) override;
  double score(const CriticData & d) const override;

private:
  int    samples_{10};          // evaluation points along trajectory
  double max_offset_{0.5};      // m — offsets beyond this are clamped
};
}  // namespace vf_robot_controller::critics
