#pragma once
#include "vf_robot_controller/critics/critic_function.hpp"

namespace vf_robot_controller::critics
{
/**
 * @brief Penalizes lateral deviation and heading error from global path.
 *
 * Weight is REDUCED by GCF in tight/cluttered regions — the robot is
 * allowed to deviate from the global path to navigate around obstacles.
 * Weight is INCREASED in open regions — follow the path efficiently.
 */
class PathFollowCritic : public CriticFunction
{
public:
  void initialize(rclcpp_lifecycle::LifecycleNode::SharedPtr n,
                  const std::string & nm, const Parameters & p) override;
  double score(const CriticData & d) const override;
private:
  double max_lateral_dev_{1.0};    // m — beyond this = maximum penalty
  double heading_weight_{0.3};     // relative weight of heading vs lateral
};
}  // namespace vf_robot_controller::critics
