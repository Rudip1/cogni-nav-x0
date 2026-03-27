#pragma once
#include "vf_robot_controller/critics/critic_function.hpp"

namespace vf_robot_controller::critics
{
/**
 * @brief Time-expanded dynamic obstacle avoidance.
 *
 * Projects each dynamic obstacle forward in time using constant-velocity
 * prediction. Penalizes trajectories whose (x,y,t) occupies the same
 * space-time cell as a predicted obstacle position.
 *
 * This is what avoids the nurse walking into the robot's path — the robot
 * sees the nurse moving, predicts where she will be in 1.5s, and routes
 * around that future position.
 */
class DynamicObstacleCritic : public CriticFunction
{
public:
  void initialize(rclcpp_lifecycle::LifecycleNode::SharedPtr n,
                  const std::string & nm, const Parameters & p) override;
  double score(const CriticData & d) const override;
private:
  double prediction_horizon_{2.0};   // seconds ahead to predict obstacles
  double collision_radius_{0.6};     // m — robot + person safety bubble
  double time_decay_{0.5};           // cost decreases with prediction distance
};
}  // namespace vf_robot_controller::critics
