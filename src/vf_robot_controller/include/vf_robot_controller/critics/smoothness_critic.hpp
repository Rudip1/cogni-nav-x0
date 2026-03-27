#pragma once
#include "vf_robot_controller/critics/critic_function.hpp"

namespace vf_robot_controller::critics
{
/**
 * @brief Penalizes high curvature and jerk in the trajectory.
 *
 * High curvature in tight spaces causes the robot to scrape obstacles
 * even if the path centre is clear. Penalizing curvature forces wider arcs
 * which are safer in cluttered rooms.
 *
 * Jerk penalty prevents aggressive velocity changes that could knock over
 * medical equipment or disturb patients.
 */
class SmoothnessCritic : public CriticFunction
{
public:
  void initialize(rclcpp_lifecycle::LifecycleNode::SharedPtr n,
                  const std::string & nm, const Parameters & p) override;
  double score(const CriticData & d) const override;
private:
  double max_curvature_{2.0};    // 1/m — penalize above this
  double jerk_weight_{0.2};      // relative weight of jerk vs curvature
  int    curvature_samples_{10}; // evaluation points along trajectory
};
}  // namespace vf_robot_controller::critics
