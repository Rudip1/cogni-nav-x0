#include "vf_robot_controller/critics/oscillation_critic.hpp"
#include "nav2_util/node_utils.hpp"
#include <cmath>

namespace vf_robot_controller::critics
{

void OscillationCritic::initialize(
  rclcpp_lifecycle::LifecycleNode::SharedPtr node,
  const std::string & nm, const Parameters & /*p*/)
{
  name_    = nm;
  weight_  = 0.8;

  nav2_util::declare_parameter_if_not_declared(
    node, nm + ".max_reversals", rclcpp::ParameterValue(2));
  nav2_util::declare_parameter_if_not_declared(
    node, nm + ".min_omega", rclcpp::ParameterValue(0.05));

  node->get_parameter(nm + ".max_reversals", max_reversals_);
  node->get_parameter(nm + ".min_omega",     min_omega_);
}

double OscillationCritic::score(const CriticData & d) const
{
  const auto & rollout = d.rollout;
  if (rollout.size() < 3) { return 0.0; }

  int reversals = 0;
  int prev_sign = 0;  // 0 = undecided

  for (const auto & state : rollout) {
    int sign = 0;
    if (state.vel_theta >  min_omega_) { sign =  1; }
    if (state.vel_theta < -min_omega_) { sign = -1; }

    if (sign != 0) {
      if (prev_sign != 0 && sign != prev_sign) {
        ++reversals;
      }
      prev_sign = sign;
    }
  }

  const double ratio = static_cast<double>(reversals) /
                       static_cast<double>(max_reversals_);
  return weight_ * std::min(ratio, 1.0);
}

}  // namespace vf_robot_controller::critics
