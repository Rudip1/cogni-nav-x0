#include "vf_robot_controller/critics/velocity_critic.hpp"
#include "nav2_util/node_utils.hpp"
#include <cmath>

namespace vf_robot_controller::critics
{
void VelocityCritic::initialize(
  rclcpp_lifecycle::LifecycleNode::SharedPtr node,
  const std::string & nm, const Parameters & /*p*/)
{
  name_ = nm; weight_ = 0.3;
  nav2_util::declare_parameter_if_not_declared(
    node, nm + ".slowness_penalty_weight", rclcpp::ParameterValue(0.1));
  node->get_parameter(nm + ".slowness_penalty_weight", slowness_penalty_weight_);
}

double VelocityCritic::score(const CriticData & d) const
{
  if (!d.gcf) return 0.0;

  // Penalise going too fast in high-GCF regions
  const auto pts = d.trajectory.sample(10);
  double total = 0.0;

  for (const auto & pt : pts) {
    const auto cell = d.gcf->query(pt.x(), pt.y());
    // Check speed via trajectory derivative magnitude
    // (approximate — real speed check is in VelocityScheduler)
    if (cell.complexity > 0.5) {
      const double vel_mag = d.trajectory.derivative(0.0).norm();
      if (vel_mag > 0.2) {
        total += weight_ * (vel_mag - 0.2) * cell.complexity;
      }
    }
  }
  return total / 10.0;
}
}  // namespace vf_robot_controller::critics
