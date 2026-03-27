#include "vf_robot_controller/critics/smoothness_critic.hpp"
#include "nav2_util/node_utils.hpp"
#include <cmath>

namespace vf_robot_controller::critics
{
void SmoothnessCritic::initialize(
  rclcpp_lifecycle::LifecycleNode::SharedPtr node,
  const std::string & nm, const Parameters & /*p*/)
{
  name_ = nm; weight_ = 0.5;
  nav2_util::declare_parameter_if_not_declared(
    node, nm + ".max_curvature", rclcpp::ParameterValue(2.0));
  node->get_parameter(nm + ".max_curvature", max_curvature_);
}

double SmoothnessCritic::score(const CriticData & d) const
{
  double total = 0.0;
  const double inv_n = 1.0 / curvature_samples_;
  for (int i = 0; i < curvature_samples_; ++i) {
    const double t = std::clamp(
      static_cast<double>(i) * inv_n, 0.0, 1.0 - 1e-9);
    const double kappa = d.trajectory.curvature(t);
    if (kappa > max_curvature_) {
      total += weight_ * (kappa - max_curvature_) / max_curvature_;
    }
  }
  return total * inv_n;
}
}  // namespace vf_robot_controller::critics
