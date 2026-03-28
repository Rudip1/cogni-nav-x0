#include "vf_robot_controller/critics/clearance_critic.hpp"
#include "nav2_util/node_utils.hpp"
#include <cmath>
#include <algorithm>

namespace vf_robot_controller::critics
{

void ClearanceCritic::initialize(
  rclcpp_lifecycle::LifecycleNode::SharedPtr node,
  const std::string & nm, const Parameters & p)
{
  name_    = nm;
  weight_  = 1.5;

  nav2_util::declare_parameter_if_not_declared(
    node, nm + ".samples", rclcpp::ParameterValue(10));
  nav2_util::declare_parameter_if_not_declared(
    node, nm + ".desired_clearance", rclcpp::ParameterValue(0.4));

  node->get_parameter(nm + ".samples",           samples_);
  node->get_parameter(nm + ".desired_clearance", desired_clearance_);

  // Scale desired clearance with robot width from params
  desired_clearance_ = std::max(desired_clearance_, p.robot_width * 0.5);
}

double ClearanceCritic::score(const CriticData & d) const
{
  if (!d.gcf) { return 0.0; }

  double total = 0.0;
  const double inv_n = 1.0 / samples_;

  for (int i = 0; i < samples_; ++i) {
    const double t = static_cast<double>(i) * inv_n;
    const Eigen::Vector2d pt = d.trajectory.evaluate(t);

    const auto gcf_result = d.gcf->query(pt.x(), pt.y());

    // min clearance = minimum of left and right wall distances
    const double clearance = gcf_result.clearance_2d;

    const double deficit = desired_clearance_ - clearance;
    if (deficit > 0.0) {
      total += deficit * deficit;
    }
  }

  return weight_ * total * inv_n;
}

}  // namespace vf_robot_controller::critics
