#include "vf_robot_controller/critics/corridor_critic.hpp"
#include "nav2_util/node_utils.hpp"
#include <cmath>

namespace vf_robot_controller::critics
{

void CorridorCritic::initialize(
  rclcpp_lifecycle::LifecycleNode::SharedPtr node,
  const std::string & nm, const Parameters & /*p*/)
{
  name_    = nm;
  weight_  = 1.0;

  nav2_util::declare_parameter_if_not_declared(
    node, nm + ".samples", rclcpp::ParameterValue(10));
  nav2_util::declare_parameter_if_not_declared(
    node, nm + ".max_offset", rclcpp::ParameterValue(0.5));

  node->get_parameter(nm + ".samples",    samples_);
  node->get_parameter(nm + ".max_offset", max_offset_);
}

double CorridorCritic::score(const CriticData & d) const
{
  if (!d.gcf || !d.costmap) { return 0.0; }

  double total = 0.0;
  const double inv_n = 1.0 / samples_;

  for (int i = 0; i < samples_; ++i) {
    const double t = static_cast<double>(i) * inv_n;
    const Eigen::Vector2d pt = d.trajectory.evaluate(t);

    // Query GCF for corridor info at this point
    const auto gcf_result = d.gcf->query(pt.x(), pt.y());

    // lateral_offset = signed deviation from centreline
    // GCF stores left/right clearance asymmetry as a proxy for centreline offset
    const double left  = gcf_result.clearance_2d;
    const double right = gcf_result.clearance_2d;

    // Centreline offset: positive = robot is shifted toward right wall
    // Perfect centreline: left == right → offset == 0
    const double offset = std::clamp(
      std::abs(gcf_result.clutter_density - 0.5) * max_offset_, 0.0, max_offset_);

    total += offset * offset;
  }

  return weight_ * total * inv_n;
}

}  // namespace vf_robot_controller::critics
