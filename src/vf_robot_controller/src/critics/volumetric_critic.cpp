#include "vf_robot_controller/critics/volumetric_critic.hpp"
#include "nav2_util/node_utils.hpp"
#include <limits>

namespace vf_robot_controller::critics
{
void VolumetricCritic::initialize(
  rclcpp_lifecycle::LifecycleNode::SharedPtr node,
  const std::string & nm, const Parameters & p)
{
  name_   = nm;
  weight_ = 2.0;
  nav2_util::declare_parameter_if_not_declared(
    node, nm + ".min_clearance", rclcpp::ParameterValue(0.08));
  node->get_parameter(nm + ".min_clearance", min_clearance_);
}

double VolumetricCritic::score(const CriticData & d) const
{
  if (!d.gcf) return 0.0;
  const auto pts = d.trajectory.sample(15);
  double total = 0.0;

  for (const auto & pt : pts) {
    const auto cell = d.gcf->query(pt.x(), pt.y());
    // Penalize low 3D clearance — chair legs, bed frames etc.
    const double c3d = cell.clearance_3d;
    if (c3d < min_clearance_) {
      return std::numeric_limits<double>::infinity();
    }
    // Soft penalty as clearance approaches threshold
    if (c3d < min_clearance_ * 3.0) {
      const double t = (c3d - min_clearance_) / (min_clearance_ * 2.0);
      total += weight_ * (1.0 - t * t);
    }
  }
  return total / 15.0;
}
}  // namespace vf_robot_controller::critics
