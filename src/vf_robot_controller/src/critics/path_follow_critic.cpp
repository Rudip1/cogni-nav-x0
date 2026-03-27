#include "vf_robot_controller/critics/path_follow_critic.hpp"
#include "vf_robot_controller/tools/utils.hpp"
#include "nav2_util/node_utils.hpp"
#include <cmath>
#include <limits>

namespace vf_robot_controller::critics
{
void PathFollowCritic::initialize(
  rclcpp_lifecycle::LifecycleNode::SharedPtr node,
  const std::string & nm, const Parameters & p)
{
  name_   = nm;
  weight_ = p.path_follow_weight_open;
  nav2_util::declare_parameter_if_not_declared(
    node, nm + ".max_lateral_dev", rclcpp::ParameterValue(1.0));
  nav2_util::declare_parameter_if_not_declared(
    node, nm + ".heading_weight", rclcpp::ParameterValue(0.3));
  node->get_parameter(nm + ".max_lateral_dev", max_lateral_dev_);
  node->get_parameter(nm + ".heading_weight",  heading_weight_);
}

double PathFollowCritic::score(const CriticData & d) const
{
  if (!d.global_plan || d.global_plan->poses.empty()) return 0.0;

  // GCF-driven weight: less path-following in tight/cluttered spaces
  const double w = weight_ * d.path_follow_weight_scale;

  const auto traj_pts = d.trajectory.sample(15);
  double total = 0.0;

  for (const auto & tp : traj_pts) {
    // Find closest point on global plan
    double min_dist = std::numeric_limits<double>::max();
    for (const auto & pp : d.global_plan->poses) {
      const double dx = tp.x() - pp.pose.position.x;
      const double dy = tp.y() - pp.pose.position.y;
      min_dist = std::min(min_dist, std::sqrt(dx*dx + dy*dy));
    }
    const double norm_dev = std::clamp(min_dist / max_lateral_dev_, 0.0, 1.0);
    total += w * norm_dev * norm_dev;
  }
  return total / static_cast<double>(traj_pts.size());
}
}  // namespace vf_robot_controller::critics
