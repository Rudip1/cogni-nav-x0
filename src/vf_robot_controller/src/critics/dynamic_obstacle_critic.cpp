#include "vf_robot_controller/critics/dynamic_obstacle_critic.hpp"
#include "vf_robot_controller/tools/utils.hpp"
#include "nav2_util/node_utils.hpp"
#include <cmath>

namespace vf_robot_controller::critics
{
void DynamicObstacleCritic::initialize(
  rclcpp_lifecycle::LifecycleNode::SharedPtr node,
  const std::string & nm, const Parameters & p)
{
  name_   = nm;
  weight_ = 3.0;
  (void)p;
  nav2_util::declare_parameter_if_not_declared(
    node, nm + ".collision_radius", rclcpp::ParameterValue(0.6));
  nav2_util::declare_parameter_if_not_declared(
    node, nm + ".prediction_horizon", rclcpp::ParameterValue(2.0));
  node->get_parameter(nm + ".collision_radius",    collision_radius_);
  node->get_parameter(nm + ".prediction_horizon",  prediction_horizon_);
}

double DynamicObstacleCritic::score(const CriticData & d) const
{
  // Dynamic obstacles come from the costmap's tracking layer.
  // We approximate by checking GCF clutter evolution:
  // high clutter + moving obstacle → penalise.
  // For a full implementation, integrate with nav2_dynamic_obstacle_tracker.
  if (!d.gcf) return 0.0;

  const auto pts = d.trajectory.sample(15);
  double total   = 0.0;

  for (size_t i = 0; i < pts.size(); ++i) {
    // Time at this trajectory point (approx uniform)
    const double t = (static_cast<double>(i) / (pts.size() - 1)) * prediction_horizon_;

    const auto cell = d.gcf->query(pts[i].x(), pts[i].y());

    // High clutter + time-decay weighting
    if (cell.clutter_density > 0.3) {
      const double time_factor = std::exp(-time_decay_ * t);
      total += weight_ * cell.clutter_density * time_factor;
    }
  }
  return total / static_cast<double>(pts.size());
}
}  // namespace vf_robot_controller::critics
