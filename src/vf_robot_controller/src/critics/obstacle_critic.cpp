#include "vf_robot_controller/critics/obstacle_critic.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_util/node_utils.hpp"
#include <cmath>
#include <limits>

namespace vf_robot_controller::critics
{

void ObstacleCritic::initialize(
  rclcpp_lifecycle::LifecycleNode::SharedPtr node,
  const std::string & nm, const Parameters & p)
{
  name_   = nm;
  weight_ = p.obstacle_weight_open;
  nav2_util::declare_parameter_if_not_declared(
    node, nm + ".repulsion_radius", rclcpp::ParameterValue(0.5));
  node->get_parameter(nm + ".repulsion_radius", repulsion_radius_);
}

double ObstacleCritic::score(const CriticData & d) const
{
  const auto pts = d.trajectory.sample(20);
  double total = 0.0;

  // Apply GCF-driven weight scaling
  const double w = weight_ * d.obstacle_weight_scale;

  for (const auto & pt : pts) {
    unsigned int mx, my;
    if (!d.costmap->worldToMap(pt.x(), pt.y(), mx, my)) {
      return std::numeric_limits<double>::infinity();  // out of map
    }
    const unsigned char cost = d.costmap->getCost(mx, my);
    if (cost >= nav2_costmap_2d::LETHAL_OBSTACLE) {
      return std::numeric_limits<double>::infinity();  // collision
    }
    // Smooth cost from inscribed to zero
    const double norm = static_cast<double>(cost) / 252.0;
    total += w * norm * norm;
  }
  return total / 20.0;
}

}  // namespace vf_robot_controller::critics
