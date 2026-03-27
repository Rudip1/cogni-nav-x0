#include "vf_robot_controller/critics/goal_critic.hpp"
#include "vf_robot_controller/tools/utils.hpp"
#include "nav2_util/node_utils.hpp"
#include <cmath>

namespace vf_robot_controller::critics
{
void GoalCritic::initialize(
  rclcpp_lifecycle::LifecycleNode::SharedPtr node,
  const std::string & nm, const Parameters & /*p*/)
{
  name_ = nm; weight_ = 1.5;
  nav2_util::declare_parameter_if_not_declared(
    node, nm + ".goal_activation_dist", rclcpp::ParameterValue(1.5));
  node->get_parameter(nm + ".goal_activation_dist", goal_activation_dist_);
}

double GoalCritic::score(const CriticData & d) const
{
  if (!d.goal) return 0.0;

  const Eigen::Vector2d end = d.trajectory.evaluate(1.0 - 1e-9);
  const double gx = d.goal->pose.position.x;
  const double gy = d.goal->pose.position.y;

  const double dist = std::hypot(end.x() - gx, end.y() - gy);
  if (dist > goal_activation_dist_) return 0.0;  // not yet near goal

  // Terminal cost: distance + heading error
  const Eigen::Vector2d vel_end = d.trajectory.derivative(1.0 - 1e-9);
  const double traj_yaw = std::atan2(vel_end.y(), vel_end.x());
  const auto & q = d.goal->pose.orientation;
  const double goal_yaw = std::atan2(
    2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z));
  const double head_err = std::abs(
    tools::normalizeAngle(traj_yaw - goal_yaw));

  return weight_ * (dist + heading_weight_ * head_err);
}
}  // namespace vf_robot_controller::critics
