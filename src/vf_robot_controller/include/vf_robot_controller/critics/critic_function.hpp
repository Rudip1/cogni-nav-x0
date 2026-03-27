#pragma once

#include <string>
#include <memory>
#include <Eigen/Dense>

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "vf_robot_controller/models/trajectory.hpp"
#include "vf_robot_controller/models/state.hpp"
#include "vf_robot_controller/gcf/geometric_complexity_field.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "vf_robot_controller/parameter_handler.hpp"

namespace vf_robot_controller::critics
{

/**
 * @brief CriticData — everything a critic needs to score one trajectory.
 */
struct CriticData
{
  const models::BSplineTrajectory & trajectory;
  const models::StateSequence     & rollout;       // robot states for this trajectory
  const nav2_costmap_2d::Costmap2D * costmap;
  const std::shared_ptr<gcf::GeometricComplexityField> gcf;
  const nav_msgs::msg::Path * global_plan;         // look-ahead portion
  const geometry_msgs::msg::PoseStamped * goal;
  const sensor_msgs::msg::PointCloud2::SharedPtr pointcloud;  // may be nullptr

  // GCF-driven weight scaling (computed once, reused by all critics)
  double gcf_mean_complexity{0.0};
  double obstacle_weight_scale{1.0};
  double path_follow_weight_scale{1.0};
};

/**
 * @brief CriticFunction — base class for all scoring critics.
 *
 * Each critic computes one cost term for a candidate trajectory.
 * Critics are pluginlib-loadable so you can add new ones without
 * recompiling the core controller.
 *
 * score() is called N times per control cycle (once per trajectory sample).
 * It must be fast. Do not allocate memory inside score().
 */
class CriticFunction
{
public:
  virtual ~CriticFunction() = default;

  /**
   * @brief Initialize critic. Called once at controller configure().
   * Declare and load critic-specific parameters here.
   */
  virtual void initialize(
    rclcpp_lifecycle::LifecycleNode::SharedPtr node,
    const std::string & critic_name,
    const Parameters & params) = 0;

  /**
   * @brief Score one trajectory. Returns cost in [0, +inf).
   * Lower is better. Returns 0 for fully satisfied.
   * Return std::numeric_limits<double>::infinity() for hard infeasibility.
   */
  virtual double score(const CriticData & data) const = 0;

  /**
   * @brief Base weight for this critic. Multiplied by GCF-driven scale.
   */
  virtual double weight() const { return weight_; }

  std::string name() const { return name_; }

protected:
  std::string name_;
  double weight_{1.0};
};

}  // namespace vf_robot_controller::critics
