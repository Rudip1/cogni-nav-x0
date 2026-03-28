#pragma once

#include <memory>
#include <vector>

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include "vf_robot_controller/parameter_handler.hpp"
#include "vf_robot_controller/critic_manager.hpp"
#include "vf_robot_controller/gcf/geometric_complexity_field.hpp"
#include "vf_robot_controller/models/trajectory.hpp"
#include "vf_robot_controller/models/state.hpp"
#include "vf_robot_controller/tools/noise_generator.hpp"
#include "vf_robot_controller/tools/velocity_scheduler.hpp"
#include "vf_robot_controller/tools/utils.hpp"

namespace vf_robot_controller
{

class Optimizer
{
public:
  Optimizer() = default;

  void initialize(
    rclcpp_lifecycle::LifecycleNode::SharedPtr node,
    const Parameters & params,
    std::shared_ptr<gcf::GeometricComplexityField> gcf);

  void activate();
  void deactivate();

  /**
   * @brief Push dynamic critic weights from WeightAdapter into CriticManager.
   * Called by controller every cycle before optimize().
   */
  void setDynamicWeights(const std::vector<float> & weights)
  {
    critic_manager_->setWeights(weights);
  }

  /**
   * @brief Main optimization call. Returns best (v, omega).
   */
  geometry_msgs::msg::Twist optimize(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & current_vel,
    const nav_msgs::msg::Path & global_plan,
    const nav2_costmap_2d::Costmap2D * costmap,
    const sensor_msgs::msg::PointCloud2::SharedPtr & pointcloud);

  const models::BSplineTrajectory & bestTrajectory() const { return best_trajectory_; }
  const std::vector<models::BSplineTrajectory> & sampledTrajectories() const
  { return sampled_trajectories_; }
  const std::vector<double> & sampleCosts() const { return sample_costs_; }

private:
  models::BSplineTrajectory buildBaseTrajectory(
    const std::vector<Eigen::Vector2d> & waypoints,
    const models::SegmentMap & seg_map) const;

  models::StateSequence rollout(
    const models::BSplineTrajectory & traj,
    const geometry_msgs::msg::PoseStamped & start_pose,
    const geometry_msgs::msg::Twist & start_vel) const;

  geometry_msgs::msg::Twist trajectoryToCmd(
    const models::BSplineTrajectory & traj,
    const geometry_msgs::msg::PoseStamped & pose,
    double v_max) const;

  std::unique_ptr<CriticManager>             critic_manager_;
  std::unique_ptr<tools::NoiseGenerator>     noise_gen_;
  std::unique_ptr<tools::VelocityScheduler>  vel_scheduler_;
  std::shared_ptr<gcf::GeometricComplexityField> gcf_;

  models::BSplineTrajectory              best_trajectory_;
  std::vector<models::BSplineTrajectory> sampled_trajectories_;
  std::vector<double>                    sample_costs_;

  const Parameters * params_{nullptr};
  rclcpp::Logger logger_{rclcpp::get_logger("VFOptimizer")};
};

}  // namespace vf_robot_controller
