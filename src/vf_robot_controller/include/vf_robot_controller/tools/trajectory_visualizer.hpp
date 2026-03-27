#pragma once

#include <memory>
#include <vector>
#include <string>

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "nav_msgs/msg/path.hpp"
#include "vf_robot_controller/models/trajectory.hpp"
#include "vf_robot_controller/gcf/geometric_complexity_field.hpp"
#include "vf_robot_controller/parameter_handler.hpp"

namespace vf_robot_controller::tools
{

/**
 * @brief TrajectoryVisualizer — publishes MarkerArrays for RViz.
 *
 * Publishes:
 *   /vf_controller/sampled_trajectories  — all N sampled trajectories
 *                                          coloured by cost (green=good, red=bad)
 *   /vf_controller/best_trajectory       — the selected trajectory (thick blue)
 *   /vf_controller/gcf_field             — GCF complexity as a heatmap grid
 *   /vf_controller/velocity_profile      — v_max profile as a line strip
 *   /vf_controller/safety_shell          — swept volume footprint (when vetoing)
 */
class TrajectoryVisualizer
{
public:
  TrajectoryVisualizer(
    rclcpp_lifecycle::LifecycleNode::SharedPtr node,
    const Parameters & params);

  void activate();
  void deactivate();

  void publishSampledTrajectories(
    const std::vector<models::BSplineTrajectory> & trajectories,
    const std::vector<double> & costs,
    const std::string & frame_id);

  void publishBestTrajectory(
    const models::BSplineTrajectory & best,
    const std::string & frame_id);

  void publishGCFField(
    const std::shared_ptr<gcf::GeometricComplexityField> & gcf,
    const std::string & frame_id);

  void publishVelocityProfile(
    const std::vector<double> & v_profile,
    const models::BSplineTrajectory & traj,
    const std::string & frame_id);

  void publishSafetyShellVeto(
    double robot_x, double robot_y, double robot_yaw,
    const std::string & frame_id);

private:
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  const Parameters & params_;

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
    sampled_pub_, best_pub_, gcf_pub_, vel_pub_, safety_pub_;
};

}  // namespace vf_robot_controller::tools
