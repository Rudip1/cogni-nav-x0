#include "vf_robot_controller/optimizer.hpp"
#include "vf_robot_controller/tools/utils.hpp"
#include "vf_robot_controller/models/state.hpp"
#include <cmath>
#include <algorithm>

namespace vf_robot_controller
{

void Optimizer::initialize(
  rclcpp_lifecycle::LifecycleNode::SharedPtr node,
  const Parameters & params,
  std::shared_ptr<gcf::GeometricComplexityField> gcf)
{
  params_ = &params;
  gcf_    = gcf;

  critic_manager_ = std::make_unique<CriticManager>();
  critic_manager_->initialize(node, params);

  noise_gen_    = std::make_unique<tools::NoiseGenerator>(params);
  vel_scheduler_ = std::make_unique<tools::VelocityScheduler>(params);

  RCLCPP_INFO(logger_, "Optimizer initialized: %d samples, %d iterations, %.1fs horizon",
    params.num_samples, params.num_iterations, params.horizon_time);
}

void Optimizer::activate()  {}
void Optimizer::deactivate(){}

geometry_msgs::msg::Twist Optimizer::optimize(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & current_vel,
  const nav_msgs::msg::Path & global_plan,
  const nav2_costmap_2d::Costmap2D * costmap,
  const sensor_msgs::msg::PointCloud2::SharedPtr & pointcloud)
{
  if (global_plan.poses.empty()) {
    RCLCPP_WARN(logger_, "Empty global plan — stopping");
    return geometry_msgs::msg::Twist{};
  }

  const double rx = pose.pose.position.x;
  const double ry = pose.pose.position.y;

  // ── Step 1: Extract look-ahead waypoints ────────────────────────────────
  const double lookahead = params_->max_vel_x * params_->horizon_time;
  const int    n_wp      = 12;
  const auto   waypoints = tools::extractLookaheadWaypoints(
    global_plan, rx, ry, lookahead, n_wp);

  if (waypoints.size() < 2) {
    RCLCPP_WARN(logger_, "Too few waypoints in look-ahead");
    return geometry_msgs::msg::Twist{};
  }

  // ── Step 2: Per-segment complexity from GCF ──────────────────────────────
  const auto complexity_per_seg = gcf_->complexityPerSegment(waypoints);
  const double gcf_mean         = gcf_->meanComplexityAlongPath(waypoints);

  // ── Step 3: DOF allocation (SegmentMap) ─────────────────────────────────
  const auto seg_map = models::SegmentMap::build(
    complexity_per_seg,
    2,                        // min knots per segment
    params_->max_knots,       // max total knots
    params_->tight_gcf_threshold);

  // ── Step 4: Base B-spline trajectory ────────────────────────────────────
  const int n_ctrl = std::clamp(seg_map.total_knots, params_->min_knots, params_->max_knots);
  best_trajectory_ = models::BSplineTrajectory::fromWaypoints(waypoints, n_ctrl, params_->spline_degree);

  // ── Step 5: Velocity limit from scheduler ───────────────────────────────
  const double v_max = vel_scheduler_->computeMaxVelocity(
    gcf_, best_trajectory_, current_vel.linear.x);

  // ── Find goal pose for critics ───────────────────────────────────────────
  geometry_msgs::msg::PoseStamped goal_pose;
  if (!global_plan.poses.empty()) goal_pose = global_plan.poses.back();

  // ── MPPI iteration loop ──────────────────────────────────────────────────
  for (int iter = 0; iter < params_->num_iterations; ++iter) {

    // Step 6: Generate N perturbed samples
    sampled_trajectories_ = noise_gen_->generateSamples(
      best_trajectory_, params_->num_samples, gcf_mean);

    // Step 7: Roll out all samples
    std::vector<models::StateSequence> rollouts(sampled_trajectories_.size());
    for (size_t s = 0; s < sampled_trajectories_.size(); ++s) {
      rollouts[s] = rollout(sampled_trajectories_[s], pose, current_vel);
    }

    // Step 8: Score
    sample_costs_ = critic_manager_->scoreAll(
      sampled_trajectories_, rollouts,
      costmap, gcf_, &global_plan, &goal_pose, pointcloud);

    // Step 9: MPPI weights
    const auto weights = tools::mppiWeights(sample_costs_, params_->temperature);

    // Step 10: Weighted mean perturbation update
    // Collect perturbations (delta from base)
    std::vector<std::vector<Eigen::Vector2d>> perturbations(sampled_trajectories_.size());
    for (size_t s = 0; s < sampled_trajectories_.size(); ++s) {
      const int nc = std::min(
        static_cast<int>(sampled_trajectories_[s].control_pts.size()),
        static_cast<int>(best_trajectory_.control_pts.size()));
      perturbations[s].resize(nc);
      for (int k = 0; k < nc; ++k) {
        perturbations[s][k] =
          sampled_trajectories_[s].control_pts[k] - best_trajectory_.control_pts[k];
      }
    }

    const auto delta = tools::mppiWeightedMean(perturbations, weights);

    // Apply delta to base trajectory (skip first control point — robot position)
    for (size_t k = 1; k < best_trajectory_.control_pts.size() &&
                        k < delta.size(); ++k) {
      best_trajectory_.control_pts[k] += delta[k];
    }
  }

  // ── Step 12: Convert to velocity command ─────────────────────────────────
  return trajectoryToCmd(best_trajectory_, pose, v_max);
}

models::StateSequence Optimizer::rollout(
  const models::BSplineTrajectory & traj,
  const geometry_msgs::msg::PoseStamped & start_pose,
  const geometry_msgs::msg::Twist & start_vel) const
{
  const int steps = 15;
  models::StateSequence seq(steps);

  const auto & q0 = start_pose.pose.orientation;
  double x   = start_pose.pose.position.x;
  double y   = start_pose.pose.position.y;
  double yaw = std::atan2(2.0*(q0.w*q0.z + q0.x*q0.y),
                          1.0 - 2.0*(q0.y*q0.y + q0.z*q0.z));
  const double dt = params_->horizon_time / steps;

  for (int i = 0; i < steps; ++i) {
    const double t = std::clamp(
      static_cast<double>(i) / (steps - 1), 0.0, 1.0 - 1e-9);
    const Eigen::Vector2d pt  = traj.evaluate(t);
    const Eigen::Vector2d vel = traj.derivative(t);

    auto [v, omega] = tools::splineToCmd(
      pt, vel, yaw, params_->max_vel_x, params_->max_vel_theta);

    seq[i] = {x, y, yaw, v, omega, t * params_->horizon_time};
    tools::diffDriveStep(x, y, yaw, v, omega, dt);
  }
  return seq;
}

geometry_msgs::msg::Twist Optimizer::trajectoryToCmd(
  const models::BSplineTrajectory & traj,
  const geometry_msgs::msg::PoseStamped & pose,
  double v_max) const
{
  const auto & q = pose.pose.orientation;
  const double yaw = std::atan2(
    2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z));

  // Evaluate first derivative of trajectory at t=0 to get desired velocity direction
  const Eigen::Vector2d vel_vec = traj.derivative(0.01);

  auto [v, omega] = tools::splineToCmd(
    traj.evaluate(0.0), vel_vec, yaw, v_max, params_->max_vel_theta);

  // Apply acceleration limits
  geometry_msgs::msg::Twist cmd;
  cmd.linear.x  = std::clamp(v,     params_->min_vel_x, v_max);
  cmd.angular.z = std::clamp(omega, -params_->max_vel_theta, params_->max_vel_theta);
  return cmd;
}

}  // namespace vf_robot_controller
