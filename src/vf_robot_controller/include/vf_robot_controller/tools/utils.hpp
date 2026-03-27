#pragma once

#include <vector>
#include <cmath>
#include <Eigen/Dense>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"

namespace vf_robot_controller::tools
{

// ── Angle utilities ──────────────────────────────────────────────────────────
inline double normalizeAngle(double a)
{
  while (a >  M_PI) a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}

inline double poseYaw(const geometry_msgs::msg::PoseStamped & p)
{
  const auto & q = p.pose.orientation;
  return std::atan2(2.0*(q.w*q.z + q.x*q.y),
                    1.0 - 2.0*(q.y*q.y + q.z*q.z));
}

// ── Path utilities ───────────────────────────────────────────────────────────

/**
 * @brief Extract N waypoints ahead of robot from global plan.
 * Finds the closest point on plan, then samples N points at equal arc-length.
 */
std::vector<Eigen::Vector2d> extractLookaheadWaypoints(
  const nav_msgs::msg::Path & plan,
  double robot_x, double robot_y,
  double lookahead_dist,
  int n_points);

/**
 * @brief Compute heading angles between consecutive waypoints.
 */
std::vector<double> computePathHeadings(
  const std::vector<Eigen::Vector2d> & waypoints);

/**
 * @brief Find index of closest path point to (x,y).
 */
size_t closestPathIndex(
  const nav_msgs::msg::Path & plan,
  double x, double y);

/**
 * @brief Arc length of path from index start to end.
 */
double pathArcLength(
  const nav_msgs::msg::Path & plan,
  size_t start, size_t end);

// ── Robot kinematics ─────────────────────────────────────────────────────────

/**
 * @brief Forward-simulate differential drive one step.
 * @param x,y,yaw  current pose
 * @param v,omega  command
 * @param dt       timestep
 */
void diffDriveStep(
  double & x, double & y, double & yaw,
  double v, double omega, double dt);

/**
 * @brief Compute (v, omega) command to follow B-spline at given parameter t.
 * Returns feasible (v,omega) clamped to kinematic limits.
 */
std::pair<double,double> splineToCmd(
  const Eigen::Vector2d & pos,
  const Eigen::Vector2d & vel_vec,
  double current_yaw,
  double max_v, double max_omega);

// ── MPPI math ────────────────────────────────────────────────────────────────

/**
 * @brief Compute MPPI importance weights from cost vector.
 * w_i = exp(-(cost_i - min_cost) / temperature) / sum(w)
 */
std::vector<double> mppiWeights(
  const std::vector<double> & costs,
  double temperature);

/**
 * @brief Compute weighted mean of control point perturbations.
 * Returns delta control points to apply to base trajectory.
 */
std::vector<Eigen::Vector2d> mppiWeightedMean(
  const std::vector<std::vector<Eigen::Vector2d>> & perturbations,
  const std::vector<double> & weights);

}  // namespace vf_robot_controller::tools
