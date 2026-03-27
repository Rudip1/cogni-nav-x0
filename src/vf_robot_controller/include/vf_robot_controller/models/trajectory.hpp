#pragma once

#include <vector>
#include <Eigen/Dense>

namespace vf_robot_controller::models
{

/**
 * @brief BSplineTrajectory — cubic B-spline over the local horizon.
 *
 * Control points P_i define the spline. The knot vector is clamped uniform.
 * Variable number of control points (min_knots to max_knots) driven by GCF.
 *
 * Why B-spline:
 *   - Local support: moving P_i only affects the curve near t_i
 *   - C2 continuity: smooth velocity and acceleration
 *   - Fast evaluation: de Boor algorithm O(degree)
 *   - Gradient exists: can use gradient-based refinement if needed
 */
struct BSplineTrajectory
{
  int degree{3};                              // cubic
  std::vector<Eigen::Vector2d> control_pts;  // (x,y) control points
  std::vector<double> knots;                 // clamped uniform knot vector
  double t_start{0.0};                       // time at first control point
  double t_end{2.0};                         // horizon time

  // ── Evaluation ─────────────────────────────────────────────────────────
  /** @brief Evaluate position at parameter t in [0,1]. */
  Eigen::Vector2d evaluate(double t) const;

  /** @brief Evaluate velocity (first derivative) at t. */
  Eigen::Vector2d derivative(double t) const;

  /** @brief Evaluate curvature at t. */
  double curvature(double t) const;

  /** @brief Sample n evenly-spaced points along trajectory. */
  std::vector<Eigen::Vector2d> sample(int n) const;

  /** @brief Arc length from t=0 to t=1. */
  double arcLength(int n_samples = 50) const;

  // ── Construction helpers ────────────────────────────────────────────────
  /** @brief Build clamped uniform knot vector for given control point count. */
  static std::vector<double> buildKnots(int n_ctrl, int degree);

  /** @brief Initialize from a sequence of waypoints (least-squares fit). */
  static BSplineTrajectory fromWaypoints(
    const std::vector<Eigen::Vector2d> & pts,
    int n_ctrl, int degree = 3);

  /** @brief Perturb control points with given noise vector. */
  BSplineTrajectory perturbed(const std::vector<Eigen::Vector2d> & noise) const;

  int numControlPoints() const { return static_cast<int>(control_pts.size()); }
};

}  // namespace vf_robot_controller::models
