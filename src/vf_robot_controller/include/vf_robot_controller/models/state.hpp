#pragma once

#include <vector>
#include <Eigen/Dense>

namespace vf_robot_controller::models
{

/** @brief Robot state at a single timestep. */
struct State
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
  double vel_x{0.0};
  double vel_theta{0.0};
  double timestamp{0.0};  // seconds from horizon start
};

/** @brief Sequence of states (one rollout). */
using StateSequence = std::vector<State>;

/**
 * @brief ControlSequence — sampled (v, ω) commands for one trajectory.
 *
 * In standard MPPI this IS the trajectory. In our B-spline MPPI this is
 * derived FROM the B-spline: we evaluate the spline to get (x,y) points,
 * then compute the required (v,ω) to follow them given robot kinematics.
 */
struct ControlSequence
{
  std::vector<double> v;       // linear velocity samples
  std::vector<double> omega;   // angular velocity samples
  double dt{0.05};             // seconds per step

  int size() const { return static_cast<int>(v.size()); }

  void resize(int n) { v.resize(n, 0.0); omega.resize(n, 0.0); }
};

/**
 * @brief SegmentMap — per-segment complexity and knot allocation.
 *
 * Produced by DOFAllocator using GCF output.
 * Consumed by Optimizer to build the BSplineTrajectory with
 * variable-density control points.
 */
struct SegmentMap
{
  struct Segment
  {
    double t_start{0.0};         // parameter start in [0,1]
    double t_end{1.0};           // parameter end
    double complexity{0.0};      // GCF value for this segment
    int    num_knots{2};         // control points allocated to this segment
    bool   is_tight{false};      // complexity > tight_threshold
  };

  std::vector<Segment> segments;
  int total_knots{6};            // sum of all segment knots

  /** @brief Build segment map from per-waypoint complexity values. */
  static SegmentMap build(
    const std::vector<double> & complexity_per_waypoint,
    int min_knots_per_segment,
    int max_total_knots,
    double tight_threshold);
};

}  // namespace vf_robot_controller::models
