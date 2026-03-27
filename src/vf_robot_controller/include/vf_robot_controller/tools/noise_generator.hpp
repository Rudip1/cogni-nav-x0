#pragma once

#include <vector>
#include <random>
#include <Eigen/Dense>
#include "vf_robot_controller/models/trajectory.hpp"
#include "vf_robot_controller/models/state.hpp"
#include "vf_robot_controller/parameter_handler.hpp"

namespace vf_robot_controller::tools
{

/**
 * @brief NoiseGenerator — generates perturbations for B-spline control points.
 *
 * Standard MPPI perturbs (v,omega) sequences with Gaussian noise.
 * Our B-spline MPPI perturbs CONTROL POINTS with Gaussian noise.
 *
 * Each control point gets an independent (dx, dy) perturbation drawn
 * from N(0, σ²) where σ is set by the GCF: tight regions get smaller
 * perturbations (explore locally) and open regions get larger ones
 * (explore more aggressively).
 *
 * Thread-safe: uses per-thread random engines to enable parallel sampling.
 */
class NoiseGenerator
{
public:
  explicit NoiseGenerator(const Parameters & params, unsigned int seed = 42);

  /**
   * @brief Generate N perturbed copies of the base trajectory.
   * @param base       nominal B-spline trajectory
   * @param n_samples  number of perturbations
   * @param gcf_mean   mean GCF complexity (scales noise magnitude)
   * @return N perturbed trajectories
   */
  std::vector<models::BSplineTrajectory> generateSamples(
    const models::BSplineTrajectory & base,
    int n_samples,
    double gcf_mean = 0.0) const;

  /**
   * @brief Single perturbation vector for one trajectory (for testing).
   */
  std::vector<Eigen::Vector2d> singlePerturbation(
    int n_ctrl_pts, double gcf_mean = 0.0) const;

private:
  const Parameters & params_;
  mutable std::mt19937 rng_;
  mutable std::normal_distribution<double> dist_{0.0, 1.0};
};

}  // namespace vf_robot_controller::tools
