#include "vf_robot_controller/tools/noise_generator.hpp"
#include <cmath>

namespace vf_robot_controller::tools
{

NoiseGenerator::NoiseGenerator(const Parameters & params, unsigned int seed)
: params_(params), rng_(seed)
{}

std::vector<models::BSplineTrajectory> NoiseGenerator::generateSamples(
  const models::BSplineTrajectory & base,
  int n_samples,
  double gcf_mean) const
{
  std::vector<models::BSplineTrajectory> samples;
  samples.reserve(n_samples);

  // Include the base trajectory as sample 0 (zero noise)
  samples.push_back(base);

  for (int s = 1; s < n_samples; ++s) {
    const auto noise = singlePerturbation(base.numControlPoints(), gcf_mean);
    samples.push_back(base.perturbed(noise));
  }
  return samples;
}

std::vector<Eigen::Vector2d> NoiseGenerator::singlePerturbation(
  int n_ctrl_pts, double gcf_mean) const
{
  // In tight regions (high GCF), use smaller perturbations to stay feasible.
  // In open regions (low GCF), use larger perturbations to explore more.
  const double open_sigma = std::max(params_.noise_sigma_x, 1e-4);
  const double sigma = open_sigma * (1.0 - 0.7 * std::clamp(gcf_mean, 0.0, 1.0));

  std::vector<Eigen::Vector2d> noise(n_ctrl_pts);
  for (auto & n : noise) {
    n.x() = dist_(rng_) * sigma;
    n.y() = dist_(rng_) * sigma;
  }

  // Fix first control point — robot cannot teleport from current position
  if (!noise.empty()) noise[0] = Eigen::Vector2d::Zero();

  return noise;
}

}  // namespace vf_robot_controller::tools
