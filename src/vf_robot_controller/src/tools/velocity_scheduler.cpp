#include "vf_robot_controller/tools/velocity_scheduler.hpp"
#include <cmath>
#include <algorithm>

namespace vf_robot_controller::tools
{

VelocityScheduler::VelocityScheduler(const Parameters & params)
: params_(params)
{}

double VelocityScheduler::computeMaxVelocity(
  const std::shared_ptr<gcf::GeometricComplexityField> & gcf,
  const models::BSplineTrajectory & trajectory,
  double current_vel_x) const
{
  // Build velocity profile and return the minimum considering decel preview
  const auto profile = computeVelocityProfile(gcf, trajectory);
  if (profile.empty()) return params_.max_vel_x;

  // Deceleration preview: find how far ahead we need to start slowing
  // d_preview = v² / (2 * a_max)
  const double a_max = params_.acc_lim_x;
  const double d_preview = (current_vel_x * current_vel_x) / (2.0 * a_max);

  // Approximate arc length of trajectory
  const double arc_len = trajectory.arcLength(profile_samples_);
  const double step_len = arc_len / std::max(profile_samples_ - 1, 1);

  // Find the minimum v_max within deceleration preview distance
  const int preview_steps = static_cast<int>(std::ceil(d_preview / std::max(step_len, 0.01)));
  const int look_ahead = std::min(preview_steps, static_cast<int>(profile.size()) - 1);

  double v_required = params_.max_vel_x;
  for (int i = 0; i <= look_ahead; ++i) {
    v_required = std::min(v_required, profile[i]);
  }

  return v_required;
}

std::vector<double> VelocityScheduler::computeVelocityProfile(
  const std::shared_ptr<gcf::GeometricComplexityField> & gcf,
  const models::BSplineTrajectory & trajectory) const
{
  std::vector<double> profile(profile_samples_);

  for (int i = 0; i < profile_samples_; ++i) {
    const double t = static_cast<double>(i) / (profile_samples_ - 1);
    const Eigen::Vector2d pt = trajectory.evaluate(
      std::clamp(t, 0.0, 1.0 - 1e-9));
    const auto cell = gcf->query(pt.x(), pt.y());
    profile[i] = complexityToVmax(cell.complexity);
  }
  return profile;
}

double VelocityScheduler::complexityToVmax(double complexity) const
{
  // Linear interpolation: 0 complexity → max_vel_x, 1 complexity → max_speed_in_tight
  const double v_open  = params_.max_vel_x;
  const double v_tight = params_.max_speed_in_tight;
  const double c       = std::clamp(complexity, 0.0, 1.0);

  // Use a soft threshold around tight_gcf_threshold
  double scale = 1.0;
  if (c >= params_.tight_gcf_threshold) {
    // Full tight speed
    scale = v_tight / v_open;
  } else {
    // Smooth transition
    const double t = c / params_.tight_gcf_threshold;
    scale = 1.0 - (1.0 - v_tight / v_open) * (t * t);
  }
  return std::max(params_.min_vel_x, v_open * scale);
}

}  // namespace vf_robot_controller::tools
