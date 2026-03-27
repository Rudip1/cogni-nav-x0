#pragma once

#include <vector>
#include "vf_robot_controller/gcf/geometric_complexity_field.hpp"
#include "vf_robot_controller/models/trajectory.hpp"
#include "vf_robot_controller/parameter_handler.hpp"

namespace vf_robot_controller::tools
{

/**
 * @brief VelocityScheduler — predictive speed profile from GCF.
 *
 * Unlike all existing Nav2 controllers (which reduce speed only when
 * already close to an obstacle), VelocityScheduler reduces speed
 * BEFORE the robot reaches a complex region.
 *
 * Algorithm:
 *   1. Sample GCF complexity along the upcoming trajectory
 *   2. Build a v_max(t) profile: v_max decreases smoothly ahead of
 *      high-C regions, accounting for deceleration distance
 *   3. Return the v_max for t=0 (current timestep)
 *
 * The deceleration preview distance is: d_preview = v_current² / (2*a_max)
 * So the robot starts slowing down exactly early enough to reach the
 * required speed at the entrance of the tight region.
 *
 * This is a genuinely novel behaviour — no existing Nav2 controller
 * does predictive velocity adaptation based on upcoming geometry.
 */
class VelocityScheduler
{
public:
  explicit VelocityScheduler(const Parameters & params);

  /**
   * @brief Compute maximum allowed velocity given current GCF state.
   * @param gcf             geometric complexity field
   * @param trajectory      current nominal trajectory
   * @param current_vel_x   current robot speed (m/s)
   * @return v_max for the current control step
   */
  double computeMaxVelocity(
    const std::shared_ptr<gcf::GeometricComplexityField> & gcf,
    const models::BSplineTrajectory & trajectory,
    double current_vel_x) const;

  /**
   * @brief Full v_max(t) profile along trajectory (for visualization).
   */
  std::vector<double> computeVelocityProfile(
    const std::shared_ptr<gcf::GeometricComplexityField> & gcf,
    const models::BSplineTrajectory & trajectory) const;

private:
  double complexityToVmax(double complexity) const;

  const Parameters & params_;
  int profile_samples_{20};  // points sampled along trajectory
};

}  // namespace vf_robot_controller::tools
