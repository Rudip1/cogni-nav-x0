#include "vf_robot_controller/tools/utils.hpp"
#include <cmath>
#include <numeric>
#include <stdexcept>

namespace vf_robot_controller::tools
{

std::vector<Eigen::Vector2d> extractLookaheadWaypoints(
  const nav_msgs::msg::Path & plan,
  double robot_x, double robot_y,
  double lookahead_dist,
  int n_points)
{
  if (plan.poses.empty()) return {};

  // Find closest point on plan
  size_t closest_idx = 0;
  double min_dist = 1e9;
  for (size_t i = 0; i < plan.poses.size(); ++i) {
    const double dx = plan.poses[i].pose.position.x - robot_x;
    const double dy = plan.poses[i].pose.position.y - robot_y;
    const double d  = std::sqrt(dx*dx + dy*dy);
    if (d < min_dist) { min_dist = d; closest_idx = i; }
  }

  // Collect points up to lookahead_dist ahead
  std::vector<Eigen::Vector2d> pts;
  double accumulated = 0.0;
  Eigen::Vector2d prev = {plan.poses[closest_idx].pose.position.x,
                           plan.poses[closest_idx].pose.position.y};
  pts.push_back(prev);

  for (size_t i = closest_idx + 1; i < plan.poses.size(); ++i) {
    Eigen::Vector2d cur = {plan.poses[i].pose.position.x,
                            plan.poses[i].pose.position.y};
    accumulated += (cur - prev).norm();
    pts.push_back(cur);
    prev = cur;
    if (accumulated >= lookahead_dist) break;
  }

  // Resample to exactly n_points
  if (static_cast<int>(pts.size()) == n_points) return pts;

  std::vector<Eigen::Vector2d> resampled;
  resampled.reserve(n_points);
  const double total = std::max(accumulated, 1e-6);
  for (int i = 0; i < n_points; ++i) {
    const double s   = (static_cast<double>(i) / (n_points - 1)) * total;
    double cum = 0.0;
    for (size_t j = 1; j < pts.size(); ++j) {
      double seg = (pts[j] - pts[j-1]).norm();
      if (cum + seg >= s || j == pts.size() - 1) {
        const double t = (seg > 1e-9) ? std::clamp((s - cum) / seg, 0.0, 1.0) : 0.0;
        resampled.push_back((1.0 - t) * pts[j-1] + t * pts[j]);
        break;
      }
      cum += seg;
    }
  }
  if (resampled.empty() && !pts.empty()) resampled = pts;
  return resampled;
}

std::vector<double> computePathHeadings(
  const std::vector<Eigen::Vector2d> & waypoints)
{
  std::vector<double> headings;
  if (waypoints.size() < 2) return headings;
  headings.reserve(waypoints.size());
  for (size_t i = 0; i + 1 < waypoints.size(); ++i) {
    const Eigen::Vector2d d = waypoints[i+1] - waypoints[i];
    headings.push_back(std::atan2(d.y(), d.x()));
  }
  headings.push_back(headings.back());  // repeat last for same size
  return headings;
}

size_t closestPathIndex(
  const nav_msgs::msg::Path & plan, double x, double y)
{
  size_t idx = 0;
  double best = 1e9;
  for (size_t i = 0; i < plan.poses.size(); ++i) {
    const double dx = plan.poses[i].pose.position.x - x;
    const double dy = plan.poses[i].pose.position.y - y;
    const double d  = dx*dx + dy*dy;
    if (d < best) { best = d; idx = i; }
  }
  return idx;
}

double pathArcLength(
  const nav_msgs::msg::Path & plan, size_t start, size_t end)
{
  double len = 0.0;
  for (size_t i = start + 1; i <= std::min(end, plan.poses.size()-1); ++i) {
    const double dx = plan.poses[i].pose.position.x - plan.poses[i-1].pose.position.x;
    const double dy = plan.poses[i].pose.position.y - plan.poses[i-1].pose.position.y;
    len += std::sqrt(dx*dx + dy*dy);
  }
  return len;
}

void diffDriveStep(
  double & x, double & y, double & yaw,
  double v, double omega, double dt)
{
  if (std::abs(omega) < 1e-6) {
    x   += v * std::cos(yaw) * dt;
    y   += v * std::sin(yaw) * dt;
  } else {
    const double r = v / omega;
    x   += r * (std::sin(yaw + omega * dt) - std::sin(yaw));
    y   += r * (std::cos(yaw) - std::cos(yaw + omega * dt));
    yaw += omega * dt;
  }
  yaw = normalizeAngle(yaw);
}

std::pair<double,double> splineToCmd(
  const Eigen::Vector2d & pos,
  const Eigen::Vector2d & vel_vec,
  double current_yaw,
  double max_v, double max_omega)
{
  const double speed = vel_vec.norm();
  if (speed < 1e-6) return {0.0, 0.0};

  const double target_yaw = std::atan2(vel_vec.y(), vel_vec.x());
  const double yaw_err    = normalizeAngle(target_yaw - current_yaw);

  const double v     = std::clamp(speed, 0.0, max_v);
  const double omega = std::clamp(2.0 * yaw_err, -max_omega, max_omega);
  return {v, omega};
}

std::vector<double> mppiWeights(
  const std::vector<double> & costs, double temperature)
{
  if (costs.empty()) return {};
  const double min_cost = *std::min_element(costs.begin(), costs.end());
  std::vector<double> w(costs.size());
  double sum = 0.0;
  for (size_t i = 0; i < costs.size(); ++i) {
    w[i] = std::exp(-(costs[i] - min_cost) / std::max(temperature, 1e-6));
    sum += w[i];
  }
  if (sum < 1e-10) {
    std::fill(w.begin(), w.end(), 1.0 / costs.size());
  } else {
    for (auto & wi : w) wi /= sum;
  }
  return w;
}

std::vector<Eigen::Vector2d> mppiWeightedMean(
  const std::vector<std::vector<Eigen::Vector2d>> & perturbations,
  const std::vector<double> & weights)
{
  if (perturbations.empty()) return {};
  const size_t n_ctrl = perturbations[0].size();
  std::vector<Eigen::Vector2d> mean(n_ctrl, Eigen::Vector2d::Zero());

  for (size_t i = 0; i < perturbations.size(); ++i) {
    const double w = (i < weights.size()) ? weights[i] : 0.0;
    for (size_t j = 0; j < std::min(perturbations[i].size(), n_ctrl); ++j) {
      mean[j] += w * perturbations[i][j];
    }
  }
  return mean;
}

}  // namespace vf_robot_controller::tools
