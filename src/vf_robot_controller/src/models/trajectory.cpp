#include "vf_robot_controller/models/state.hpp"
#include "vf_robot_controller/models/trajectory.hpp"
#include <stdexcept>
#include <cmath>
#include <numeric>

namespace vf_robot_controller::models
{

// ── De Boor algorithm — evaluate B-spline at parameter t ─────────────────────
static double bsplineBasis(int i, int k, double t, const std::vector<double> & knots)
{
  if (k == 0) {
    return (t >= knots[i] && t < knots[i+1]) ? 1.0 : 0.0;
  }
  double left  = 0.0, right = 0.0;
  double denom1 = knots[i+k]   - knots[i];
  double denom2 = knots[i+k+1] - knots[i+1];
  if (denom1 > 1e-10) left  = ((t - knots[i])        / denom1) * bsplineBasis(i, k-1, t, knots);
  if (denom2 > 1e-10) right = ((knots[i+k+1] - t)    / denom2) * bsplineBasis(i+1, k-1, t, knots);
  return left + right;
}

Eigen::Vector2d BSplineTrajectory::evaluate(double t) const
{
  // Clamp t to [0, 1)
  t = std::clamp(t, 0.0, 1.0 - 1e-9);
  const int n = static_cast<int>(control_pts.size());
  Eigen::Vector2d result = Eigen::Vector2d::Zero();
  for (int i = 0; i < n; ++i) {
    const double b = bsplineBasis(i, degree, t, knots);
    result += b * control_pts[i];
  }
  return result;
}

Eigen::Vector2d BSplineTrajectory::derivative(double t) const
{
  // First derivative via finite difference (robust for all knot configs)
  const double eps = 1e-5;
  const double t1 = std::clamp(t + eps, 0.0, 1.0 - 1e-9);
  const double t0 = std::clamp(t - eps, 0.0, 1.0 - 1e-9);
  return (evaluate(t1) - evaluate(t0)) / (2.0 * eps);
}

double BSplineTrajectory::curvature(double t) const
{
  const double eps = 1e-4;
  const Eigen::Vector2d d1 = derivative(t);
  const Eigen::Vector2d d2 = (derivative(t + eps) - derivative(t - eps)) / (2.0 * eps);
  const double cross = d1.x() * d2.y() - d1.y() * d2.x();
  const double speed = d1.norm();
  if (speed < 1e-6) return 0.0;
  return std::abs(cross) / (speed * speed * speed);
}

std::vector<Eigen::Vector2d> BSplineTrajectory::sample(int n) const
{
  std::vector<Eigen::Vector2d> pts;
  pts.reserve(n);
  for (int i = 0; i < n; ++i) {
    const double t = static_cast<double>(i) / static_cast<double>(n - 1);
    pts.push_back(evaluate(std::clamp(t, 0.0, 1.0 - 1e-9)));
  }
  return pts;
}

double BSplineTrajectory::arcLength(int n_samples) const
{
  double len = 0.0;
  Eigen::Vector2d prev = evaluate(0.0);
  for (int i = 1; i <= n_samples; ++i) {
    const double t    = static_cast<double>(i) / n_samples;
    const Eigen::Vector2d cur = evaluate(std::clamp(t, 0.0, 1.0 - 1e-9));
    len += (cur - prev).norm();
    prev = cur;
  }
  return len;
}

std::vector<double> BSplineTrajectory::buildKnots(int n_ctrl, int p)
{
  // Clamped uniform knot vector: p+1 repeated ends, uniform interior
  const int m = n_ctrl + p + 1;
  std::vector<double> t(m, 0.0);
  for (int i = 0; i <= p; ++i)      t[i] = 0.0;
  for (int i = m - p - 1; i < m; ++i) t[i] = 1.0;
  const int n_interior = m - 2*(p+1);
  for (int i = 0; i < n_interior; ++i) {
    t[p + 1 + i] = static_cast<double>(i + 1) / static_cast<double>(n_interior + 1);
  }
  return t;
}

BSplineTrajectory BSplineTrajectory::fromWaypoints(
  const std::vector<Eigen::Vector2d> & pts,
  int n_ctrl, int p)
{
  if (n_ctrl < p + 1) n_ctrl = p + 1;
  if (static_cast<int>(pts.size()) < 2) {
    BSplineTrajectory t;
    t.degree = p;
    t.control_pts = pts.empty() ? std::vector<Eigen::Vector2d>(n_ctrl, Eigen::Vector2d::Zero())
                                 : std::vector<Eigen::Vector2d>(n_ctrl, pts.front());
    t.knots = buildKnots(n_ctrl, p);
    return t;
  }

  // Simple approach: distribute control points uniformly along waypoints
  BSplineTrajectory traj;
  traj.degree = p;
  traj.knots  = buildKnots(n_ctrl, p);
  traj.control_pts.resize(n_ctrl);

  for (int i = 0; i < n_ctrl; ++i) {
    const double s   = static_cast<double>(i) / static_cast<double>(n_ctrl - 1);
    const double idx = s * static_cast<double>(pts.size() - 1);
    const int    lo  = static_cast<int>(std::floor(idx));
    const int    hi  = std::min(lo + 1, static_cast<int>(pts.size()) - 1);
    const double frac = idx - lo;
    traj.control_pts[i] = (1.0 - frac) * pts[lo] + frac * pts[hi];
  }
  return traj;
}

BSplineTrajectory BSplineTrajectory::perturbed(
  const std::vector<Eigen::Vector2d> & noise) const
{
  BSplineTrajectory result = *this;
  const int n = std::min(static_cast<int>(noise.size()),
                         static_cast<int>(control_pts.size()));
  for (int i = 0; i < n; ++i) {
    result.control_pts[i] += noise[i];
  }
  return result;
}

// ── SegmentMap ────────────────────────────────────────────────────────────────
SegmentMap SegmentMap::build(
  const std::vector<double> & complexity_per_waypoint,
  int min_knots_per_segment,
  int max_total_knots,
  double tight_threshold)
{
  SegmentMap map;
  if (complexity_per_waypoint.empty()) return map;

  const int n_seg = static_cast<int>(complexity_per_waypoint.size());

  // Allocate extra knots proportionally to complexity
  double total_complexity = 0.0;
  for (double c : complexity_per_waypoint) total_complexity += c;

  map.segments.resize(n_seg);
  int allocated = 0;

  for (int i = 0; i < n_seg; ++i) {
    auto & seg   = map.segments[i];
    seg.t_start  = static_cast<double>(i)   / n_seg;
    seg.t_end    = static_cast<double>(i+1) / n_seg;
    seg.complexity = complexity_per_waypoint[i];
    seg.is_tight   = (seg.complexity > tight_threshold);

    // Base knots + complexity-proportional bonus
    const double frac = (total_complexity > 1e-6)
      ? complexity_per_waypoint[i] / total_complexity : 0.0;
    const int bonus = static_cast<int>(
      std::round(frac * (max_total_knots - min_knots_per_segment * n_seg)));
    seg.num_knots = min_knots_per_segment + std::max(0, bonus);
    allocated += seg.num_knots;
  }

  // Trim if over budget
  while (allocated > max_total_knots && n_seg > 0) {
    // Find segment with most knots above minimum and trim it
    int max_idx = 0;
    for (int i = 1; i < n_seg; ++i) {
      if (map.segments[i].num_knots > map.segments[max_idx].num_knots)
        max_idx = i;
    }
    if (map.segments[max_idx].num_knots <= min_knots_per_segment) break;
    map.segments[max_idx].num_knots--;
    allocated--;
  }

  map.total_knots = allocated;
  return map;
}

}  // namespace vf_robot_controller::models
