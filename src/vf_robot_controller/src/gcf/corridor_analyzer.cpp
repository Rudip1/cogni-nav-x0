#include "nav2_costmap_2d/cost_values.hpp"
#include "vf_robot_controller/gcf/corridor_analyzer.hpp"
#include <cmath>
#include <algorithm>

namespace vf_robot_controller::gcf
{

CorridorAnalyzer::CorridorAnalyzer(const Parameters & params)
: params_(params)
{}

std::vector<CorridorAnalyzer::CorridorInfo> CorridorAnalyzer::analyze(
  const std::vector<Eigen::Vector2d> & waypoints,
  const std::vector<double> & headings,
  const nav2_costmap_2d::Costmap2D * costmap) const
{
  std::vector<CorridorInfo> results;
  results.reserve(waypoints.size());

  for (size_t i = 0; i < waypoints.size(); ++i) {
    const double heading = (i < headings.size()) ? headings[i] : 0.0;
    results.push_back(analyzePoint(
      waypoints[i].x(), waypoints[i].y(), heading, costmap));
  }
  return results;
}

CorridorAnalyzer::CorridorInfo CorridorAnalyzer::analyzePoint(
  double x, double y, double heading,
  const nav2_costmap_2d::Costmap2D * costmap) const
{
  CorridorInfo info;
  const double max_dist = params_.gcf_radius * 2.0;

  // Perpendicular directions to heading
  const double left_dx  = -std::sin(heading);
  const double left_dy  =  std::cos(heading);
  const double right_dx =  std::sin(heading);
  const double right_dy = -std::cos(heading);

  info.left_clearance  = raycastDistance(x, y, left_dx,  left_dy,  max_dist, costmap);
  info.right_clearance = raycastDistance(x, y, right_dx, right_dy, max_dist, costmap);
  info.width           = info.left_clearance + info.right_clearance;

  const double narrow_thresh  = params_.robot_width * 1.8;
  const double doorway_thresh = params_.robot_width * 1.3;

  info.is_narrow  = (info.width < narrow_thresh);
  info.is_doorway = (info.width < doorway_thresh);

  return info;
}

double CorridorAnalyzer::raycastDistance(
  double x, double y,
  double dir_x, double dir_y,
  double max_dist,
  const nav2_costmap_2d::Costmap2D * costmap) const
{
  const double res = costmap->getResolution();
  const double step = res * 0.8;  // slightly smaller than cell for accuracy
  double dist = 0.0;

  while (dist < max_dist) {
    const double qx = x + dir_x * dist;
    const double qy = y + dir_y * dist;

    unsigned int mx, my;
    if (!costmap->worldToMap(qx, qy, mx, my)) {
      return dist;  // hit map boundary
    }
    if (costmap->getCost(mx, my) >= nav2_costmap_2d::LETHAL_OBSTACLE) {
      return dist;
    }
    dist += step;
  }
  return max_dist;
}

}  // namespace vf_robot_controller::gcf
