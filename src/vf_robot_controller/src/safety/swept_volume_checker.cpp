#include "vf_robot_controller/safety/swept_volume_checker.hpp"
#include <cmath>
#include <algorithm>
#include <unordered_set>

namespace vf_robot_controller::safety
{

SweptVolumeChecker::SweptVolumeChecker(const Parameters & params)
: params_(params)
{}

std::vector<SweptVolumeChecker::FootprintCell>
SweptVolumeChecker::computeSweptCells(
  double x0, double y0, double yaw0,
  double v, double omega,
  double dt, double resolution,
  double origin_x, double origin_y,
  int n_steps) const
{
  // Use a set to deduplicate cells
  struct PairHash {
    size_t operator()(const std::pair<int,int> & p) const {
      return std::hash<int>()(p.first) ^ (std::hash<int>()(p.second) << 16);
    }
  };
  std::unordered_set<std::pair<int,int>, PairHash> cell_set;

  const double hl = params_.robot_length * 0.5 + params_.safety_inflation;
  const double hw = params_.robot_width  * 0.5 + params_.safety_inflation;

  double x = x0, y = y0, yaw = yaw0;
  const double step_dt = dt / n_steps;

  for (int step = 0; step <= n_steps; ++step) {
    // Forward simulate to this pose
    if (step > 0) {
      if (std::abs(omega) < 1e-6) {
        x   += v * std::cos(yaw) * step_dt;
        y   += v * std::sin(yaw) * step_dt;
      } else {
        x   += (v / omega) * (std::sin(yaw + omega * step_dt) - std::sin(yaw));
        y   += (v / omega) * (std::cos(yaw) - std::cos(yaw + omega * step_dt));
        yaw += omega * step_dt;
      }
    }

    // Get footprint corners at this pose
    auto corners = rectangleCorners(x, y, yaw, hl, hw);

    // Rasterize into cells
    auto cells = rasterizeFootprint(corners, resolution, origin_x, origin_y);
    for (const auto & c : cells) {
      cell_set.insert({c.mx, c.my});
    }
  }

  std::vector<FootprintCell> result;
  result.reserve(cell_set.size());
  for (const auto & [mx, my] : cell_set) {
    result.push_back({mx, my});
  }
  return result;
}

std::vector<Eigen::Vector2d> SweptVolumeChecker::computeSweptWorldPoints(
  double x0, double y0, double yaw0,
  double v, double omega, double dt,
  int n_steps) const
{
  std::vector<Eigen::Vector2d> pts;

  const double hl = params_.robot_length * 0.5 + params_.safety_inflation;
  const double hw = params_.robot_width  * 0.5 + params_.safety_inflation;

  double x = x0, y = y0, yaw = yaw0;
  const double step_dt = dt / n_steps;

  for (int step = 0; step <= n_steps; ++step) {
    if (step > 0) {
      if (std::abs(omega) < 1e-6) {
        x   += v * std::cos(yaw) * step_dt;
        y   += v * std::sin(yaw) * step_dt;
      } else {
        x   += (v / omega) * (std::sin(yaw + omega * step_dt) - std::sin(yaw));
        y   += (v / omega) * (std::cos(yaw) - std::cos(yaw + omega * step_dt));
        yaw += omega * step_dt;
      }
    }
    // Store footprint corners and centre
    pts.push_back({x, y});
    auto corners = rectangleCorners(x, y, yaw, hl, hw);
    for (const auto & c : corners) pts.push_back(c);
  }
  return pts;
}

std::vector<Eigen::Vector2d> SweptVolumeChecker::rectangleCorners(
  double cx, double cy, double yaw,
  double half_length, double half_width) const
{
  const double cos_y = std::cos(yaw);
  const double sin_y = std::sin(yaw);

  // 4 corners of oriented rectangle
  std::vector<Eigen::Vector2d> corners(4);
  const double lx = cos_y * half_length;
  const double ly = sin_y * half_length;
  const double wx = -sin_y * half_width;
  const double wy =  cos_y * half_width;

  corners[0] = {cx + lx + wx, cy + ly + wy};
  corners[1] = {cx + lx - wx, cy + ly - wy};
  corners[2] = {cx - lx - wx, cy - ly - wy};
  corners[3] = {cx - lx + wx, cy - ly + wy};
  return corners;
}

std::vector<SweptVolumeChecker::FootprintCell>
SweptVolumeChecker::rasterizeFootprint(
  const std::vector<Eigen::Vector2d> & corners,
  double resolution, double origin_x, double origin_y) const
{
  // Find bounding box of the 4 corners
  double min_x = corners[0].x(), max_x = corners[0].x();
  double min_y = corners[0].y(), max_y = corners[0].y();
  for (const auto & c : corners) {
    min_x = std::min(min_x, c.x()); max_x = std::max(max_x, c.x());
    min_y = std::min(min_y, c.y()); max_y = std::max(max_y, c.y());
  }

  // Convert corners to grid for point-in-polygon test
  std::vector<FootprintCell> cells;
  const int gx0 = static_cast<int>((min_x - origin_x) / resolution) - 1;
  const int gx1 = static_cast<int>((max_x - origin_x) / resolution) + 1;
  const int gy0 = static_cast<int>((min_y - origin_y) / resolution) - 1;
  const int gy1 = static_cast<int>((max_y - origin_y) / resolution) + 1;

  // For each grid cell in bounding box, test if cell centre is inside polygon
  for (int gy = gy0; gy <= gy1; ++gy) {
    for (int gx = gx0; gx <= gx1; ++gx) {
      const double px = origin_x + (gx + 0.5) * resolution;
      const double py = origin_y + (gy + 0.5) * resolution;

      // Ray-casting point-in-polygon for the 4-corner footprint
      bool inside = false;
      const int n = static_cast<int>(corners.size());
      for (int i = 0, j = n - 1; i < n; j = i++) {
        const double xi = corners[i].x(), yi = corners[i].y();
        const double xj = corners[j].x(), yj = corners[j].y();
        if (((yi > py) != (yj > py)) &&
            (px < (xj - xi) * (py - yi) / (yj - yi) + xi)) {
          inside = !inside;
        }
      }
      if (inside) cells.push_back({gx, gy});
    }
  }
  return cells;
}

}  // namespace vf_robot_controller::safety
