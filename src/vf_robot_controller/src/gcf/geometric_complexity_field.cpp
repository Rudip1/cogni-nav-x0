#include "nav2_costmap_2d/cost_values.hpp"
#include "vf_robot_controller/gcf/geometric_complexity_field.hpp"
#include "vf_robot_controller/gcf/corridor_analyzer.hpp"
#include "vf_robot_controller/gcf/clutter_detector.hpp"
#include "vf_robot_controller/gcf/volumetric_clearance.hpp"

#include <algorithm>
#include <cmath>

namespace vf_robot_controller::gcf
{

GeometricComplexityField::GeometricComplexityField(const Parameters & params)
: params_(params)
{}

void GeometricComplexityField::update(
  const nav2_costmap_2d::Costmap2D * costmap,
  const sensor_msgs::msg::PointCloud2::SharedPtr & pointcloud,
  double robot_x, double robot_y, double robot_yaw)
{
  robot_x_   = robot_x;
  robot_y_   = robot_y;
  robot_yaw_ = robot_yaw;

  // Sync grid dimensions with costmap
  width_      = static_cast<int>(costmap->getSizeInCellsX());
  height_     = static_cast<int>(costmap->getSizeInCellsY());
  resolution_ = costmap->getResolution();
  origin_x_   = costmap->getOriginX();
  origin_y_   = costmap->getOriginY();

  const int n = width_ * height_;
  grid_.resize(n);
  clearance_2d_field_.resize(n, 0.0);
  clutter_field_.resize(n, 0.0);
  volumetric_field_.resize(n, 1.0);  // default: fully clear

  // Step 1: 2D clearance from costmap
  compute2DClearance(costmap);

  // Step 2: Clutter density (distinct obstacle clusters)
  computeClutterDensity(costmap);

  // Step 3: Volumetric clearance from 3D pointcloud (if available)
  if (pointcloud && params_.use_3d_clearance) {
    computeVolumetricClearance(pointcloud);
  }

  // Step 4: Combine into final GCF
  combineComponents();
}

void GeometricComplexityField::compute2DClearance(
  const nav2_costmap_2d::Costmap2D * costmap)
{
  // Convert costmap cost to clearance distance
  // Lethal (254) → 0 clearance, free (0) → max clearance
  const double max_dist = params_.gcf_radius;
  const double res      = resolution_;

  for (int y = 0; y < height_; ++y) {
    for (int x = 0; x < width_; ++x) {
      const int idx = y * width_ + x;
      const unsigned char cost = costmap->getCost(x, y);

      if (cost >= nav2_costmap_2d::LETHAL_OBSTACLE) {
        clearance_2d_field_[idx] = 0.0;
        grid_[idx].traversable   = false;
      } else if (cost >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
        clearance_2d_field_[idx] = res * 0.5;
      } else {
        // Approximate clearance from inflation cost
        const double norm_cost = static_cast<double>(cost) / 252.0;
        const double clearance = (1.0 - norm_cost) * max_dist;
        clearance_2d_field_[idx] = clearance;
        grid_[idx].clearance_2d  = clearance;
      }
    }
  }
}

void GeometricComplexityField::computeClutterDensity(
  const nav2_costmap_2d::Costmap2D * costmap)
{
  ClutterDetector detector(params_);
  clutter_field_ = detector.computeField(costmap);

  for (int i = 0; i < static_cast<int>(grid_.size()); ++i) {
    grid_[i].clutter_density = clutter_field_[i];
  }
}

void GeometricComplexityField::computeVolumetricClearance(
  const sensor_msgs::msg::PointCloud2::SharedPtr & cloud)
{
  VolumetricClearance vol_checker(params_);
  vol_checker.updateFromPointcloud(cloud, robot_x_, robot_y_);

  if (!vol_checker.hasData()) return;

  volumetric_field_ = vol_checker.computeField(
    origin_x_, origin_y_, width_, height_, resolution_);

  for (int i = 0; i < static_cast<int>(grid_.size()); ++i) {
    grid_[i].clearance_3d = volumetric_field_[i];
  }
}

void GeometricComplexityField::combineComponents()
{
  const double w1 = params_.gcf_weight_2d;
  const double w2 = params_.gcf_weight_clutter;
  const double w3 = params_.gcf_weight_volumetric;

  for (int i = 0; i < static_cast<int>(grid_.size()); ++i) {
    if (!grid_[i].traversable) {
      grid_[i].complexity = 1.0;
      continue;
    }

    // Invert clearances: low clearance → high complexity
    const double c2d  = 1.0 - std::min(clearance_2d_field_[i] / params_.gcf_radius, 1.0);
    const double clut = clutter_field_[i];
    const double c3d  = 1.0 - std::min(volumetric_field_[i], 1.0);

    // Weighted combination
    double complexity = w1 * c2d + w2 * clut + w3 * c3d;
    complexity = std::clamp(complexity, 0.0, 1.0);

    grid_[i].complexity = complexity;
  }
}

GCFCell GeometricComplexityField::query(double wx, double wy) const
{
  if (grid_.empty()) return GCFCell{};

  // World → grid coordinates
  const double gx = (wx - origin_x_) / resolution_;
  const double gy = (wy - origin_y_) / resolution_;

  // Bilinear interpolation
  const int x0 = static_cast<int>(std::floor(gx));
  const int y0 = static_cast<int>(std::floor(gy));
  const int x1 = x0 + 1;
  const int y1 = y0 + 1;

  // Clamp to grid bounds
  auto inBounds = [&](int x, int y) {
    return x >= 0 && x < width_ && y >= 0 && y < height_;
  };

  if (!inBounds(x0, y0)) return GCFCell{0.5, 0.5, 0.5, 0.0, true};

  // Bilinear weights
  const double tx = gx - x0;
  const double ty = gy - y0;

  auto cellAt = [&](int x, int y) -> const GCFCell & {
    const int cx = std::clamp(x, 0, width_ - 1);
    const int cy = std::clamp(y, 0, height_ - 1);
    return grid_[cy * width_ + cx];
  };

  const auto & c00 = cellAt(x0, y0);
  const auto & c10 = cellAt(x1, y0);
  const auto & c01 = cellAt(x0, y1);
  const auto & c11 = cellAt(x1, y1);

  GCFCell result;
  auto lerp = [](double a, double b, double t) { return a + t * (b - a); };
  result.complexity = lerp(lerp(c00.complexity, c10.complexity, tx),
                           lerp(c01.complexity, c11.complexity, tx), ty);
  result.clearance_2d = lerp(lerp(c00.clearance_2d, c10.clearance_2d, tx),
                              lerp(c01.clearance_2d, c11.clearance_2d, tx), ty);
  result.clutter_density = lerp(lerp(c00.clutter_density, c10.clutter_density, tx),
                                 lerp(c01.clutter_density, c11.clutter_density, tx), ty);
  result.traversable = c00.traversable && c10.traversable &&
                       c01.traversable && c11.traversable;
  return result;
}

double GeometricComplexityField::meanComplexityAlongPath(
  const std::vector<Eigen::Vector2d> & waypoints) const
{
  if (waypoints.empty()) return 0.0;
  double sum = 0.0;
  for (const auto & wp : waypoints) {
    sum += query(wp.x(), wp.y()).complexity;
  }
  return sum / static_cast<double>(waypoints.size());
}

std::vector<double> GeometricComplexityField::complexityPerSegment(
  const std::vector<Eigen::Vector2d> & waypoints) const
{
  if (waypoints.size() < 2) return {};
  std::vector<double> result;
  result.reserve(waypoints.size() - 1);
  for (size_t i = 0; i + 1 < waypoints.size(); ++i) {
    // Sample midpoint of each segment
    const Eigen::Vector2d mid = 0.5 * (waypoints[i] + waypoints[i+1]);
    result.push_back(query(mid.x(), mid.y()).complexity);
  }
  return result;
}

}  // namespace vf_robot_controller::gcf
