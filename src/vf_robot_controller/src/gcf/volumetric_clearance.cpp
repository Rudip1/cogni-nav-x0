#include "vf_robot_controller/gcf/volumetric_clearance.hpp"
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <cmath>
#include <algorithm>
#include <limits>

namespace vf_robot_controller::gcf
{

VolumetricClearance::VolumetricClearance(const Parameters & params)
: params_(params)
{}

void VolumetricClearance::updateFromPointcloud(
  const sensor_msgs::msg::PointCloud2::SharedPtr & cloud,
  double robot_x, double robot_y)
{
  if (!cloud) return;
  data_origin_x_ = robot_x;
  data_origin_y_ = robot_y;
  relevant_points_.clear();

  const float h_min = static_cast<float>(params_.robot_height_min);
  const float h_max = static_cast<float>(params_.robot_height_max);
  const float query_r = static_cast<float>(params_.gcf_radius * 2.0);

  // Iterate pointcloud — filter to robot height band and local radius
  sensor_msgs::PointCloud2ConstIterator<float> it_x(*cloud, "x");
  sensor_msgs::PointCloud2ConstIterator<float> it_y(*cloud, "y");
  sensor_msgs::PointCloud2ConstIterator<float> it_z(*cloud, "z");

  for (; it_x != it_x.end(); ++it_x, ++it_y, ++it_z) {
    const float z = *it_z;
    if (z < h_min || z > h_max) continue;  // outside robot height band

    const float dx = *it_x - static_cast<float>(robot_x);
    const float dy = *it_y - static_cast<float>(robot_y);
    if (dx*dx + dy*dy > query_r*query_r) continue;  // outside local window

    relevant_points_.push_back({*it_x, *it_y, z});
  }

  has_data_ = !relevant_points_.empty();
  if (has_data_) buildSpatialIndex();
}

void VolumetricClearance::buildSpatialIndex()
{
  if (relevant_points_.empty()) return;

  // Build a coarse grid index for O(1) range queries
  index_resolution_ = 0.15;  // m — coarser than costmap, fine enough for chairs
  const double range = params_.gcf_radius * 2.0;

  index_origin_x_ = data_origin_x_ - range;
  index_origin_y_ = data_origin_y_ - range;
  index_width_  = static_cast<int>(std::ceil(2.0 * range / index_resolution_)) + 1;
  index_height_ = index_width_;

  spatial_index_.assign(index_width_ * index_height_, {});

  for (size_t i = 0; i < relevant_points_.size(); ++i) {
    const auto & pt = relevant_points_[i];
    const int cx = static_cast<int>((pt.x - index_origin_x_) / index_resolution_);
    const int cy = static_cast<int>((pt.y - index_origin_y_) / index_resolution_);
    if (cx < 0 || cx >= index_width_ || cy < 0 || cy >= index_height_) continue;
    spatial_index_[cy * index_width_ + cx].push_back(i);
  }
}

std::vector<double> VolumetricClearance::computeField(
  double origin_x, double origin_y,
  int width, int height, double resolution) const
{
  std::vector<double> field(width * height, 1.0);  // default: clear

  if (!has_data_) return field;

  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      const double wx = origin_x + (x + 0.5) * resolution;
      const double wy = origin_y + (y + 0.5) * resolution;
      field[y * width + x] = queryAt(wx, wy);
    }
  }
  return field;
}

double VolumetricClearance::queryAt(double wx, double wy) const
{
  if (!has_data_) return 1.0;

  // Find minimum distance to any relevant point near (wx, wy)
  const double search_r = params_.robot_width * 0.6;
  double min_dist = std::numeric_limits<double>::max();

  // Query spatial index cells in range
  const int cx0 = static_cast<int>((wx - search_r - index_origin_x_) / index_resolution_);
  const int cx1 = static_cast<int>((wx + search_r - index_origin_x_) / index_resolution_);
  const int cy0 = static_cast<int>((wy - search_r - index_origin_y_) / index_resolution_);
  const int cy1 = static_cast<int>((wy + search_r - index_origin_y_) / index_resolution_);

  for (int cy = std::max(0, cy0); cy <= std::min(index_height_-1, cy1); ++cy) {
    for (int cx = std::max(0, cx0); cx <= std::min(index_width_-1, cx1); ++cx) {
      for (size_t idx : spatial_index_[cy * index_width_ + cx]) {
        const auto & pt = relevant_points_[idx];
        const double dx = pt.x - wx;
        const double dy = pt.y - wy;
        const double d  = std::sqrt(dx*dx + dy*dy);
        min_dist = std::min(min_dist, d);
      }
    }
  }

  if (min_dist == std::numeric_limits<double>::max()) return 1.0;

  // Normalize: 0 = point touching robot, 1 = clear at search_r
  return std::clamp(min_dist / search_r, 0.0, 1.0);
}

bool VolumetricClearance::isTraversable(
  double wx, double wy, double robot_half_width) const
{
  return queryAt(wx, wy) * (params_.robot_width * 0.6) > robot_half_width;
}

std::vector<size_t> VolumetricClearance::queryCell(int cx, int cy) const
{
  if (cx < 0 || cx >= index_width_ || cy < 0 || cy >= index_height_) return {};
  return spatial_index_[cy * index_width_ + cx];
}

}  // namespace vf_robot_controller::gcf
