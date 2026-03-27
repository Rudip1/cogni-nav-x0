#include "vf_robot_controller/gcf/clutter_detector.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include <queue>
#include <unordered_set>
#include <cmath>

namespace vf_robot_controller::gcf
{

ClutterDetector::ClutterDetector(const Parameters & params)
: params_(params)
{}

std::vector<double> ClutterDetector::computeField(
  const nav2_costmap_2d::Costmap2D * costmap) const
{
  const int W = static_cast<int>(costmap->getSizeInCellsX());
  const int H = static_cast<int>(costmap->getSizeInCellsY());
  const int n = W * H;

  // Step 1: Build binary obstacle mask
  std::vector<bool> mask(n, false);
  for (int y = 0; y < H; ++y) {
    for (int x = 0; x < W; ++x) {
      if (costmap->getCost(x, y) >= nav2_costmap_2d::LETHAL_OBSTACLE) {
        mask[y * W + x] = true;
      }
    }
  }

  // Step 2: Connected component labeling (4-connectivity BFS)
  const std::vector<int> labels = labelConnectedComponents(mask, W, H);

  // Step 3: For each cell, count distinct component IDs within radius
  const double res = costmap->getResolution();
  const int radius_cells = static_cast<int>(
    std::ceil(params_.gcf_clutter_radius / res));

  std::vector<double> field(n, 0.0);
  for (int y = 0; y < H; ++y) {
    for (int x = 0; x < W; ++x) {
      const int count = countClustersInRadius(x, y, labels, W, H, radius_cells);
      field[y * W + x] = std::min(
        static_cast<double>(count) / max_expected_clusters_, 1.0);
    }
  }
  return field;
}

double ClutterDetector::queryAt(
  double wx, double wy,
  const nav2_costmap_2d::Costmap2D * costmap) const
{
  unsigned int mx, my;
  if (!costmap->worldToMap(wx, wy, mx, my)) return 0.0;

  const int W = static_cast<int>(costmap->getSizeInCellsX());
  const int H = static_cast<int>(costmap->getSizeInCellsY());
  const int n = W * H;

  std::vector<bool> mask(n, false);
  for (int y = 0; y < H; ++y)
    for (int x = 0; x < W; ++x)
      if (costmap->getCost(x, y) >= nav2_costmap_2d::LETHAL_OBSTACLE)
        mask[y * W + x] = true;

  const auto labels = labelConnectedComponents(mask, W, H);

  const double res = costmap->getResolution();
  const int radius_cells = static_cast<int>(
    std::ceil(params_.gcf_clutter_radius / res));

  const int count = countClustersInRadius(
    static_cast<int>(mx), static_cast<int>(my), labels, W, H, radius_cells);

  return std::min(static_cast<double>(count) / max_expected_clusters_, 1.0);
}

std::vector<int> ClutterDetector::labelConnectedComponents(
  const std::vector<bool> & mask,
  int W, int H) const
{
  std::vector<int> labels(W * H, 0);
  int current_label = 0;

  const int dx[] = {1, -1, 0, 0};
  const int dy[] = {0, 0, 1, -1};

  for (int y = 0; y < H; ++y) {
    for (int x = 0; x < W; ++x) {
      const int idx = y * W + x;
      if (!mask[idx] || labels[idx] != 0) continue;

      ++current_label;
      std::queue<std::pair<int,int>> q;
      q.push({x, y});
      labels[idx] = current_label;

      while (!q.empty()) {
        auto [cx, cy] = q.front(); q.pop();
        for (int d = 0; d < 4; ++d) {
          const int nx = cx + dx[d];
          const int ny = cy + dy[d];
          if (nx < 0 || nx >= W || ny < 0 || ny >= H) continue;
          const int nidx = ny * W + nx;
          if (!mask[nidx] || labels[nidx] != 0) continue;
          labels[nidx] = current_label;
          q.push({nx, ny});
        }
      }
    }
  }
  return labels;
}

int ClutterDetector::countClustersInRadius(
  int cx, int cy,
  const std::vector<int> & labels,
  int W, int H,
  int radius_cells) const
{
  std::unordered_set<int> seen;

  const int x0 = std::max(0, cx - radius_cells);
  const int x1 = std::min(W - 1, cx + radius_cells);
  const int y0 = std::max(0, cy - radius_cells);
  const int y1 = std::min(H - 1, cy + radius_cells);

  const int r2 = radius_cells * radius_cells;

  for (int y = y0; y <= y1; ++y) {
    for (int x = x0; x <= x1; ++x) {
      const int dx = x - cx, dy = y - cy;
      if (dx*dx + dy*dy > r2) continue;
      const int lbl = labels[y * W + x];
      if (lbl > 0) seen.insert(lbl);
    }
  }
  return static_cast<int>(seen.size());
}

}  // namespace vf_robot_controller::gcf
