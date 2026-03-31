#include "nav2_costmap_2d/cost_values.hpp"
#include "vf_robot_controller/safety/hard_safety_shell.hpp"
#include "vf_robot_controller/safety/swept_volume_checker.hpp"
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>

namespace vf_robot_controller::safety
{

static rclcpp::Logger SAFETY_LOG = rclcpp::get_logger("HardSafetyShell");

HardSafetyShell::HardSafetyShell(const Parameters & params)
: params_(params)
{}

geometry_msgs::msg::Twist HardSafetyShell::check(
  const geometry_msgs::msg::Twist & proposed,
  const geometry_msgs::msg::PoseStamped & pose,
  const nav2_costmap_2d::Costmap2D * costmap,
  const sensor_msgs::msg::PointCloud2::SharedPtr & pointcloud) const
{
  // Check 2D swept volume
  const bool blocked_2d = checkSweptVolume2D(proposed, pose, costmap);

  // Check 3D swept volume if data available
  bool blocked_3d = false;
  if (pointcloud && params_.use_3d_clearance) {
    blocked_3d = checkSweptVolume3D(proposed, pose, pointcloud);
  }

  if (blocked_2d || blocked_3d) {
    last_vetoed_ = true;
    consecutive_vetoes_++;
    RCLCPP_WARN(SAFETY_LOG,
      "SAFETY VETO [%s%s] — stopping robot. Consecutive: %d",
      blocked_2d ? "2D " : "",
      blocked_3d ? "3D" : "",
      consecutive_vetoes_);

    // Escape mechanism — only activates after 5 consecutive vetoes
    // Tries all escape directions, picks first safe one
    // If all fail, robot is truly trapped — let BT recovery handle it
    if (consecutive_vetoes_ > 5) {
      // Candidate escape velocities: back, forward, turn-left, turn-right
      std::vector<geometry_msgs::msg::Twist> candidates;
      geometry_msgs::msg::Twist t;

      t = geometry_msgs::msg::Twist{};
      t.linear.x = -0.05;  // backward
      candidates.push_back(t);

      t = geometry_msgs::msg::Twist{};
      t.linear.x = 0.05;   // forward
      candidates.push_back(t);

      t = geometry_msgs::msg::Twist{};
      t.angular.z = 0.3;   // turn left
      candidates.push_back(t);

      t = geometry_msgs::msg::Twist{};
      t.angular.z = -0.3;  // turn right
      candidates.push_back(t);

      for (const auto & candidate : candidates) {
        if (safe) {
          RCLCPP_WARN(SAFETY_LOG,
            "Safety escape: vx=%.2f wz=%.2f",
            candidate.linear.x, candidate.angular.z);
          return candidate;
        }
      }

      // All directions blocked — truly trapped, stop and let BT recover
      RCLCPP_WARN(SAFETY_LOG,
        "Safety shell: all escape directions blocked — waiting for BT recovery");
    }
    return geometry_msgs::msg::Twist{};
  }

  last_vetoed_ = false;
  consecutive_vetoes_ = 0;
  return proposed;
}

bool HardSafetyShell::checkSweptVolume2D(
  const geometry_msgs::msg::Twist & vel,
  const geometry_msgs::msg::PoseStamped & pose,
  const nav2_costmap_2d::Costmap2D * costmap) const
{
  SweptVolumeChecker checker(params_);

  const auto & q = pose.pose.orientation;
  const double yaw = std::atan2(
    2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z));

  auto cells = checker.computeSweptCells(
    pose.pose.position.x, pose.pose.position.y, yaw,
    vel.linear.x, vel.angular.z,
    check_horizon_s_,
    costmap->getResolution(),
    costmap->getOriginX(), costmap->getOriginY(),
    footprint_samples_);

  for (const auto & cell : cells) {
    if (cell.mx < 0 || cell.my < 0 ||
        cell.mx >= static_cast<int>(costmap->getSizeInCellsX()) ||
        cell.my >= static_cast<int>(costmap->getSizeInCellsY())) {
      return true;  // out of map = unsafe
    }
    const unsigned char cost = costmap->getCost(
      static_cast<unsigned int>(cell.mx),
      static_cast<unsigned int>(cell.my));
    if (cost == nav2_costmap_2d::LETHAL_OBSTACLE) return true;
  }
  return false;
}

bool HardSafetyShell::checkSweptVolume3D(
  const geometry_msgs::msg::Twist & vel,
  const geometry_msgs::msg::PoseStamped & pose,
  const sensor_msgs::msg::PointCloud2::SharedPtr & cloud) const
{
  if (!cloud) return false;

  SweptVolumeChecker checker(params_);
  const auto & q = pose.pose.orientation;
  const double yaw = std::atan2(
    2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z));

  const auto swept_pts = checker.computeSweptWorldPoints(
    pose.pose.position.x, pose.pose.position.y, yaw,
    vel.linear.x, vel.angular.z,
    check_horizon_s_, footprint_samples_);

  const float h_min = static_cast<float>(params_.robot_height_min);
  const float h_max = static_cast<float>(params_.robot_height_max);
  const float margin = static_cast<float>(params_.safety_inflation);
  const float hw = static_cast<float>(params_.robot_width * 0.5 + margin);

  sensor_msgs::PointCloud2ConstIterator<float> it_x(*cloud, "x");
  sensor_msgs::PointCloud2ConstIterator<float> it_y(*cloud, "y");
  sensor_msgs::PointCloud2ConstIterator<float> it_z(*cloud, "z");

  for (; it_x != it_x.end(); ++it_x, ++it_y, ++it_z) {
    const float pz = *it_z;
    if (pz < h_min || pz > h_max) continue;

    const float px = *it_x;
    const float py = *it_y;

    // Check if point is inside any swept footprint rectangle
    for (const auto & sp : swept_pts) {
      const float dx = px - static_cast<float>(sp.x());
      const float dy = py - static_cast<float>(sp.y());
      if (std::abs(dx) < hw && std::abs(dy) < hw) {
        return true;  // 3D collision detected
      }
    }
  }
  return false;
}

}  // namespace vf_robot_controller::safety
