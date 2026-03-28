#pragma once

#include <vector>
#include <mutex>
#include <string>
#include <atomic>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"

namespace vf_robot_controller::tools
{

/**
 * @brief DataRecorder — publishes raw critic data every control cycle
 *        when the controller runs in COLLECT mode (Mode 2).
 *
 * Publishes to /vf_controller/critic_data (Float32MultiArray).
 * The Python data_logger.py subscribes and writes to HDF5.
 *
 * Message layout (flat float32 array):
 *   [0]          timestamp (seconds, double cast to float — precision ok for logging)
 *   [1]          robot_x
 *   [2]          robot_y
 *   [3]          robot_theta
 *   [4]          linear_vel
 *   [5]          angular_vel
 *   [6]          goal_distance
 *   [7]          goal_heading_error
 *   [8]          best_trajectory_idx
 *   [9]          n_candidates  (N)
 *   [10]         n_critics     (K)
 *   [11 .. 11+N*K-1]  critic score matrix, row-major [traj_i * K + critic_j]
 *
 * Total size: 11 + N*K floats.
 * For N=200, K=10: 2011 floats per message = ~8 KB at 20 Hz = ~160 KB/s.
 * Python side writes to HDF5 with resizable datasets — no memory issues.
 *
 * Only active when enabled_ == true. Call setEnabled(true) when the
 * controller is configured in COLLECT mode.
 */
class DataRecorder
{
public:
  explicit DataRecorder(
    rclcpp_lifecycle::LifecycleNode::SharedPtr node,
    const std::string & topic = "/vf_controller/critic_data");

  /**
   * @brief Publish one record. Called once per control cycle in COLLECT mode.
   *
   * @param pose          Current robot pose
   * @param velocity      Current robot velocity
   * @param goal_distance Distance to current goal
   * @param goal_heading  Heading error to goal (radians)
   * @param critic_scores Flat score matrix [n_candidates x n_critics], row-major
   * @param n_candidates  Number of trajectory candidates (N)
   * @param n_critics     Number of critics (K)
   * @param best_idx      Index of the selected trajectory
   */
  void record(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    double goal_distance,
    double goal_heading,
    const std::vector<double> & critic_scores,
    int n_candidates,
    int n_critics,
    int best_idx);

  void setEnabled(bool enabled) { enabled_ = enabled; }
  bool isEnabled() const { return enabled_; }

private:
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_;
  std::atomic<bool> enabled_{false};
  rclcpp::Logger logger_{rclcpp::get_logger("DataRecorder")};
};

}  // namespace vf_robot_controller::tools
