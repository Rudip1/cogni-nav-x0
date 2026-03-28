#pragma once

#include <vector>
#include <mutex>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

namespace vf_robot_controller::tools
{

/**
 * @brief WeightAdapter — bridges the Python meta-critic inference node
 *        and the C++ CriticManager.
 *
 * Subscribes to /vf_controller/meta_weights (Float32MultiArray, K floats).
 * Thread-safe: getWeights() can be called from the control loop thread
 * while the subscription callback fires on the executor thread.
 *
 * Fallback behaviour:
 *   If no message is received within timeout_ms_, getWeights() returns
 *   fallback_ (uniform weights set from YAML at startup). This means
 *   the controller silently degrades to classical fixed-weight MPPI
 *   if the Python inference node crashes or is not launched.
 *
 * Slot order must match critics.xml registration order:
 *   0  ObstacleCritic
 *   1  VolumetricCritic
 *   2  DynamicObstacleCritic
 *   3  PathFollowCritic
 *   4  SmoothnessCritic
 *   5  GoalCritic
 *   6  VelocityCritic
 *   7  CorridorCritic
 *   8  ClearanceCritic
 *   9  OscillationCritic
 */
class WeightAdapter
{
public:
  /**
   * @param node        Lifecycle node (used for subscription + clock)
   * @param num_critics Number of critics — must match network output dim
   * @param topic       Topic name for incoming weights
   * @param timeout_ms  Stale-weight timeout in milliseconds (default 200)
   */
  WeightAdapter(
    rclcpp_lifecycle::LifecycleNode::SharedPtr node,
    int num_critics,
    const std::string & topic = "/vf_controller/meta_weights",
    double timeout_ms = 200.0);

  /**
   * @brief Returns the latest valid weight vector.
   *
   * If the last received message is older than timeout_ms_, returns
   * fallback_ (uniform weights) and logs a throttled warning.
   * Never returns an empty vector — always num_critics_ elements.
   */
  std::vector<float> getWeights();

  /**
   * @brief Set the fallback weights used when the inference node is absent.
   *
   * Call this from ParameterHandler after loading YAML weights so the
   * fallback reflects the operator-configured values, not just uniform.
   * Ignored if fallback.size() != num_critics_.
   */
  void setFallback(const std::vector<float> & fallback);

  /**
   * @brief Returns true if a fresh weight message was received within timeout.
   * Useful for the controller to log operating mode.
   */
  bool isActive() const;

  int numCritics() const { return num_critics_; }

private:
  void weightCallback(
    const std_msgs::msg::Float32MultiArray::SharedPtr msg);

  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_;

  mutable std::mutex mutex_;
  std::vector<float> weights_;    // latest received weights
  std::vector<float> fallback_;   // YAML-configured fallback
  rclcpp::Time last_update_;

  int num_critics_;
  double timeout_ms_;

  rclcpp::Logger logger_{rclcpp::get_logger("WeightAdapter")};
};

}  // namespace vf_robot_controller::tools
