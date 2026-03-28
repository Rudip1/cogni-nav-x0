#include "vf_robot_controller/tools/weight_adapter.hpp"

namespace vf_robot_controller::tools
{

WeightAdapter::WeightAdapter(
  rclcpp_lifecycle::LifecycleNode::SharedPtr node,
  int num_critics,
  const std::string & topic,
  double timeout_ms)
: node_(node),
  num_critics_(num_critics),
  timeout_ms_(timeout_ms)
{
  // Initialise both vectors to uniform weights
  const float uniform = 1.0f / static_cast<float>(num_critics_);
  weights_.assign(num_critics_, uniform);
  fallback_.assign(num_critics_, uniform);

  // Set last_update far in the past so first call returns fallback
  // until a real message arrives
  last_update_ = node_->now() - rclcpp::Duration::from_seconds(10.0);

  sub_ = node_->create_subscription<std_msgs::msg::Float32MultiArray>(
    topic, rclcpp::QoS(10),
    [this](const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
      weightCallback(msg);
    });

  RCLCPP_INFO(logger_,
    "WeightAdapter: listening on '%s' (%d critics, %.0f ms timeout)",
    topic.c_str(), num_critics_, timeout_ms_);
}

void WeightAdapter::weightCallback(
  const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
  if (static_cast<int>(msg->data.size()) != num_critics_) {
    RCLCPP_WARN_ONCE(logger_,
      "WeightAdapter: received %zu weights, expected %d — ignoring",
      msg->data.size(), num_critics_);
    return;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  weights_ = msg->data;
  last_update_ = node_->now();
}

std::vector<float> WeightAdapter::getWeights()
{
  std::lock_guard<std::mutex> lock(mutex_);

  const double age_ms =
    (node_->now() - last_update_).seconds() * 1000.0;

  if (age_ms > timeout_ms_) {
    RCLCPP_WARN_THROTTLE(logger_, *node_->get_clock(), 5000,
      "WeightAdapter: weights stale (%.0f ms) — using fallback YAML weights",
      age_ms);
    return fallback_;
  }

  return weights_;
}

void WeightAdapter::setFallback(const std::vector<float> & fallback)
{
  if (static_cast<int>(fallback.size()) != num_critics_) {
    RCLCPP_WARN(logger_,
      "WeightAdapter::setFallback: size mismatch (%zu vs %d) — ignored",
      fallback.size(), num_critics_);
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  fallback_ = fallback;
  RCLCPP_INFO(logger_, "WeightAdapter: fallback weights updated from YAML");
}

bool WeightAdapter::isActive() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  const double age_ms =
    (node_->now() - last_update_).seconds() * 1000.0;
  return age_ms <= timeout_ms_;
}

}  // namespace vf_robot_controller::tools
