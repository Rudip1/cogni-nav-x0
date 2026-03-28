#include "vf_robot_controller/tools/data_recorder.hpp"
#include <tf2/utils.h>

namespace vf_robot_controller::tools
{

DataRecorder::DataRecorder(
  rclcpp_lifecycle::LifecycleNode::SharedPtr node,
  const std::string & topic)
: node_(node)
{
  pub_ = node_->create_publisher<std_msgs::msg::Float32MultiArray>(
    topic, rclcpp::QoS(10));

  RCLCPP_INFO(logger_, "DataRecorder: publishing to '%s'", topic.c_str());
}

void DataRecorder::record(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity,
  double goal_distance,
  double goal_heading,
  const std::vector<double> & critic_scores,
  int n_candidates,
  int n_critics,
  int best_idx)
{
  if (!enabled_) { return; }

  // Header: 11 scalar fields
  constexpr int HEADER_SIZE = 11;
  const int matrix_size = n_candidates * n_critics;
  std_msgs::msg::Float32MultiArray msg;
  msg.data.reserve(HEADER_SIZE + matrix_size);

  // Extract yaw from quaternion
  const double yaw = tf2::getYaw(pose.pose.orientation);

  // [0..10] scalars
  msg.data.push_back(static_cast<float>(
    rclcpp::Time(pose.header.stamp).seconds()));
  msg.data.push_back(static_cast<float>(pose.pose.position.x));
  msg.data.push_back(static_cast<float>(pose.pose.position.y));
  msg.data.push_back(static_cast<float>(yaw));
  msg.data.push_back(static_cast<float>(velocity.linear.x));
  msg.data.push_back(static_cast<float>(velocity.angular.z));
  msg.data.push_back(static_cast<float>(goal_distance));
  msg.data.push_back(static_cast<float>(goal_heading));
  msg.data.push_back(static_cast<float>(best_idx));
  msg.data.push_back(static_cast<float>(n_candidates));
  msg.data.push_back(static_cast<float>(n_critics));

  // [11 .. end] critic score matrix (double → float)
  for (const double s : critic_scores) {
    msg.data.push_back(static_cast<float>(s));
  }

  pub_->publish(msg);
}

}  // namespace vf_robot_controller::tools
