#pragma once

#include <memory>
#include <string>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav2_core/controller.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_ros/buffer.h"

#include "vf_robot_controller/parameter_handler.hpp"
#include "vf_robot_controller/optimizer.hpp"
#include "vf_robot_controller/safety/hard_safety_shell.hpp"
#include "vf_robot_controller/gcf/geometric_complexity_field.hpp"
#include "vf_robot_controller/tools/trajectory_visualizer.hpp"
#include "vf_robot_controller/tools/weight_adapter.hpp"
#include "vf_robot_controller/tools/data_recorder.hpp"

namespace vf_robot_controller
{

/**
 * @brief Operating mode — set via controller_mode YAML parameter.
 *
 *   FIXED     : classical MPPI, fixed YAML weights, no Python nodes needed.
 *               Use this as the baseline for thesis benchmarking.
 *
 *   COLLECT   : fixed weights + DataRecorder active.
 *               Publishes critic data to /vf_controller/critic_data.
 *               Run meta_critic_data_logger.py alongside to write HDF5 data.
 *
 *   INFERENCE : WeightAdapter active, receives dynamic weights from
 *               meta_critic_inference_node.py via /vf_controller/meta_weights.
 *               Falls back to YAML weights if Python node is absent.
 *
 *   PASSIVE   : satisfies Nav2 action server but returns zero velocity.
 *               Python imitation_inference_node.py owns /cmd_vel directly.
 *               Use for imitation model live deployment.
 */
enum class ControllerMode { FIXED, COLLECT, INFERENCE, PASSIVE };

class VFRobotController : public nav2_core::Controller
{
public:
  VFRobotController() = default;
  ~VFRobotController() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf_buffer,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void activate() override;
  void deactivate() override;
  void cleanup() override;

  void setPlan(const nav_msgs::msg::Path & path) override;

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;

  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

private:
  void pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  static ControllerMode modeFromString(const std::string & s);

  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  rclcpp::Logger logger_{rclcpp::get_logger("VFRobotController")};
  std::string name_;

  std::shared_ptr<tf2_ros::Buffer>              tf_buffer_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav_msgs::msg::Path global_plan_;

  std::unique_ptr<ParameterHandler>              param_handler_;
  std::unique_ptr<Optimizer>                     optimizer_;
  std::unique_ptr<safety::HardSafetyShell>       safety_shell_;
  std::shared_ptr<gcf::GeometricComplexityField> gcf_;
  std::unique_ptr<tools::TrajectoryVisualizer>   visualizer_;

  // ── Meta-critic additions ─────────────────────────────────────────────────
  ControllerMode                           mode_{ControllerMode::FIXED};
  std::unique_ptr<tools::WeightAdapter>    weight_adapter_;
  std::unique_ptr<tools::DataRecorder>     data_recorder_;

  // Pointcloud
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_sub_;
  sensor_msgs::msg::PointCloud2::SharedPtr latest_pointcloud_;
  std::mutex pcl_mutex_;

  double speed_limit_{1.0};
  bool   speed_limit_is_percentage_{false};
};

}  // namespace vf_robot_controller
