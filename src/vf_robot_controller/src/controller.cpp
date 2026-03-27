#include "vf_robot_controller/controller.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace vf_robot_controller
{

void VFRobotController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf_buffer,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_        = parent;
  name_        = name;
  tf_buffer_   = tf_buffer;
  costmap_ros_ = costmap_ros;

  auto node = node_.lock();
  logger_ = node->get_logger();
  RCLCPP_INFO(logger_, "Configuring VFRobotController: %s", name_.c_str());

  // ── Parameters ────────────────────────────────────────────────────────────
  auto costmap = costmap_ros_->getCostmap();
  param_handler_ = std::make_unique<ParameterHandler>(
    node, name_,
    costmap_ros_->getGlobalFrameID(),
    costmap->getSizeInMetersX(),
    costmap->getSizeInMetersY());

  const auto & p = param_handler_->getParams();

  // ── GCF (shared with optimizer) ───────────────────────────────────────────
  gcf_ = std::make_shared<gcf::GeometricComplexityField>(p);

  // ── Optimizer ─────────────────────────────────────────────────────────────
  optimizer_ = std::make_unique<Optimizer>();
  optimizer_->initialize(node, p, gcf_);

  // ── Hard safety shell ─────────────────────────────────────────────────────
  safety_shell_ = std::make_unique<safety::HardSafetyShell>(p);

  // ── Visualizer ────────────────────────────────────────────────────────────
  visualizer_ = std::make_unique<tools::TrajectoryVisualizer>(node, p);

  // ── 3D pointcloud subscription ────────────────────────────────────────────
  if (p.use_3d_clearance) {
    pcl_sub_ = node->create_subscription<sensor_msgs::msg::PointCloud2>(
      p.pointcloud_topic, rclcpp::SensorDataQoS(),
      std::bind(&VFRobotController::pointcloudCallback, this, std::placeholders::_1));
    RCLCPP_INFO(logger_, "Subscribed to 3D pointcloud: %s", p.pointcloud_topic.c_str());
  }

  RCLCPP_INFO(logger_, "VFRobotController configured successfully");
}

void VFRobotController::activate()
{
  RCLCPP_INFO(logger_, "Activating VFRobotController");
  optimizer_->activate();
  visualizer_->activate();
}

void VFRobotController::deactivate()
{
  RCLCPP_INFO(logger_, "Deactivating VFRobotController");
  optimizer_->deactivate();
  visualizer_->deactivate();
}

void VFRobotController::cleanup()
{
  RCLCPP_INFO(logger_, "Cleaning up VFRobotController");
  optimizer_.reset();
  safety_shell_.reset();
  gcf_.reset();
  visualizer_.reset();
  param_handler_.reset();
  pcl_sub_.reset();
}

void VFRobotController::setPlan(const nav_msgs::msg::Path & path)
{
  global_plan_ = path;
}

void VFRobotController::setSpeedLimit(
  const double & speed_limit, const bool & percentage)
{
  speed_limit_              = speed_limit;
  speed_limit_is_percentage_ = percentage;
}

geometry_msgs::msg::TwistStamped VFRobotController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * /*goal_checker*/)
{
  auto node = node_.lock();
  const auto & p = param_handler_->getParams();
  auto costmap   = costmap_ros_->getCostmap();

  // ── Get latest 3D data thread-safely ─────────────────────────────────────
  sensor_msgs::msg::PointCloud2::SharedPtr pcl_snap;
  {
    std::lock_guard<std::mutex> lock(pcl_mutex_);
    pcl_snap = latest_pointcloud_;
  }

  // ── Update GCF ────────────────────────────────────────────────────────────
  // GCF update runs at gcf_update_rate — the field self-throttles internally
  const double yaw = tools::poseYaw(pose);
  gcf_->update(costmap, pcl_snap,
    pose.pose.position.x, pose.pose.position.y, yaw);

  // ── Run optimizer ─────────────────────────────────────────────────────────
  geometry_msgs::msg::Twist raw_cmd = optimizer_->optimize(
    pose, velocity, global_plan_, costmap, pcl_snap);

  // Apply external speed limit
  if (speed_limit_is_percentage_) {
    raw_cmd.linear.x  *= speed_limit_;
    raw_cmd.angular.z *= speed_limit_;
  } else {
    raw_cmd.linear.x  = std::min(raw_cmd.linear.x,  speed_limit_);
  }

  // ── Hard safety shell — DECOUPLED veto ────────────────────────────────────
  geometry_msgs::msg::Twist safe_cmd = safety_shell_->check(
    raw_cmd, pose, costmap, pcl_snap);

  if (safety_shell_->lastCheckVetoed()) {
    RCLCPP_WARN(logger_,
      "HardSafetyShell VETO — consecutive: %d",
      safety_shell_->consecutiveVetoes());
    if (p.visualize_trajectories) {
      visualizer_->publishSafetyShellVeto(
        pose.pose.position.x, pose.pose.position.y, yaw,
        costmap_ros_->getGlobalFrameID());
    }
  }

  // ── Visualization ─────────────────────────────────────────────────────────
  if (p.visualize_trajectories) {
    visualizer_->publishSampledTrajectories(
      optimizer_->sampledTrajectories(),
      optimizer_->sampleCosts(),
      costmap_ros_->getGlobalFrameID());
    visualizer_->publishBestTrajectory(
      optimizer_->bestTrajectory(),
      costmap_ros_->getGlobalFrameID());
  }
  if (p.visualize_gcf) {
    visualizer_->publishGCFField(gcf_, costmap_ros_->getGlobalFrameID());
  }

  // ── Package result ────────────────────────────────────────────────────────
  geometry_msgs::msg::TwistStamped result;
  result.header.stamp    = node->now();
  result.header.frame_id = costmap_ros_->getBaseFrameID();
  result.twist           = safe_cmd;
  return result;
}

void VFRobotController::pointcloudCallback(
  const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(pcl_mutex_);
  latest_pointcloud_ = msg;
}

}  // namespace vf_robot_controller

PLUGINLIB_EXPORT_CLASS(
  vf_robot_controller::VFRobotController,
  nav2_core::Controller)
