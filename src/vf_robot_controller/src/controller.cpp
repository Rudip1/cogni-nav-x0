#include "vf_robot_controller/controller.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace vf_robot_controller
{

ControllerMode VFRobotController::modeFromString(const std::string & s)
{
  if (s == "collect")   return ControllerMode::COLLECT;
  if (s == "inference") return ControllerMode::INFERENCE;
  if (s == "passive")   return ControllerMode::PASSIVE;
  return ControllerMode::FIXED;
}

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

  // ── Operating mode ────────────────────────────────────────────────────────
  mode_ = modeFromString(p.controller_mode);
  const char * mode_str[] = {"FIXED", "COLLECT", "INFERENCE", "PASSIVE"};
  RCLCPP_INFO(logger_, "Controller mode: %s",
    mode_str[static_cast<int>(mode_)]);

  // ── GCF ───────────────────────────────────────────────────────────────────
  gcf_ = std::make_shared<gcf::GeometricComplexityField>(p);

  // ── Optimizer ─────────────────────────────────────────────────────────────
  optimizer_ = std::make_unique<Optimizer>();
  optimizer_->initialize(node, p, gcf_);

  // ── Hard safety shell ─────────────────────────────────────────────────────
  safety_shell_ = std::make_unique<safety::HardSafetyShell>(p);

  // ── Visualizer ────────────────────────────────────────────────────────────
  visualizer_ = std::make_unique<tools::TrajectoryVisualizer>(node, p);

  // ── WeightAdapter (INFERENCE mode) ────────────────────────────────────────
  if (mode_ == ControllerMode::INFERENCE) {
    weight_adapter_ = std::make_unique<tools::WeightAdapter>(
      node,
      static_cast<int>(p.critics.size()),
      "/vf_controller/meta_weights",
      200.0);
    RCLCPP_INFO(logger_, "WeightAdapter created — listening for meta-critic weights");
  }

  // ── DataRecorder (COLLECT mode) ───────────────────────────────────────────
  if (mode_ == ControllerMode::COLLECT) {
    data_recorder_ = std::make_unique<tools::DataRecorder>(
      node, "/vf_controller/critic_data");
    data_recorder_->setEnabled(true);
    RCLCPP_INFO(logger_, "DataRecorder created — publishing critic data for training");
  }

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
  weight_adapter_.reset();
  data_recorder_.reset();
  pcl_sub_.reset();
}

void VFRobotController::setPlan(const nav_msgs::msg::Path & path)
{
  global_plan_ = path;
}

void VFRobotController::setSpeedLimit(
  const double & speed_limit, const bool & percentage)
{
  speed_limit_               = speed_limit;
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

  // ── Snapshot pointcloud thread-safely ────────────────────────────────────
  sensor_msgs::msg::PointCloud2::SharedPtr pcl_snap;
  {
    std::lock_guard<std::mutex> lock(pcl_mutex_);
    pcl_snap = latest_pointcloud_;
  }

  // ── Update GCF ────────────────────────────────────────────────────────────
  const double yaw = tools::poseYaw(pose);
  gcf_->update(costmap, pcl_snap,
    pose.pose.position.x, pose.pose.position.y, yaw);

  // ── PASSIVE mode — return zero twist, Python node owns /cmd_vel ───────────
  if (mode_ == ControllerMode::PASSIVE) {
    geometry_msgs::msg::TwistStamped zero;
    zero.header.stamp    = node->now();
    zero.header.frame_id = costmap_ros_->getBaseFrameID();
    return zero;
  }
  // ── Push dynamic weights into optimizer (INFERENCE mode) ──────────────────
  if (mode_ == ControllerMode::INFERENCE && weight_adapter_) {
    optimizer_->setDynamicWeights(weight_adapter_->getWeights());
  }

  // ── Run optimizer ─────────────────────────────────────────────────────────
  geometry_msgs::msg::Twist raw_cmd = optimizer_->optimize(
    pose, velocity, global_plan_, costmap, pcl_snap);

  // Apply external speed limit
  if (speed_limit_is_percentage_) {
    raw_cmd.linear.x  *= speed_limit_;
    raw_cmd.angular.z *= speed_limit_;
  } else {
    raw_cmd.linear.x = std::min(raw_cmd.linear.x, speed_limit_);
  }

  // ── Hard safety shell — DECOUPLED veto ───────────────────────────────────
  geometry_msgs::msg::Twist safe_cmd = safety_shell_->check(
    raw_cmd, pose, costmap, pcl_snap);

  if (safety_shell_->lastCheckVetoed()) {
    RCLCPP_WARN(logger_, "HardSafetyShell VETO — consecutive: %d",
      safety_shell_->consecutiveVetoes());
    if (p.visualize_trajectories) {
      visualizer_->publishSafetyShellVeto(
        pose.pose.position.x, pose.pose.position.y, yaw,
        costmap_ros_->getGlobalFrameID());
    }
  }

  // ── DataRecorder (COLLECT mode) ───────────────────────────────────────────
  if (mode_ == ControllerMode::COLLECT && data_recorder_) {
    // goal distance and heading — use last pose of global plan
    double goal_dist = 0.0, goal_heading = 0.0;
    if (!global_plan_.poses.empty()) {
      const auto & gp = global_plan_.poses.back().pose.position;
      const double dx = gp.x - pose.pose.position.x;
      const double dy = gp.y - pose.pose.position.y;
      goal_dist    = std::hypot(dx, dy);
      goal_heading = std::atan2(dy, dx) - yaw;
    }

    data_recorder_->record(
      pose, velocity,
      goal_dist, goal_heading,
      optimizer_->perCriticScores(),          // N*K matrix — real per-critic scores
      p.num_samples,
      static_cast<int>(p.critics.size()),
      optimizer_->bestTrajectoryIdx());        // real argmin, not placeholder 0
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
