#include "vf_robot_controller/tools/trajectory_visualizer.hpp"
#include <algorithm>
#include <cmath>

namespace vf_robot_controller::tools
{

TrajectoryVisualizer::TrajectoryVisualizer(
  rclcpp_lifecycle::LifecycleNode::SharedPtr node,
  const Parameters & params)
: node_(node), params_(params)
{}

void TrajectoryVisualizer::activate()
{
  sampled_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
    "vf_controller/sampled_trajectories",
    rclcpp::QoS(1).durability_volatile());

  best_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
    "vf_controller/best_trajectory",
    rclcpp::QoS(1).durability_volatile());

  gcf_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
    "vf_controller/gcf_field",
    rclcpp::QoS(1).durability_volatile());

  vel_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
    "vf_controller/velocity_profile",
    rclcpp::QoS(1).durability_volatile());

  safety_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
    "vf_controller/safety_veto",
    rclcpp::QoS(1).durability_volatile());
}

void TrajectoryVisualizer::deactivate()
{
  sampled_pub_.reset();
  best_pub_.reset();
  gcf_pub_.reset();
  vel_pub_.reset();
  safety_pub_.reset();
}

void TrajectoryVisualizer::publishSampledTrajectories(
  const std::vector<models::BSplineTrajectory> & trajectories,
  const std::vector<double> & costs,
  const std::string & frame_id)
{
  if (!sampled_pub_ || trajectories.empty()) return;

  const double min_cost = costs.empty() ? 0.0 : *std::min_element(costs.begin(), costs.end());
  const double max_cost = costs.empty() ? 1.0 : *std::max_element(costs.begin(), costs.end());
  const double range    = std::max(max_cost - min_cost, 1e-6);

  visualization_msgs::msg::MarkerArray ma;
  const int n_show = std::min(
    static_cast<int>(trajectories.size()),
    params_.visualize_max_trajectories);

  for (int s = 0; s < n_show; ++s) {
    const double t_norm = (s < static_cast<int>(costs.size()))
      ? (costs[s] - min_cost) / range : 0.5;

    visualization_msgs::msg::Marker m;
    m.header.frame_id = frame_id;
    m.header.stamp    = node_->now();
    m.ns              = "sampled";
    m.id              = s;
    m.type            = visualization_msgs::msg::Marker::LINE_STRIP;
    m.action          = visualization_msgs::msg::Marker::ADD;
    m.scale.x         = 0.02;
    m.color.a         = 0.4f;
    m.color.r         = static_cast<float>(t_norm);        // red = bad
    m.color.g         = static_cast<float>(1.0 - t_norm);  // green = good
    m.color.b         = 0.0f;

    const auto pts = trajectories[s].sample(15);
    for (const auto & pt : pts) {
      geometry_msgs::msg::Point p;
      p.x = pt.x(); p.y = pt.y(); p.z = 0.05;
      m.points.push_back(p);
    }
    ma.markers.push_back(m);
  }
  sampled_pub_->publish(ma);
}

void TrajectoryVisualizer::publishBestTrajectory(
  const models::BSplineTrajectory & best,
  const std::string & frame_id)
{
  if (!best_pub_) return;

  visualization_msgs::msg::MarkerArray ma;
  visualization_msgs::msg::Marker m;
  m.header.frame_id = frame_id;
  m.header.stamp    = node_->now();
  m.ns              = "best";
  m.id              = 0;
  m.type            = visualization_msgs::msg::Marker::LINE_STRIP;
  m.action          = visualization_msgs::msg::Marker::ADD;
  m.scale.x         = 0.06;
  m.color.a         = 1.0f;
  m.color.r         = 0.0f;
  m.color.g         = 0.4f;
  m.color.b         = 1.0f;

  const auto pts = best.sample(30);
  for (const auto & pt : pts) {
    geometry_msgs::msg::Point p;
    p.x = pt.x(); p.y = pt.y(); p.z = 0.08;
    m.points.push_back(p);
  }
  ma.markers.push_back(m);
  best_pub_->publish(ma);
}

void TrajectoryVisualizer::publishGCFField(
  const std::shared_ptr<gcf::GeometricComplexityField> & gcf,
  const std::string & frame_id)
{
  if (!gcf_pub_) return;

  visualization_msgs::msg::MarkerArray ma;
  const auto & grid = gcf->getGrid();
  const double res  = gcf->getResolution();
  const double ox   = gcf->getOriginX();
  const double oy   = gcf->getOriginY();
  const int    W    = gcf->getWidth();
  const int    H    = gcf->getHeight();

  // Downsample for performance — show every 4th cell
  const int step = 4;
  int id = 0;
  for (int y = 0; y < H; y += step) {
    for (int x = 0; x < W; x += step) {
      const double c = grid[y * W + x].complexity;
      if (c < 0.1) continue;  // skip near-zero cells

      visualization_msgs::msg::Marker m;
      m.header.frame_id = frame_id;
      m.header.stamp    = node_->now();
      m.ns              = "gcf";
      m.id              = id++;
      m.type            = visualization_msgs::msg::Marker::CUBE;
      m.action          = visualization_msgs::msg::Marker::ADD;
      m.pose.position.x = ox + (x + 0.5) * res;
      m.pose.position.y = oy + (y + 0.5) * res;
      m.pose.position.z = -0.05;
      m.pose.orientation.w = 1.0;
      m.scale.x = res * step * 0.9;
      m.scale.y = res * step * 0.9;
      m.scale.z = 0.01;
      m.color.a = static_cast<float>(c * 0.6);
      m.color.r = static_cast<float>(c);
      m.color.g = static_cast<float>(1.0 - c) * 0.3f;
      m.color.b = 0.0f;
      ma.markers.push_back(m);
    }
  }
  gcf_pub_->publish(ma);
}

void TrajectoryVisualizer::publishSafetyShellVeto(
  double robot_x, double robot_y, double robot_yaw,
  const std::string & frame_id)
{
  if (!safety_pub_) return;

  visualization_msgs::msg::MarkerArray ma;
  visualization_msgs::msg::Marker m;
  m.header.frame_id    = frame_id;
  m.header.stamp       = node_->now();
  m.ns                 = "safety_veto";
  m.id                 = 0;
  m.type               = visualization_msgs::msg::Marker::CUBE;
  m.action             = visualization_msgs::msg::Marker::ADD;
  m.pose.position.x    = robot_x;
  m.pose.position.y    = robot_y;
  m.pose.position.z    = 0.1;
  m.pose.orientation.z = std::sin(robot_yaw * 0.5);
  m.pose.orientation.w = std::cos(robot_yaw * 0.5);
  m.scale.x = 0.6;   // robot_length approx
  m.scale.y = 0.4;   // robot_width approx
  m.scale.z = 0.05;
  m.color.a = 0.7f;
  m.color.r = 1.0f;
  m.color.g = 0.0f;
  m.color.b = 0.0f;
  m.lifetime = rclcpp::Duration::from_seconds(0.5);
  ma.markers.push_back(m);
  safety_pub_->publish(ma);
}

void TrajectoryVisualizer::publishVelocityProfile(
  const std::vector<double> & v_profile,
  const models::BSplineTrajectory & traj,
  const std::string & frame_id)
{
  if (!vel_pub_ || v_profile.empty()) return;
  const int n = static_cast<int>(v_profile.size());
  const auto pts = traj.sample(n);

  visualization_msgs::msg::MarkerArray ma;
  visualization_msgs::msg::Marker m;
  m.header.frame_id = frame_id;
  m.header.stamp    = node_->now();
  m.ns    = "vel_profile";
  m.id    = 0;
  m.type  = visualization_msgs::msg::Marker::LINE_STRIP;
  m.action = visualization_msgs::msg::Marker::ADD;
  m.scale.x = 0.04;
  m.color.a = 1.0f; m.color.r = 1.0f; m.color.g = 0.7f; m.color.b = 0.0f;

  for (int i = 0; i < n && i < static_cast<int>(pts.size()); ++i) {
    geometry_msgs::msg::Point p;
    p.x = pts[i].x(); p.y = pts[i].y();
    p.z = v_profile[i];  // height encodes speed
    m.points.push_back(p);
  }
  ma.markers.push_back(m);
  vel_pub_->publish(ma);
}

}  // namespace vf_robot_controller::tools
