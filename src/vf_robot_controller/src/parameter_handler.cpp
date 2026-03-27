#include "vf_robot_controller/parameter_handler.hpp"
#include "nav2_util/node_utils.hpp"

namespace vf_robot_controller
{

// Helper macro to shorten declare+get pattern
#define DECLARE_GET(node, prefix, name, default_val) \
  nav2_util::declare_parameter_if_not_declared( \
    node, prefix + "." + #name, rclcpp::ParameterValue(default_val)); \
  params_.name = node->get_parameter(prefix + "." + #name).get_value<decltype(params_.name)>()

ParameterHandler::ParameterHandler(
  rclcpp_lifecycle::LifecycleNode::SharedPtr node,
  const std::string & plugin_name,
  const std::string & /*costmap_frame*/,
  double /*costmap_size_x*/,
  double /*costmap_size_y*/)
: plugin_name_(plugin_name)
{
  declareAndGet(node, plugin_name);

  dyn_cb_handle_ = node->add_on_set_parameters_callback(
    std::bind(&ParameterHandler::dynamicParametersCallback,
              this, std::placeholders::_1));
}

void ParameterHandler::declareAndGet(
  rclcpp_lifecycle::LifecycleNode::SharedPtr node,
  const std::string & prefix)
{
  // Optimizer
  DECLARE_GET(node, prefix, num_samples,       500);
  DECLARE_GET(node, prefix, num_iterations,    3);
  DECLARE_GET(node, prefix, horizon_time,      2.0);
  DECLARE_GET(node, prefix, control_frequency, 20.0);
  DECLARE_GET(node, prefix, temperature,       0.3);

  // B-spline
  DECLARE_GET(node, prefix, spline_degree,      3);
  DECLARE_GET(node, prefix, min_knots,          6);
  DECLARE_GET(node, prefix, max_knots,          20);
  DECLARE_GET(node, prefix, knot_density_scale, 1.0);

  // GCF
  DECLARE_GET(node, prefix, gcf_radius,           1.5);
  DECLARE_GET(node, prefix, gcf_weight_2d,        0.4);
  DECLARE_GET(node, prefix, gcf_weight_clutter,   0.3);
  DECLARE_GET(node, prefix, gcf_weight_volumetric,0.3);
  DECLARE_GET(node, prefix, gcf_clutter_radius,   0.6);
  DECLARE_GET(node, prefix, gcf_update_rate,      10.0);

  // Robot footprint
  DECLARE_GET(node, prefix, robot_length,     0.6);
  DECLARE_GET(node, prefix, robot_width,      0.4);
  DECLARE_GET(node, prefix, robot_height_min, 0.05);
  DECLARE_GET(node, prefix, robot_height_max, 1.2);
  DECLARE_GET(node, prefix, safety_inflation, 0.05);

  // Velocity
  DECLARE_GET(node, prefix, max_vel_x,          0.5);
  DECLARE_GET(node, prefix, min_vel_x,          0.05);
  DECLARE_GET(node, prefix, max_vel_theta,      1.0);
  DECLARE_GET(node, prefix, min_vel_theta,      0.0);
  DECLARE_GET(node, prefix, max_speed_in_tight, 0.15);
  DECLARE_GET(node, prefix, tight_gcf_threshold,0.6);
  DECLARE_GET(node, prefix, acc_lim_x,          0.5);
  DECLARE_GET(node, prefix, acc_lim_theta,      1.5);

  // Noise
  DECLARE_GET(node, prefix, noise_sigma_x,     0.1);
  DECLARE_GET(node, prefix, noise_sigma_theta, 0.2);

  // Critic weights
  DECLARE_GET(node, prefix, obstacle_weight_open,     1.0);
  DECLARE_GET(node, prefix, obstacle_weight_tight,    3.0);
  DECLARE_GET(node, prefix, path_follow_weight_open,  2.0);
  DECLARE_GET(node, prefix, path_follow_weight_tight, 0.5);

  // Visualization
  DECLARE_GET(node, prefix, visualize_trajectories,   true);
  DECLARE_GET(node, prefix, visualize_gcf,            true);
  DECLARE_GET(node, prefix, visualize_max_trajectories, 50);

  // 3D
  DECLARE_GET(node, prefix, pointcloud_topic, std::string("/scan_3d"));
  DECLARE_GET(node, prefix, use_3d_clearance, true);
}

rcl_interfaces::msg::SetParametersResult
ParameterHandler::dynamicParametersCallback(
  std::vector<rclcpp::Parameter> params)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  for (const auto & param : params) {
    const auto & name = param.get_name();

    // Only handle params belonging to this plugin
    if (name.find(plugin_name_ + ".") != 0) continue;

    // Velocity limits — dynamically tunable
    if (name == plugin_name_ + ".max_vel_x") {
      params_.max_vel_x = param.as_double();
    } else if (name == plugin_name_ + ".max_speed_in_tight") {
      params_.max_speed_in_tight = param.as_double();
    } else if (name == plugin_name_ + ".max_vel_theta") {
      params_.max_vel_theta = param.as_double();
    } else if (name == plugin_name_ + ".visualize_trajectories") {
      params_.visualize_trajectories = param.as_bool();
    } else if (name == plugin_name_ + ".visualize_gcf") {
      params_.visualize_gcf = param.as_bool();
    } else if (name == plugin_name_ + ".temperature") {
      params_.temperature = param.as_double();
    } else if (name == plugin_name_ + ".num_samples") {
      params_.num_samples = static_cast<int>(param.as_int());
    }
  }

  return result;
}

}  // namespace vf_robot_controller
