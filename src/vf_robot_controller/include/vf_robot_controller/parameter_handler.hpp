#pragma once

#include <string>
#include <vector>
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

namespace vf_robot_controller
{

/**
 * @brief All runtime parameters for vf_robot_controller in one place.
 *
 * Uses nav2_util::declare_parameter_if_not_declared so the controller can
 * coexist with other plugins without namespace collisions.
 * All parameters are under <plugin_name>.<param_name>.
 */
struct Parameters
{
  // ── Optimizer ─────────────────────────────────────────────────────────────
  int    num_samples{500};          // MPPI trajectory samples
  int    num_iterations{3};         // MPPI update iterations per cycle
  double horizon_time{2.0};         // seconds
  double control_frequency{20.0};   // Hz
  double temperature{0.3};          // MPPI temperature λ

  // ── B-spline trajectory ───────────────────────────────────────────────────
  int    spline_degree{3};                  // cubic = C2
  int    min_knots{6};                      // minimum control points
  int    max_knots{20};                     // maximum control points (complex regions)
  double knot_density_scale{1.0};           // scale GCF → extra knots

  // ── GCF (Geometric Complexity Field) ─────────────────────────────────────
  double gcf_radius{1.5};                   // local query radius (m)
  double gcf_weight_2d{0.4};               // weight for 2D clearance component
  double gcf_weight_clutter{0.3};          // weight for clutter density component
  double gcf_weight_volumetric{0.3};       // weight for 3D clearance component
  double gcf_clutter_radius{0.6};          // radius to count clutter clusters
  double gcf_update_rate{10.0};            // Hz — GCF is expensive, update sub-cycle

  // ── Robot footprint ───────────────────────────────────────────────────────
  double robot_length{0.6};         // m
  double robot_width{0.4};          // m
  double robot_height_min{0.05};    // m — lowest point of robot body
  double robot_height_max{1.2};     // m — highest point of robot body
  double safety_inflation{0.05};    // m — hard safety margin beyond footprint

  // ── Velocity scheduling ───────────────────────────────────────────────────
  double max_vel_x{0.5};            // m/s open space
  double min_vel_x{0.05};           // m/s minimum forward
  double max_vel_theta{1.0};        // rad/s
  double min_vel_theta{0.0};
  double max_speed_in_tight{0.15};  // m/s forced when GCF > tight_threshold
  double tight_gcf_threshold{0.6};  // GCF value that triggers tight-mode
  double acc_lim_x{0.5};
  double acc_lim_theta{1.5};

  // ── Critics ───────────────────────────────────────────────────────────────
  // Slot order (0-9) must match critics.xml and meta-critic network output:
  //   0 ObstacleCritic      1 VolumetricCritic     2 DynamicObstacleCritic
  //   3 PathFollowCritic    4 SmoothnessCritic     5 GoalCritic
  //   6 VelocityCritic      7 CorridorCritic       8 ClearanceCritic
  //   9 OscillationCritic
  std::vector<std::string> critics{
    "ObstacleCritic",
    "VolumetricCritic",
    "DynamicObstacleCritic",
    "PathFollowCritic",
    "SmoothnessCritic",
    "GoalCritic",
    "VelocityCritic",
    "CorridorCritic",
    "ClearanceCritic",
    "OscillationCritic"
  };

  // Dynamic critic weight adaptation (driven by GCF)
  double obstacle_weight_open{1.0};
  double obstacle_weight_tight{3.0};
  double path_follow_weight_open{2.0};
  double path_follow_weight_tight{0.5};

  // ── Noise for sampling ────────────────────────────────────────────────────
  double noise_sigma_x{0.1};
  double noise_sigma_theta{0.2};

  // ── Visualization ─────────────────────────────────────────────────────────
  bool visualize_trajectories{true};
  bool visualize_gcf{true};
  int  visualize_max_trajectories{50};

  // ── Pointcloud topic for 3D data ──────────────────────────────────────────
  std::string pointcloud_topic{"/scan_3d"};
  std::string controller_mode{"fixed"};     // fixed | collect | inference
  bool use_3d_clearance{true};
};

class ParameterHandler
{
public:
  ParameterHandler(
    rclcpp_lifecycle::LifecycleNode::SharedPtr node,
    const std::string & plugin_name,
    const std::string & costmap_frame,
    double costmap_size_x,
    double costmap_size_y);

  Parameters & getParams() { return params_; }
  const Parameters & getParams() const { return params_; }

private:
  void declareAndGet(
    rclcpp_lifecycle::LifecycleNode::SharedPtr node,
    const std::string & prefix);

  rcl_interfaces::msg::SetParametersResult dynamicParametersCallback(
    std::vector<rclcpp::Parameter> params);

  Parameters params_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_cb_handle_;
  std::string plugin_name_;
};

}  // namespace vf_robot_controller