#pragma once
#include "vf_robot_controller/critics/critic_function.hpp"

namespace vf_robot_controller::critics
{
/**
 * @brief Penalizes trajectories that pass through occupied 3D voxels.
 *
 * This is the critic that catches chair legs, bed frames, IV stand bases
 * that are INVISIBLE to the 2D costmap but present in 3D pointcloud data.
 * Without this critic the robot walks into chair legs.
 */
class VolumetricCritic : public CriticFunction
{
public:
  void initialize(rclcpp_lifecycle::LifecycleNode::SharedPtr n,
                  const std::string & nm, const Parameters & p) override;
  double score(const CriticData & d) const override;
private:
  double min_clearance_{0.08};  // m — minimum 3D clearance required
};
}  // namespace vf_robot_controller::critics
