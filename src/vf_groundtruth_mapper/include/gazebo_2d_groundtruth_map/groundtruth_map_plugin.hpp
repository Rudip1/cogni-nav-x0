#ifndef GAZEBO_2D_GROUNDTRUTH_MAP_PLUGIN_HPP
#define GAZEBO_2D_GROUNDTRUTH_MAP_PLUGIN_HPP

#include "gazebo_2d_groundtruth_map/common.hpp"

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Vector3.hh>

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/empty.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <memory>
#include <string>
#include <thread>
#include <limits>

namespace gazebo
{

#if GAZEBO_MAJOR_VERSION >= 9
using vector3d = ignition::math::Vector3d;
#else
using vector3d = math::Vector3;
#endif

class OccupancyMapFromWorld : public WorldPlugin
{
public:
    OccupancyMapFromWorld();
    ~OccupancyMapFromWorld() override;

    void Load(physics::WorldPtr world, sdf::ElementPtr sdf) override;

private:
    // ROS2 node and executor
    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
    std::thread executor_thread_;

    // ROS2 interfaces
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr map_service_;

    // Gazebo world
    physics::WorldPtr world_;

    // Plugin name
    std::string name_;

    // Map parameters
    double map_resolution_;
    double map_size_x_;
    double map_size_y_;
    double map_min_x_;
    double map_min_y_;

    // Occupancy constants
    static constexpr int OCCUPIED = 100;
    static constexpr int FREE = 0;
    static constexpr int UNKNOWN = -1;

    // Map functions
    void CreateOccupancyMap();
    void cell2world(unsigned int cell_x, unsigned int cell_y, double &world_x, double &world_y);
    void world2cell(double world_x, double world_y, unsigned int &cell_x, unsigned int &cell_y);

    // Ray intersection
    bool worldCellIntersection(const vector3d &cell_center, double cell_length, physics::RayShapePtr ray);

    // Dynamic Z computation per XY cell
    void computeZBounds(const double x, const double y, double &z_min, double &z_max);

    // Automatic ROI computation based on objects
    void computeObjectBounds(double &x_min, double &x_max, double &y_min, double &y_max);

    // Service callback
    bool ServiceCallback(const std::shared_ptr<std_srvs::srv::Empty::Request>,
                         std::shared_ptr<std_srvs::srv::Empty::Response>);
};

} // namespace gazebo

#endif // GAZEBO_2D_GROUNDTRUTH_MAP_PLUGIN_HPP