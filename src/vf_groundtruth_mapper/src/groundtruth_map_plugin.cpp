#include "gazebo_2d_groundtruth_map/groundtruth_map_plugin.hpp"
#include "gazebo_2d_groundtruth_map/common.hpp"

#include <iostream>
#include <thread>
#include <memory>
#include <limits>

namespace gazebo
{

OccupancyMapFromWorld::OccupancyMapFromWorld()
{
    name_ = "groundtruth_map_plugin";
}

OccupancyMapFromWorld::~OccupancyMapFromWorld()
{
    if (executor_)
    {
        executor_->cancel();
        if (executor_thread_.joinable())
            executor_thread_.join();
    }
    if (node_)
        node_.reset();
}

void OccupancyMapFromWorld::Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
{
    world_ = _world;

    if (!rclcpp::ok())
    {
        int argc = 0;
        char **argv = nullptr;
        rclcpp::init(argc, argv);
    }

    node_ = std::make_shared<rclcpp::Node>("groundtruth_map_plugin");

    map_pub_ = node_->create_publisher<nav_msgs::msg::OccupancyGrid>("map", 1);
    map_service_ = node_->create_service<std_srvs::srv::Empty>(
        "/gazebo_2d_groundtruth_map/generate_map",
        std::bind(&OccupancyMapFromWorld::ServiceCallback, this,
                  std::placeholders::_1, std::placeholders::_2));

    // Read SDF parameters
    getSdfParam(_sdf, "map_resolution", map_resolution_, 0.05, true);

    // Compute automatic XY ROI from all objects
    double obj_x_min, obj_x_max, obj_y_min, obj_y_max;
    computeObjectBounds(obj_x_min, obj_x_max, obj_y_min, obj_y_max);

    map_min_x_ = obj_x_min;
    map_min_y_ = obj_y_min;
    map_size_x_ = obj_x_max - obj_x_min;
    map_size_y_ = obj_y_max - obj_y_min;

    RCLCPP_INFO(node_->get_logger(), "Map resolution: %.3f, size: %.2fx%.2f",
                map_resolution_, map_size_x_, map_size_y_);

    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(node_);
    executor_thread_ = std::thread([this]() { executor_->spin(); });
}

// Service callback
bool OccupancyMapFromWorld::ServiceCallback(
    const std::shared_ptr<std_srvs::srv::Empty::Request>,
    std::shared_ptr<std_srvs::srv::Empty::Response>)
{
    CreateOccupancyMap();
    return true;
}

// Convert cell to world coordinates
void OccupancyMapFromWorld::cell2world(unsigned int cell_x, unsigned int cell_y,
                                       double &world_x, double &world_y)
{
    world_x = map_min_x_ + cell_x * map_resolution_ + map_resolution_ / 2.0;
    world_y = map_min_y_ + cell_y * map_resolution_ + map_resolution_ / 2.0;
}

// Convert world to cell coordinates
void OccupancyMapFromWorld::world2cell(double world_x, double world_y,
                                       unsigned int &cell_x, unsigned int &cell_y)
{
    cell_x = static_cast<unsigned int>((world_x - map_min_x_) / map_resolution_);
    cell_y = static_cast<unsigned int>((world_y - map_min_y_) / map_resolution_);
}

// Compute dynamic Z for XY cell
void OccupancyMapFromWorld::computeZBounds(const double x, const double y,
                                           double &z_min, double &z_max)
{
    z_min = std::numeric_limits<double>::max();
    z_max = std::numeric_limits<double>::lowest();

    for (auto model : world_->Models())
    {
        for (auto link : model->GetLinks())
        {
            for (auto collision : link->GetCollisions())
            {
                auto bb = collision->BoundingBox();
                if (x >= bb.Min().X() && x <= bb.Max().X() &&
                    y >= bb.Min().Y() && y <= bb.Max().Y())
                {
                    z_min = std::min(z_min, bb.Min().Z());
                    z_max = std::max(z_max, bb.Max().Z());
                }
            }
        }
    }

    if (z_min == std::numeric_limits<double>::max())
        z_min = 0.0;
    if (z_max == std::numeric_limits<double>::lowest())
        z_max = 1.0;
}

// Ray intersection for occupancy
bool OccupancyMapFromWorld::worldCellIntersection(
    const vector3d &cell_center,
    double /*cell_length*/,
    physics::RayShapePtr ray)
{
    double z_min, z_max;
    computeZBounds(cell_center.X(), cell_center.Y(), z_min, z_max);

    double dist;
    std::string entity;

    ray->SetPoints(
        vector3d(cell_center.X(), cell_center.Y(), z_max + 0.05),
        vector3d(cell_center.X(), cell_center.Y(), z_min - 0.05));

    ray->GetIntersection(dist, entity);

    return !entity.empty();
}

// Create occupancy map
void OccupancyMapFromWorld::CreateOccupancyMap()
{
    unsigned int cells_x = static_cast<unsigned int>(map_size_x_ / map_resolution_);
    unsigned int cells_y = static_cast<unsigned int>(map_size_y_ / map_resolution_);

    auto map = std::make_shared<nav_msgs::msg::OccupancyGrid>();
    map->header.frame_id = "map";
    map->header.stamp = node_->get_clock()->now();

    map->info.resolution = map_resolution_;
    map->info.width = cells_x;
    map->info.height = cells_y;

    map->info.origin.position.x = map_min_x_;
    map->info.origin.position.y = map_min_y_;
    map->info.origin.position.z = 0.0; // dynamic Z handled per ray
    map->info.origin.orientation.w = 1.0;

    map->data.resize(cells_x * cells_y, UNKNOWN);

    auto physics_engine = world_->Physics();
    physics_engine->InitForThread();

    auto ray = boost::dynamic_pointer_cast<physics::RayShape>(
        physics_engine->CreateShape("ray", physics::CollisionPtr()));

    RCLCPP_INFO(node_->get_logger(), "Generating occupancy map: %ux%u", cells_x, cells_y);

    for (unsigned int y = 0; y < cells_y; ++y)
    {
        for (unsigned int x = 0; x < cells_x; ++x)
        {
            unsigned int index = y * cells_x + x;
            double wx, wy;
            cell2world(x, y, wx, wy);

            if (worldCellIntersection(vector3d(wx, wy, 0.0), map_resolution_, ray))
                map->data[index] = OCCUPIED;
            else
                map->data[index] = FREE;
        }
    }

    map_pub_->publish(*map);
    RCLCPP_INFO(node_->get_logger(), "Map generated and published.");
}

// Compute ROI automatically based on object bounding boxes
void OccupancyMapFromWorld::computeObjectBounds(double &x_min, double &x_max,
                                                double &y_min, double &y_max)
{
    x_min = std::numeric_limits<double>::max();
    x_max = std::numeric_limits<double>::lowest();
    y_min = std::numeric_limits<double>::max();
    y_max = std::numeric_limits<double>::lowest();

    for (auto model : world_->Models())
    {
        for (auto link : model->GetLinks())
        {
            for (auto collision : link->GetCollisions())
            {
                auto bb = collision->BoundingBox();
                x_min = std::min(x_min, bb.Min().X());
                x_max = std::max(x_max, bb.Max().X());
                y_min = std::min(y_min, bb.Min().Y());
                y_max = std::max(y_max, bb.Max().Y());
            }
        }
    }

    // Fallback if no models found
    if (x_min == std::numeric_limits<double>::max()) x_min = -5.0;
    if (x_max == std::numeric_limits<double>::lowest()) x_max = 5.0;
    if (y_min == std::numeric_limits<double>::max()) y_min = -5.0;
    if (y_max == std::numeric_limits<double>::lowest()) y_max = 5.0;
}

GZ_REGISTER_WORLD_PLUGIN(OccupancyMapFromWorld)

} // namespace gazebo