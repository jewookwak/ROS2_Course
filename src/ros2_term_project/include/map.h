#ifndef ROS2_TERM_PROJECT_MAP_H
#define ROS2_TERM_PROJECT_MAP_H

#include <dynamicEDT3D/dynamicEDTOctomap.h>
#include <octomap/ColorOcTree.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/msg/octomap.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <memory>
#include <limits>

typedef std::shared_ptr<octomap::OcTree> Octree_ptr;
typedef DynamicEDTOctomapBase<octomap::OcTree> DynamicEDTOctomap;
typedef std::shared_ptr<DynamicEDTOctomap> DynamicEDTMapPtr;
typedef octomap_msgs::msg::Octomap OctomapMsg;

class Map {
    friend class CmdPublisher;  // Allow CmdPublisher to access private members
public:
    Map() = default;

    // OctoMap related functions
    void update_octomap(const OctomapMsg& octomap_msg,
                       const octomap::point3d& world_min,
                       const octomap::point3d& world_max);
    
    void get_distance_and_closest_obstacle(const octomap::point3d& search_point,
                                         float& distance,
                                         octomap::point3d& closest_obstacle) const;
    
    bool is_point_known_octomap(const octomap::point3d& point) const;

    // Grid map (Cartographer) related functions
    void update_gridmap(const nav_msgs::msg::OccupancyGrid::SharedPtr map_msg);
    bool is_point_valid(double x, double y) const;
    double get_distance_to_obstacle(double x, double y) const;
    bool is_updated() const;

private:
    // OctoMap members
    bool octomap_updated = false;
    double world_maxdist = 1.0;
    Octree_ptr octree_ptr = nullptr;
    DynamicEDTMapPtr dynamic_edt_map_ptr = nullptr;

    // Grid map members
    bool gridmap_updated = false;
    nav_msgs::msg::OccupancyGrid::SharedPtr grid_map_data;
    double resolution;
    uint32_t width;
    uint32_t height;
    double origin_x;
    double origin_y;
};

#endif //ROS2_TERM_PROJECT_MAP_H