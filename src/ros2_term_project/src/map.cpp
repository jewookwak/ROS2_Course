#include "map.h"

// OctoMap related functions
void Map::update_octomap(const OctomapMsg& octomap_msg,
                        const octomap::point3d& world_min,
                        const octomap::point3d& world_max) {
    // Octomap Msg to Octree conversion
    octree_ptr.reset(dynamic_cast<octomap::OcTree*>(octomap_msgs::fullMsgToMap(octomap_msg)));

    // Octree to DynamicEDTOctomap conversion
    dynamic_edt_map_ptr = std::make_shared<DynamicEDTOctomap>(
        world_maxdist, octree_ptr.get(), world_min, world_max, false);
    dynamic_edt_map_ptr->update();
    octomap_updated = true;
}

void Map::get_distance_and_closest_obstacle(const octomap::point3d& search_point,
                                          float& distance,
                                          octomap::point3d& closest_obstacle) const {
    if (dynamic_edt_map_ptr) {
        dynamic_edt_map_ptr->getDistanceAndClosestObstacle(search_point, distance, closest_obstacle);
    }
}

bool Map::is_point_known_octomap(const octomap::point3d& point) const {
    if (!octree_ptr) return true;

    // Safety distance considering robot size
    const float SAFETY_DISTANCE = 0.30;  // 30cm safety distance

    // Check distance to closest obstacle
    octomap::point3d closest_obstacle;
    float distance;
    get_distance_and_closest_obstacle(point, distance, closest_obstacle);

    // If too close to obstacle, point is invalid
    if (distance < SAFETY_DISTANCE) {
        return false;
    }

    octomap::OcTreeNode* node = octree_ptr->search(point);
    if (!node) return true;  // Unknown space is considered free
    
    return !octree_ptr->isNodeOccupied(node);
}

// Grid map (Cartographer) related functions
void Map::update_gridmap(const nav_msgs::msg::OccupancyGrid::SharedPtr map_msg) {
    grid_map_data = map_msg;
    resolution = map_msg->info.resolution;
    width = map_msg->info.width;
    height = map_msg->info.height;
    origin_x = map_msg->info.origin.position.x;
    origin_y = map_msg->info.origin.position.y;
    gridmap_updated = true;
}

bool Map::is_point_valid(double x, double y) const {
    if (!grid_map_data) {
        return false;
    }

    // Convert world coordinates to map coordinates
    int map_x = static_cast<int>((x - origin_x) / resolution);
    int map_y = static_cast<int>((y - origin_y) / resolution);

    // Check if point is within map bounds
    if (map_x < 0 || map_x >= static_cast<int>(width) || 
        map_y < 0 || map_y >= static_cast<int>(height)) {
        return false;
    }

    // Get occupancy value
    int index = map_y * width + map_x;
    int occupancy = grid_map_data->data[index];

    // Check nearby cells for safety margin only for known obstacles
    const int safety_cells = static_cast<int>(0.3 / resolution);  
    for (int dy = -safety_cells; dy <= safety_cells; ++dy) {
        for (int dx = -safety_cells; dx <= safety_cells; ++dx) {
            int check_x = map_x + dx;
            int check_y = map_y + dy;
            
            if (check_x >= 0 && check_x < static_cast<int>(width) && 
                check_y >= 0 && check_y < static_cast<int>(height)) {
                int check_index = check_y * width + check_x;
                // Only consider known obstacles (>=50) as invalid
                if (grid_map_data->data[check_index] >= 60) {
                    return false;  // Too close to obstacle
                }
            }
        }
    }

    // Allow unknown (-1) and free (0-49) spaces
    return occupancy < 60;
}

double Map::get_distance_to_obstacle(double x, double y) const {
    if (!grid_map_data) {
        return 0.0;
    }

    const int search_radius = 20;  // cells
    double min_distance = std::numeric_limits<double>::max();
    
    int center_x = static_cast<int>((x - origin_x) / resolution);
    int center_y = static_cast<int>((y - origin_y) / resolution);

    for (int dy = -search_radius; dy <= search_radius; dy++) {
        for (int dx = -search_radius; dx <= search_radius; dx++) {
            int map_x = center_x + dx;
            int map_y = center_y + dy;

            if (map_x < 0 || map_x >= static_cast<int>(width) || 
                map_y < 0 || map_y >= static_cast<int>(height)) {
                continue;
            }

            int index = map_y * width + map_x;
            int occupancy = grid_map_data->data[index];

            if (occupancy >= 50) {  // If obstacle
                double dx_world = (dx * resolution);
                double dy_world = (dy * resolution);
                double distance = std::sqrt(dx_world * dx_world + dy_world * dy_world);
                min_distance = std::min(min_distance, distance);
            }
        }
    }

    return min_distance;
}

bool Map::is_updated() const { 
    return gridmap_updated; 
}