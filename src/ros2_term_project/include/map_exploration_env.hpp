#ifndef MAP_EXPLORATION_ENV_HPP_
#define MAP_EXPLORATION_ENV_HPP_

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float32.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "robot_state_msgs/msg/robot_state.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2/exceptions.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <octomap/OcTree.h>
#include <cmath>
#include <limits>
#include <vector>

namespace exploration {

class RobotEnvironment : public rclcpp::Node {
public:
    RobotEnvironment();

private:
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void updateState();
    float getMinObstacleDistance(float x, float y);
    void getLocalMap(float center_x, float center_y, int size, std::vector<int8_t>& local_map);
    float total_explored_radius_; 

    // Publishers
    rclcpp::Publisher<robot_state_msgs::msg::RobotState>::SharedPtr state_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr reward_pub_;
    
    // Subscribers
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    
    // TF
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    
    // Timer
    rclcpp::TimerBase::SharedPtr timer_;

    // State variables
    struct {
        double x, y, yaw;
        std::vector<int8_t> grid_map;
        uint32_t map_width, map_height;
    } current_state_;

    // Additional state tracking
    int previous_known_cells_;
    double current_linear_vel_;
    double current_angular_vel_;
    double map_resolution_;
    double map_origin_x_;
    double map_origin_y_;
};

} // namespace exploration

#endif // MAP_EXPLORATION_ENV_HPP_