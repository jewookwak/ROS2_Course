#ifndef ROS2_TERM_PROJECT_CMD_PUBLISHER_H
#define ROS2_TERM_PROJECT_CMD_PUBLISHER_H

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "map.h"
#include <vector>
#include <queue>
#include <map>
#include <cmath>
#include <algorithm>

struct PathNode {
    double x, y;
    double g_cost;
    double h_cost;
    double f_cost;
    std::shared_ptr<PathNode> parent;

    PathNode(double _x, double _y) : x(_x), y(_y), 
        g_cost(0), h_cost(0), f_cost(0), parent(nullptr) {}
};

class CmdPublisher : public rclcpp::Node {
public:
    CmdPublisher();

private:
    // PID Controller structure
    struct PIDController {
        double p_gain, i_gain, d_gain;
        double prev_error;
        double integral;
        double dt;

        PIDController() : p_gain(0), i_gain(0), d_gain(0), 
                         prev_error(0), integral(0), dt(0.05) {}
        
        double compute(double error) {
            integral += error * dt;
            double derivative = (error - prev_error) / dt;
            prev_error = error;
            return p_gain * error + i_gain * integral + d_gain * derivative;
        }

        void reset() {
            prev_error = 0;
            integral = 0;
        }
    };

    // Member functions
    void init_pid_controllers();
    void timer_tf_callback();
    void timer_cmd_callback();
    void octomap_callback(const OctomapMsg& octomap_msg);
    void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void goal_callback(const std::shared_ptr<geometry_msgs::msg::PoseStamped> goal);
    void path_planning_callback();
    void visualize_path();

    // Path planning functions
    bool plan_path();
    std::vector<std::pair<double, double>> find_path_astar(
        std::pair<double, double> start,
        std::pair<double, double> goal
    );
    double heuristic(double x1, double y1, double x2, double y2) {
        return std::abs(x2 - x1) + std::abs(y2 - y1);
    }
    std::vector<std::shared_ptr<PathNode>> get_neighbors(
        const std::shared_ptr<PathNode>& node);

    // Member variables
    PIDController linear_pid;
    PIDController angular_pid;
    double yaw;
    double x, y, z;
    bool position_updated = false;
    bool has_goal = false;
    Map map;

    // Path tracking variables
    std::vector<std::pair<double, double>> path;
    size_t current_path_index = 0;
    geometry_msgs::msg::PoseStamped goal_pose;

    // ROS2 members
    rclcpp::TimerBase::SharedPtr timer_cmd, timer_tf, timer_path;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path;
    rclcpp::Subscription<OctomapMsg>::SharedPtr sub_octomap;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_map;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_goal;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer;
};

#endif //ROS2_TERM_PROJECT_CMD_PUBLISHER_H