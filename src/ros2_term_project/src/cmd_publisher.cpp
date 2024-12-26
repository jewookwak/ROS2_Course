#include "cmd_publisher.h"

using namespace std::chrono_literals;
using std::placeholders::_1;

CmdPublisher::CmdPublisher() : Node("cmd_publisher") {
    // Publisher
    pub_cmd = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    
    // Subscribers
    sub_map = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map", 10, std::bind(&CmdPublisher::map_callback, this, _1));
        
    sub_octomap = this->create_subscription<OctomapMsg>(
        "octomap_full", 10, std::bind(&CmdPublisher::octomap_callback, this, _1));
    
    sub_goal = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/move_base_simple/goal", 10, std::bind(&CmdPublisher::goal_callback, this, _1));

    // TF listener
    tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

    // Timers
    timer_tf = this->create_wall_timer(
        10ms, std::bind(&CmdPublisher::timer_tf_callback, this));
    timer_cmd = this->create_wall_timer(
        500ms, std::bind(&CmdPublisher::timer_cmd_callback, this));
    timer_path = this->create_wall_timer(
        10ms, std::bind(&CmdPublisher::path_planning_callback, this));

    // Path publisher
    pub_path = this->create_publisher<nav_msgs::msg::Path>("planned_path", 10);

    init_pid_controllers();
}

void CmdPublisher::init_pid_controllers() {
    // Linear velocity PID gains
    linear_pid.p_gain = 0.5;
    linear_pid.i_gain = 0.01;
    linear_pid.d_gain = 0.1;

    // Angular velocity PID gains
    angular_pid.p_gain = 0.8;
    angular_pid.i_gain = 0.01;
    angular_pid.d_gain = 0.01;
}

void CmdPublisher::map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    map.update_gridmap(msg);
}

void CmdPublisher::octomap_callback(const OctomapMsg& octomap_msg) {
    octomap::point3d world_min(-5, -5, 0);
    octomap::point3d world_max(5, 5, 2);
    map.update_octomap(octomap_msg, world_min, world_max);
}

void CmdPublisher::timer_tf_callback() {
    geometry_msgs::msg::TransformStamped t;
    try {
        t = tf_buffer->lookupTransform("map", "base_scan", tf2::TimePointZero);
    } catch (const tf2::TransformException &ex) {
        RCLCPP_INFO_ONCE(this->get_logger(), "Could not transform map to base_scan");
        return;
    }

    x = t.transform.translation.x;
    y = t.transform.translation.y;
    z = t.transform.translation.z;
    yaw = tf2::getYaw(t.transform.rotation);

    position_updated = true;
}

void CmdPublisher::timer_cmd_callback() {
    if (!position_updated || path.empty()) {
        return;
    }

    // Find closest point on path
    size_t closest_point_idx = 0;
    double min_distance = std::numeric_limits<double>::max();
    
    for (size_t i = 0; i < path.size(); i++) {
        double dx = path[i].first - x;
        double dy = path[i].second - y;
        double distance = std::sqrt(dx*dx + dy*dy);
        
        if (distance < min_distance) {
            min_distance = distance;
            closest_point_idx = i;
        }
    }

    // Set target point (look-ahead)
    size_t target_idx;
    if (path.size() - closest_point_idx <= 2) {
        target_idx = path.size() - 1;
    } else {
        target_idx = closest_point_idx + 2;
    }
    double target_x = path[target_idx].first;
    double target_y = path[target_idx].second;

    // Calculate angle to target
    double angle_to_target = std::atan2(target_y - y, target_x - x);
    double angle_diff = angle_to_target - yaw;
    
    while (angle_diff > M_PI) angle_diff -= 2*M_PI;
    while (angle_diff < -M_PI) angle_diff += 2*M_PI;

    geometry_msgs::msg::Twist cmd_vel;
    
    if (path.size() <= 1) {
        // Final pose adjustment
        double goal_pose_z = atan2(2.0 * (goal_pose.pose.orientation.w * goal_pose.pose.orientation.z + 
                                         goal_pose.pose.orientation.x * goal_pose.pose.orientation.y),
                                 1.0 - 2.0 * (goal_pose.pose.orientation.y * goal_pose.pose.orientation.y + 
                                            goal_pose.pose.orientation.z * goal_pose.pose.orientation.z));
        double goal_heading_angle_diff = goal_pose_z - yaw;
        
        while (goal_heading_angle_diff > M_PI) goal_heading_angle_diff -= 2*M_PI;
        while (goal_heading_angle_diff < -M_PI) goal_heading_angle_diff += 2*M_PI;

        cmd_vel.linear.x = 0;
        cmd_vel.angular.z = std::clamp(angular_pid.compute(goal_heading_angle_diff), -1.0, 1.0);

        if (std::abs(goal_heading_angle_diff) <= 0.05) {
            cmd_vel.angular.z = 0;
            has_goal = false;
            RCLCPP_INFO(this->get_logger(), "Goal reached!");
        }
    } else {
        // Normal path following
        if (std::abs(angle_diff) > 0.5) {
            cmd_vel.linear.x = 0.01;
            cmd_vel.angular.z = 0.5 * angle_diff;
            RCLCPP_INFO(this->get_logger(), "Rotating to target...");
        } else {
            cmd_vel.linear.x = 0.2;
            cmd_vel.angular.z = 0.2 * angle_diff;
            RCLCPP_INFO(this->get_logger(), "Moving to target...");
        }
    }

    pub_cmd->publish(cmd_vel);
}

void CmdPublisher::goal_callback(const std::shared_ptr<geometry_msgs::msg::PoseStamped> goal) {
    // Check if the goal point is in an occupied area
    double goal_x = goal->pose.position.x;
    double goal_y = goal->pose.position.y;
    
    if (!map.grid_map_data) {
        RCLCPP_WARN(this->get_logger(), "No map data available yet. Cannot validate goal position.");
        return;
    }
    
    // Convert world coordinates to map coordinates
    int map_x = static_cast<int>((goal_x - map.origin_x) / map.resolution);
    int map_y = static_cast<int>((goal_y - map.origin_y) / map.resolution);
    
    // Check if point is within map bounds
    if (map_x < 0 || map_x >= static_cast<int>(map.width) || 
        map_y < 0 || map_y >= static_cast<int>(map.height)) {
        RCLCPP_WARN(this->get_logger(), "Goal position is outside map bounds. Keeping previous goal.");
        return;
    }
    
    // Get occupancy value
    int index = map_y * map.width + map_x;
    int occupancy = map.grid_map_data->data[index];
    
    if (occupancy > 60) {
        RCLCPP_WARN(this->get_logger(), "Goal position is in occupied space (occupancy: %d). Keeping previous goal.", 
                    occupancy);
        return;
    }
    
    // If we get here, the goal position is valid
    goal_pose = *goal;
    has_goal = true;
    current_path_index = 0;
    
    RCLCPP_INFO(this->get_logger(), "Received new goal: x=%.2f, y=%.2f",
                goal_pose.pose.position.x, goal_pose.pose.position.y);
    
    if (position_updated) {
        plan_path();
    }
}

void CmdPublisher::path_planning_callback() {
    if (has_goal && position_updated && map.is_updated()) {
        plan_path();
    }
}

bool CmdPublisher::plan_path() {
    RCLCPP_INFO(this->get_logger(), "Planning path...");
    std::pair<double, double> start_point(x, y);
    std::pair<double, double> goal_point(goal_pose.pose.position.x, goal_pose.pose.position.y);
    
    path = find_path_astar(start_point, goal_point);
    
    if (!path.empty()) {
        RCLCPP_INFO(this->get_logger(), "Path found with %zu points", path.size());
        visualize_path();
        return true;
    }
    else {
        RCLCPP_WARN(this->get_logger(), "No path found!");
        geometry_msgs::msg::Twist cmd_vel;
        cmd_vel.linear.x = 0;
        
        // 현재 시간을 기준으로 회전 방향 결정 (3초 주기)
        auto now = this->get_clock()->now().seconds();
        bool clockwise = (static_cast<int>(now) % 6) >= 3;  // 3초마다 방향 전환
        
        cmd_vel.angular.z = clockwise ? -0.2 : 0.2;
        RCLCPP_INFO(this->get_logger(), "Rotating %s until path is found", 
                    clockwise ? "clockwise" : "counter-clockwise");
        
        pub_cmd->publish(cmd_vel);
    }
    return false;
}

std::vector<std::pair<double, double>> CmdPublisher::find_path_astar(
    std::pair<double, double> start,
    std::pair<double, double> goal) {
    RCLCPP_INFO(this->get_logger(), "Astar calculating...");
    
    auto compare = [](std::shared_ptr<PathNode> a, std::shared_ptr<PathNode> b) {
        return a->f_cost > b->f_cost;
    };
    std::priority_queue<std::shared_ptr<PathNode>,
                       std::vector<std::shared_ptr<PathNode>>,
                       decltype(compare)> open_set(compare);

    auto start_node = std::make_shared<PathNode>(start.first, start.second);
    start_node->h_cost = heuristic(start.first, start.second, goal.first, goal.second);
    start_node->f_cost = start_node->h_cost;

    open_set.push(start_node);
    std::map<std::pair<int, int>, std::shared_ptr<PathNode>> closed_set;

    while (!open_set.empty()) {
        auto current = open_set.top();
        open_set.pop();

        // 시작점인 경우 is_point_valid 검사를 건너뜀
        if (current != start_node && !map.is_point_valid(current->x, current->y)) {
            continue;
        }

        int grid_x = static_cast<int>(current->x * 20);
        int grid_y = static_cast<int>(current->y * 20);
        std::pair<int, int> grid_pos(grid_x, grid_y);

        if (closed_set.find(grid_pos) != closed_set.end()) {
            continue;
        }
        closed_set[grid_pos] = current;

        double distance_to_goal = std::hypot(current->x - goal.first,
                                           current->y - goal.second);
        
        if (distance_to_goal < 0.15) {
            std::vector<std::pair<double, double>> path;
            auto node = current;
            while (node != nullptr) {
                path.push_back({node->x, node->y});
                node = node->parent;
            }
            std::reverse(path.begin(), path.end());
            return path;
        }

        auto neighbors = get_neighbors(current);
        for (auto neighbor : neighbors) {
            grid_x = static_cast<int>(neighbor->x * 20);
            grid_y = static_cast<int>(neighbor->y * 20);
            grid_pos = std::pair<int, int>(grid_x, grid_y);

            if (closed_set.find(grid_pos) != closed_set.end()) {
                continue;
            }

            neighbor->g_cost = current->g_cost +
                std::hypot(neighbor->x - current->x, neighbor->y - current->y);
            neighbor->h_cost = heuristic(neighbor->x, neighbor->y,
                                       goal.first, goal.second);
            neighbor->f_cost = neighbor->g_cost + neighbor->h_cost;
            neighbor->parent = current;

            open_set.push(neighbor);
        }
    }

    return std::vector<std::pair<double, double>>();
}

std::vector<std::shared_ptr<PathNode>> CmdPublisher::get_neighbors(
    const std::shared_ptr<PathNode>& node) {
    
    std::vector<std::shared_ptr<PathNode>> neighbors;
    const double step = 0.15;
    
    const std::vector<std::pair<double, double>> directions = {
        {1, 0}, {-1, 0}, {0, 1}, {0, -1},
        {1, 1}, {1, -1}, {-1, 1}, {-1, -1}
    };

    for (const auto& dir : directions) {
        double new_x = node->x + dir.first * step;
        double new_y = node->y + dir.second * step;
        
        if (map.is_point_valid(new_x, new_y)) {
            neighbors.push_back(std::make_shared<PathNode>(new_x, new_y));
        }
    }

    return neighbors;
}

void CmdPublisher::visualize_path() {
    nav_msgs::msg::Path path_msg;
    path_msg.header.frame_id = "map";
    path_msg.header.stamp = this->get_clock()->now();

    for (const auto& point : path) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header = path_msg.header;
        pose.pose.position.x = point.first;
        pose.pose.position.y = point.second;
        pose.pose.position.z = 0.0;
        pose.pose.orientation.w = 1.0;
        path_msg.poses.push_back(pose);
    }

    pub_path->publish(path_msg);
}