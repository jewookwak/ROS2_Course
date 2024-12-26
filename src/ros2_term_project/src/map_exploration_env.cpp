#include "map_exploration_env.hpp"

namespace exploration {

RobotEnvironment::RobotEnvironment() 
    : Node("robot_environment") 
{
    // 기존 퍼블리셔/서브스크라이버 설정 유지
    state_pub_ = this->create_publisher<robot_state_msgs::msg::RobotState>("robot_state", 10);
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    reward_pub_ = this->create_publisher<std_msgs::msg::Float32>("reward", 10);

    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map", 10,
        std::bind(&RobotEnvironment::mapCallback, this, std::placeholders::_1));

    // TF 설정
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // 상태 업데이트 타이머 (10Hz)
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&RobotEnvironment::updateState, this));
        
    // 초기화
    previous_known_cells_ = 0;
    total_explored_radius_ = 3.5;  // Lidar 측정 범위: 3.5m
    current_linear_vel_ = 0.0;
    current_angular_vel_ = 0.0;
    
    RCLCPP_INFO(this->get_logger(), "Robot Environment 노드 초기화");
}

void RobotEnvironment::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) 
{
    // 맵 데이터 업데이트
    current_state_.grid_map = msg->data;
    current_state_.map_width = msg->info.width;
    current_state_.map_height = msg->info.height;
    map_resolution_ = msg->info.resolution;
    map_origin_x_ = msg->info.origin.position.x;
    map_origin_y_ = msg->info.origin.position.y;
        
    RCLCPP_DEBUG(this->get_logger(), "맵 업데이트 수신: %dx%d 셀", 
                 msg->info.width, msg->info.height);
}

void RobotEnvironment::updateState() 
{
    if (current_state_.grid_map.empty()) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, 
                            "맵 데이터 없음");
        return;
    }

    try {
        // TF에서 로봇 포즈 가져오기
        auto t = tf_buffer_->lookupTransform("map", "base_scan", tf2::TimePointZero);
        
        // 로봇 상태 업데이트
        current_state_.x = t.transform.translation.x;
        current_state_.y = t.transform.translation.y;
        current_state_.yaw = tf2::getYaw(t.transform.rotation);

        // 탐험 영역 계산
        int known_cells_count = 0;
        int total_frontier_cells = 0;
        int cells_in_radius = static_cast<int>(total_explored_radius_ / map_resolution_);
        
        // 로봇 중심 좌표 계산
        int robot_map_x = static_cast<int>((current_state_.x - map_origin_x_) / map_resolution_);
        int robot_map_y = static_cast<int>((current_state_.y - map_origin_y_) / map_resolution_);

        // 3.5m 반경 내 셀 탐색
        for (int dy = -cells_in_radius; dy <= cells_in_radius; ++dy) {
            for (int dx = -cells_in_radius; dx <= cells_in_radius; ++dx) {
                int map_x = robot_map_x + dx;
                int map_y = robot_map_y + dy;
                
                // 맵 경계 체크
                if (map_x < 0 || map_x >= static_cast<int>(current_state_.map_width) ||
                    map_y < 0 || map_y >= static_cast<int>(current_state_.map_height)) {
                    continue;
                }

                // 거리 체크 (원형 영역)
                float distance = std::sqrt(dx*dx + dy*dy) * map_resolution_;
                if (distance > total_explored_radius_) {
                    continue;
                }

                // 인덱스 계산
                int idx = map_y * current_state_.map_width + map_x;
                int8_t cell_value = current_state_.grid_map[idx];

                // Known cells 계산 (0-59: free space)
                if (cell_value >= 0 && cell_value < 60) {
                    known_cells_count++;
                }

                // Frontier cells 계산 (인접한 알 수 없는 셀 확인)
                bool is_frontier = false;
                for (int ny = -1; ny <= 1; ++ny) {
                    for (int nx = -1; nx <= 1; ++nx) {
                        int neighbor_x = map_x + nx;
                        int neighbor_y = map_y + ny;
                        
                        if (neighbor_x >= 0 && neighbor_x < static_cast<int>(current_state_.map_width) &&
                            neighbor_y >= 0 && neighbor_y < static_cast<int>(current_state_.map_height)) {
                            int neighbor_idx = neighbor_y * current_state_.map_width + neighbor_x;
                            if (current_state_.grid_map[neighbor_idx] == -1) {
                                is_frontier = true;
                                break;
                            }
                        }
                    }
                    if (is_frontier) break;
                }

                if (is_frontier) {
                    total_frontier_cells++;
                }
            }
        }

        // 보상 계산
        float reward = 0.0;
        int new_cells_explored = known_cells_count - previous_known_cells_;
        
        if (new_cells_explored > 0) {
            reward += new_cells_explored * 0.1f;  // 탐험한 셀에 대한 기본 보상
            
            // Frontier 셀 탐험 보상
            if (total_frontier_cells > 0) {
                reward += 0.05f * total_frontier_cells;
            }
        }
        
        // 장애물과의 최소 거리 패널티
        float min_obstacle_distance = getMinObstacleDistance(current_state_.x, current_state_.y);
        if (min_obstacle_distance < 0.3) {  // 30cm 임계값
            reward -= (0.3f - min_obstacle_distance) * 2.0f;
        }

        // 보상 퍼블리시
        auto reward_msg = std_msgs::msg::Float32();
        reward_msg.data = reward;
        reward_pub_->publish(reward_msg);

        // 로봇 상태 메시지 생성
        auto state_msg = robot_state_msgs::msg::RobotState();
        state_msg.x = current_state_.x;
        state_msg.y = current_state_.y;
        state_msg.yaw = current_state_.yaw;
        state_msg.linear_velocity = current_linear_vel_;
        state_msg.angular_velocity = current_angular_vel_;
        
        // 3.5m 반경 내 주요 메트릭 추가
        state_msg.known_cells_count = known_cells_count;
        state_msg.frontier_cells_count = total_frontier_cells;
        
        // 로컬 맵 데이터 추가 (3.5m 반경)
        std::vector<int8_t> local_map;
        getLocalMap(current_state_.x, current_state_.y, cells_in_radius, local_map);
        state_msg.local_map = local_map;
        
        // 상태 퍼블리시
        state_pub_->publish(state_msg);

        // 이전 알려진 셀 업데이트
        previous_known_cells_ = known_cells_count;

    } catch (const tf2::TransformException &ex) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                            "맵-베이스 스캔 변환 실패: %s", ex.what());
    }
}

void RobotEnvironment::getLocalMap(float center_x, float center_y, int cells_radius, 
                                   std::vector<int8_t>& local_map) 
{
    local_map.clear();
    local_map.resize((2 * cells_radius + 1) * (2 * cells_radius + 1), -1);  // 수정된 부분
    
    int center_grid_x = static_cast<int>((center_x - map_origin_x_) / map_resolution_);
    int center_grid_y = static_cast<int>((center_y - map_origin_y_) / map_resolution_);
    
    for (int dy = -cells_radius; dy <= cells_radius; ++dy) {
        for (int dx = -cells_radius; dx <= cells_radius; ++dx) {
            int map_x = center_grid_x + dx;
            int map_y = center_grid_y + dy;
            
            // 맵 경계 체크
            if (map_x >= 0 && map_x < static_cast<int>(current_state_.map_width) &&
                map_y >= 0 && map_y < static_cast<int>(current_state_.map_height)) {
                
                int map_idx = map_y * current_state_.map_width + map_x;
                int local_idx = (dy + cells_radius) * (2 * cells_radius + 1) + (dx + cells_radius);
                local_map[local_idx] = current_state_.grid_map[map_idx];
            }
        }
    }
}
float RobotEnvironment::getMinObstacleDistance(float x, float y) 
{
    if (current_state_.grid_map.empty()) {
        return std::numeric_limits<float>::max();
    }
    
    // 로봇 위치를 그리드 좌표로 변환
    int grid_x = static_cast<int>((x - map_origin_x_) / map_resolution_);
    int grid_y = static_cast<int>((y - map_origin_y_) / map_resolution_);
    
    float min_distance = std::numeric_limits<float>::max();
    const int search_radius = static_cast<int>(2.0 / map_resolution_);  // 2m 검색 반경
    
    for (int dy = -search_radius; dy <= search_radius; ++dy) {
        for (int dx = -search_radius; dx <= search_radius; ++dx) {
            int check_x = grid_x + dx;
            int check_y = grid_y + dy;
            
            // 맵 경계 체크
            if (check_x >= 0 && check_x < static_cast<int>(current_state_.map_width) &&
                check_y >= 0 && check_y < static_cast<int>(current_state_.map_height)) {
                
                int idx = check_y * current_state_.map_width + check_x;
                
                // 장애물 셀 (60 이상)인 경우 거리 계산
                if (current_state_.grid_map[idx] >= 60) {
                    float distance = std::hypot(dx * map_resolution_, dy * map_resolution_);
                    min_distance = std::min(min_distance, distance);
                }
            }
        }
    }
    
    return min_distance;
}
// 나머지 메서드들은 기존 구현 유지
} // namespace exploration