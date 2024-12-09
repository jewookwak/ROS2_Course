#include "agent.h"
#include <cmath>

using namespace std::chrono_literals;
namespace apf {

// static 멤버 변수 초기화 - namespace 안에, 모든 함수 정의 전에 위치
std::vector<bool> ApfAgent::collision_checked;
int ApfAgent::total_collisions = 0;
std::vector<bool> ApfAgent::is_arrived;
std::vector<double> ApfAgent::arrival_times;
size_t ApfAgent::arrived_agents_count = 0; 


ApfAgent::ApfAgent() : rclcpp::Node("agent") {

    // Agent id 설정
    this->declare_parameter("agent_id", 0);
    agent_id = this->get_parameter("agent_id").as_int();

    // 미션 파일 설정 및 로드
    this->declare_parameter("mission_file_name", "~/ros2_ws/src/ros2_artificial_potential_field/mission/mission_single_agent.yaml");
    std::string mission_file_name = this->get_parameter("mission_file_name").as_string();

    // tuning parameter setting
    this->declare_parameter("alpha", 1.0);
    this->declare_parameter("beta", 1.0);
    this->declare_parameter("influence_distance", 1.0);

    alpha = this->get_parameter("alpha").as_double();
    beta = this->get_parameter("beta").as_double();
    influence_distance = this->get_parameter("influence_distance").as_double();

    YAML::Node mission = YAML::LoadFile(mission_file_name);
    auto agents_yaml = mission["agents"];
    number_of_agents = agents_yaml.size();
    agent_states.resize(number_of_agents);

    // 각 agent의 시작 및 목표 위치 로드
    for (size_t id = 0; id < number_of_agents; id++) {
        agent_states[id].position = Vector3d(agents_yaml[id]["start"][0].as<double>(),
                                            agents_yaml[id]["start"][1].as<double>(),
                                            agents_yaml[id]["start"][2].as<double>());
    }
    start = agent_states[agent_id].position;
    goal = Vector3d(agents_yaml[agent_id]["goal"][0].as<double>(),
                    agents_yaml[agent_id]["goal"][1].as<double>(),
                    agents_yaml[agent_id]["goal"][2].as<double>());

    // 장애물 정보 로드
    auto obstacles_yaml = mission["obstacles"];
    number_of_obstacles = obstacles_yaml.size();
    obstacles.resize(number_of_obstacles);
    for (size_t obs_id = 0; obs_id < number_of_obstacles; obs_id++) {
        obstacles[obs_id].position = Vector3d(obstacles_yaml[obs_id]["position"][0].as<double>(),
                                            obstacles_yaml[obs_id]["position"][1].as<double>(),
                                            obstacles_yaml[obs_id]["position"][2].as<double>());
        obstacles[obs_id].radius = obstacles_yaml[obs_id]["radius"].as<double>();
    }

    // Initialize agent state
    state = agent_states[agent_id];
    state.velocity = Vector3d(0, 0, 0);

    // TF2_ROS 설정
    tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
    tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // ROS timer 및 publisher 설정
    int timer_period_ms = static_cast<int>(dt * 1000);
    timer_tf = this->create_wall_timer(std::chrono::milliseconds(timer_period_ms), std::bind(&ApfAgent::timer_tf_callback, this));
    timer_pub = this->create_wall_timer(40ms,
                                    std::bind(&ApfAgent::timer_pub_callback, this));

    pub_pose = this->create_publisher<visualization_msgs::msg::MarkerArray>("robot/pose", 10);

    std::cout << "[ApfAgent] Agent" << agent_id << " is ready." << std::endl;

    // Publisher와 Subscriber 설정
    arrival_pub_ = this->create_publisher<std_msgs::msg::String>(
        "agent_arrivals", 10);
    arrival_sub_ = this->create_subscription<std_msgs::msg::String>(
        "agent_arrivals", 10,
        std::bind(&ApfAgent::arrival_callback, this, std::placeholders::_1));

    // 도착 관련 초기화
    agents_arrived_.resize(number_of_agents, false);
    total_arrived_ = 0;
    has_arrived_ = false;
    all_arrival_times_.resize(number_of_agents, 0.0);
    arrival_time_ = 0.0;
    
    // is_arrived와 arrival_times 벡터 초기화
    if (is_arrived.empty()) {
        is_arrived.resize(number_of_agents, false);
        arrival_times.resize(number_of_agents, 0.0);
        RCLCPP_INFO(this->get_logger(), "Initialized arrival tracking vectors with size: %zu", number_of_agents);
    }
    
    start_time = 0.0;  // 출발 시간 초기화
    start_time_ = 0.0;  // 초기화
    
    // collision_checked 벡터 초기화 (static 멤버이므로 한 번만 초기화되도록)
    if (collision_checked.empty()) {
        collision_checked.resize(number_of_agents, false);
        RCLCPP_INFO(this->get_logger(), "Initialized collision_checked vector with size: %zu", number_of_agents);
    }
    RCLCPP_INFO(this->get_logger(), "Agent %zu: collision_checked size is %zu", agent_id, collision_checked.size());



    // 각 에이전트의 출발 상태 초기화
    is_started.resize(number_of_agents, false);

}

void ApfAgent::arrival_callback(const std_msgs::msg::String::SharedPtr msg) {
    auto [arrived_agent_id, arrived_time] = parse_arrival_message(msg->data);
    
    if (!agents_arrived_[arrived_agent_id]) {
        agents_arrived_[arrived_agent_id] = true;
        all_arrival_times_[arrived_agent_id] = arrived_time;
        total_arrived_++;
        
        // RCLCPP_INFO(this->get_logger(), 
        //     "Agent%zu received arrival notification from Agent%zu. Time: %.2f",
        //     agent_id, arrived_agent_id, arrived_time);

        if (total_arrived_ == number_of_agents) {
            // 모든 에이전트가 도착했을 때
            double total_time = 0.0;
            double max_time = 0.0;
            double min_time = std::numeric_limits<double>::max();
            
            RCLCPP_INFO(this->get_logger(), "\n=== All Agents Have Arrived ===");
            for (size_t i = 0; i < number_of_agents; i++) {
                RCLCPP_INFO(this->get_logger(), "Agent %zu arrival time: %.2f seconds",
                    i, all_arrival_times_[i]);
                total_time += all_arrival_times_[i];
                max_time = std::max(max_time, all_arrival_times_[i]);
                min_time = std::min(min_time, all_arrival_times_[i]);
            }
            
            double avg_time = total_time / number_of_agents;
            RCLCPP_INFO(this->get_logger(), "Average arrival time: %.2f seconds", avg_time);
            RCLCPP_INFO(this->get_logger(), "Minimum arrival time: %.2f seconds", min_time);
            RCLCPP_INFO(this->get_logger(), "Maximum arrival time: %.2f seconds", max_time);
            RCLCPP_INFO(this->get_logger(), "===========================\n");
        }
    }
}

std::string ApfAgent::create_arrival_message(size_t agent_id, double arrival_time) {
    return std::to_string(agent_id) + "," + std::to_string(arrival_time);
}

std::pair<size_t, double> ApfAgent::parse_arrival_message(const std::string& msg) {
    size_t comma_pos = msg.find(',');
    size_t agent_id = std::stoul(msg.substr(0, comma_pos));
    double arrival_time = std::stod(msg.substr(comma_pos + 1));
    return {agent_id, arrival_time};
}


void ApfAgent::timer_tf_callback() {
  listen_tf();
  update_state();
  broadcast_tf();
}

void ApfAgent::timer_pub_callback() {
  publish_marker_pose();
}

Vector3d ApfAgent::cal_force(double k, double q_T, double q_V, Vector3d target_position, Vector3d AGV_position) {
    Vector3d r_VT = target_position - AGV_position;  
    double dist_VT = r_VT.norm();          
    Vector3d u;

    if (dist_VT > 0.001) {  // Avoid division by zero
        // const double d_threshold = 4.0;  // 전환 거리 임계값
        double magnitude;
        magnitude = k * q_V * q_T / (dist_VT * dist_VT);
        // magnitude = k * q_V * q_T / ((dist_VT-radius*2.5) * (dist_VT-radius*2.5));
        // if (dist_VT > d_threshold) {
        //     // 먼 거리: Quadratic potential (1/r)
        //     magnitude = k * q_V * q_T / (dist_VT * dist_VT);
        // } else {
        //     // 가까운 거리: Linear potential (constant)
        //     magnitude = k * q_V * q_T / (d_threshold * d_threshold);
        // }
        
        Vector3d direction = r_VT / dist_VT;
        u = -magnitude * direction;
    } else {
        u = Vector3d::Zero();
    }
    return u;
}

void ApfAgent::listen_tf() {
  for (size_t id = 0; id < number_of_agents; id++) {
    if (id == agent_id) {
      // 자신의 상태는 직접 관리하므로 스킵
      continue;
    }

    std::string target_frame = "world";
    std::string source_frame = "agent_" + std::to_string(id);
    geometry_msgs::msg::TransformStamped t;
    
    try {
      t = tf_buffer->lookupTransform(target_frame, source_frame, tf2::TimePointZero);
      agent_states[id].position = Vector3d(t.transform.translation.x, 
                                         t.transform.translation.y, 
                                         t.transform.translation.z);
    } catch (const tf2::TransformException &ex) {
      // TF를 받아올 수 없는 경우 초기 위치 유지
      // 에러 메시지는 불필요한 스팸을 방지하기 위해 주석 처리
      // RCLCPP_DEBUG(this->get_logger(), "Could not transform %s to %s: %s", 
      //              source_frame.c_str(), target_frame.c_str(), ex.what());
      continue;  // 다음 에이전트로 진행
    }
  }
}
void ApfAgent::check_collisions() {
    // 이미 충돌이 체크된 에이전트는 스킵
    if (collision_checked[agent_id]) {
        return;
    }

    // 현재 에이전트와 다른 모든 에이전트 사이의 거리 체크
    for (size_t other_id = 0; other_id < number_of_agents; other_id++) {
        // 자기 자신이거나 이미 충돌이 체크된 에이전트는 스킵
        // if (other_id == agent_id || collision_checked[other_id]) {
        if (other_id == agent_id) {
            continue;
        }

        // 두 에이전트 사이의 거리 계산
        Vector3d relative_pos = agent_states[agent_id].position - agent_states[other_id].position;
        double distance = relative_pos.norm();
        // RCLCPP_INFO(this->get_logger(),
        //         "Agent %zu and Agent %zu distance is %f and radius is %f",
        //         agent_id, other_id, distance, radius*2);
            
        // 디버깅을 위한 거리 출력 (1초마다)
        static double last_debug_time = 0;
        double current_time = this->get_clock()->now().seconds();
        if (current_time - last_debug_time > 1.0) {
            RCLCPP_DEBUG(this->get_logger(), 
                "Distance between Agent %zu and Agent %zu: %f (collision threshold: %f)",
                agent_id, other_id, distance, radius * 2.0);
            last_debug_time = current_time;
        }

        // 충돌 발생 (두 에이전트의 반지름 합보다 거리가 작은 경우)
        if (distance <= radius * 2.0) {
            collision_checked[agent_id] = true;
            collision_checked[other_id] = true;
            total_collisions++;

            RCLCPP_INFO(this->get_logger(),
                "Collision detected between Agent %zu and Agent %zu at distance %f (Time: %f s)",
                agent_id, other_id, distance, current_time);
            
            break;  // 현재 에이전트는 더 이상 체크할 필요 없음
        }
    }
}

void ApfAgent::print_collision_statistics() {
   if (agent_id == 0) {  // 0번 에이전트만 통계 출력
       std::cout << "\n=== Collision Statistics ===" << std::endl;
       std::cout << "Current time: " << this->get_clock()->now().seconds() << "s" << std::endl;
       
       // collision_checked 배열에서 true인 에이전트 확인
       int agents_with_collision = 0;
       std::cout << "Agents involved in collisions: ";
       for (size_t id = 0; id < number_of_agents; id++) {
           if (collision_checked[id]) {
               std::cout << id << " ";
               agents_with_collision++;
           }
       }
       std::cout << std::endl;
       
       // 전체 통계 출력
       std::cout << "\nSummary:" << std::endl;
       std::cout << "Total collisions: " << agents_with_collision/2 << std::endl;  // 충돌 쌍의 수
       std::cout << "Number of agents involved in collisions: " << agents_with_collision << std::endl;
       std::cout << "Number of agents without collisions: " << (number_of_agents - agents_with_collision) << std::endl;
       std::cout << "========================\n" << std::endl;
   }
}

void ApfAgent::update_state() {
   // 현재 시간 확인
   double current_time = this->get_clock()->now().seconds();
   
   // 시작 시간 계산 (처음 한 번만)
   static bool time_initialized = false;
   static double simulation_start_time;
   static double last_print_time = 0;  // 통계 출력용 시간 변수
   static double last_collision_check_print = 0;  // collision_checked 출력용 시간 변수
   
   if (!time_initialized) {
       simulation_start_time = current_time;
       start_time = current_time;  // 출발 시간 기록
       start_time_ = current_time;  // 출발 시간 기록
       last_print_time = current_time;
       last_collision_check_print = current_time;
       time_initialized = true;
       std::cout << "[ApfAgent] Agent" << agent_id << " will start at " << 
                   simulation_start_time + (agent_id * agent_delay) << " seconds." << std::endl;
   }

   // 에이전트의 시작 시간 확인
   double agent_start_time = simulation_start_time + (agent_id * agent_delay);
   
   if (!is_started[agent_id] && current_time >= agent_start_time) {
       is_started[agent_id] = true;
       
       std::cout << "[ApfAgent] Agent" << agent_id << " started moving at " << current_time << " seconds." << std::endl;
   }
   
   if (is_started[agent_id]) {
       // 시작된 경우에만 제어 입력을 계산하고 상태를 업데이트
       Vector3d u = apf_controller();
       
       // 상태 업데이트: 속도와 위치를 갱신
       state.velocity += u * dt;
       state.position += state.velocity * dt;
       
       // 최대 속도를 넘지 않도록 클램핑
       double max_velocity = 10.0;
       if (state.velocity.norm() > max_velocity) {
           state.velocity = state.velocity.normalized() * max_velocity;
       }
   } else {
       // 아직 시작하지 않은 경우 초기 위치에 고정
       state.velocity = Vector3d::Zero();
   }
   
   // 업데이트된 상태를 agent_states에 저장
   agent_states[agent_id] = state;

   // 모든 agent의 상태가 업데이트된 후에만 마커를 게시
   if (agent_id == number_of_agents - 1) {
       publish_marker_pose();
   }

   // 상태 업데이트 후 충돌 체크
   check_collisions();

   // collision_checked 상태를 2초마다 출력
//    if (agent_id == 0 && current_time - last_collision_check_print > 2.0) {
//        std::cout << "\n=== Collision Checked Status at " << current_time << "s ===" << std::endl;
//        for (size_t id = 0; id < number_of_agents; id++) {
//            if (collision_checked[id]) {
//                std::cout << "Agent " << id << ": Collision detected" << std::endl;
//            } else {
//                std::cout << "Agent " << id << ": No collision" << std::endl;
//            }
//        }
//        std::cout << "================================\n" << std::endl;
//        last_collision_check_print = current_time;
//    }

//    // 주기적으로 충돌 통계 출력 (5초마다)
//    if (agent_id == 0 && current_time - last_print_time > 5.0) {
//        print_collision_statistics();
//        last_print_time = current_time;
//    }
}


void ApfAgent::broadcast_tf() {
  // world 프레임 브로드캐스트
  geometry_msgs::msg::TransformStamped world_tf;
  world_tf.header.stamp = this->get_clock()->now();
  world_tf.header.frame_id = "map";  // ROS의 기본 프레임
  world_tf.child_frame_id = "world";
  world_tf.transform.rotation.w = 1.0;  // 단위 회전
  tf_broadcaster->sendTransform(world_tf);

  // 에이전트 프레임 브로드캐스트
  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = this->get_clock()->now();
  t.header.frame_id = "world";
  t.child_frame_id = "agent_" + std::to_string(agent_id);

  t.transform.translation.x = state.position.x();
  t.transform.translation.y = state.position.y();
  t.transform.translation.z = 0.0;

  tf2::Quaternion q;
  double yaw = std::atan2(state.velocity.y(), state.velocity.x());  // 속도 방향으로 회전
  q.setRPY(0, 0, yaw);
  t.transform.rotation.x = q.x();
  t.transform.rotation.y = q.y();
  t.transform.rotation.z = q.z();
  t.transform.rotation.w = q.w();

  tf_broadcaster->sendTransform(t);
}

Vector3d ApfAgent::apf_controller() {
    Vector3d u;
    const double min_safe_distance = radius * 2.0+2.0;
    
    // 거리 계산
    double dist_to_goal = (goal - state.position).norm();
    double dist_to_start = (start - state.position).norm();
    
    // 제어 구간 설정
    const double arrival_zone = 0.001;       // 최종 도착 판정 거리
    const double slow_zone = 8.0;          // 감속 시작 거리
    const double final_approach_zone = 4.0; // 최종 접근 구간
    
    // 속도 제어
    double max_velocity = 10.0;
    double min_velocity = 0.02;
    
    if (dist_to_goal < arrival_zone) {
        // check arrival time
        if (!has_arrived_) {
            // 처음 도착한 경우
            has_arrived_ = true;
            arrival_time_ = this->get_clock()->now().seconds() - start_time_;
            
            // 도착 메시지 발행
            auto msg = std_msgs::msg::String();
            msg.data = create_arrival_message(agent_id, arrival_time_);
            arrival_pub_->publish(msg);
            
            RCLCPP_INFO(this->get_logger(), 
                "Agent%zu has arrived at goal. Time taken: %.2f seconds.",
                agent_id, arrival_time_);
        }

        // 매우 근접한 경우 목표점으로 직접 이동
        Vector3d direct_to_goal = goal - state.position;
        u = direct_to_goal / dt;
        state.velocity = Vector3d::Zero();
        return u;
    } else if (dist_to_goal < final_approach_zone) {
        max_velocity = min_velocity + (max_velocity - min_velocity) * 
                      (dist_to_goal / final_approach_zone);
    } else if (dist_to_goal < slow_zone) {
        max_velocity = min_velocity + (max_velocity - min_velocity) * 
                      std::sqrt(dist_to_goal / slow_zone);
    }
    
    if (state.velocity.norm() > max_velocity) {
        state.velocity = state.velocity.normalized() * max_velocity;
    }

    // 목표점까지 거리에 따른 인력 강화
    double k_GV = 12.0;
    double q_G = -12.0;
    double q_V = 12.0;
    
    // 목표 지점 근처에서 인력 강화
    double goal_force_multiplier = 1.0;
    if (dist_to_goal < final_approach_zone) {
        // 거리가 가까워질수록 인력 강화
        goal_force_multiplier = 1.0 + 5.0 * (1.0 - dist_to_goal / final_approach_zone);
        k_GV *= goal_force_multiplier;
    }
    
    Vector3d u_goal = cal_force(k_GV, q_G, q_V, goal, state.position);

    // Repulsive Forces
    Vector3d u_repulsive(0, 0, 0);
    double k_VO = 5.0;
    double d_o = 2.0;

    // 시작점/목표점에서의 가중치
    const double transition_distance = 1.6;
    const double transition_width = 0.4;
    
    double weight_start = 1.0;
    double weight_goal = 1.0;
    
    if (dist_to_start < transition_distance + transition_width) {
        double x = dist_to_start - transition_distance;
        weight_start = 1.0 / (1.0 + std::exp(-x / (transition_width * 0.1)));
    }
    
    if (dist_to_goal < transition_distance + transition_width) {
        double x = dist_to_goal - transition_distance;
        weight_goal = 1.0 / (1.0 + std::exp(-x / (transition_width * 0.1)));
    }
    
    double agent_force_weight = std::min(weight_start, weight_goal);
    
    // 목표 지점 근처에서 척력 급격히 감소
    if (dist_to_goal < final_approach_zone) {
        // 목표 지점에 가까워질수록 척력 더 강하게 감소
        double repulsive_reduction = std::pow(dist_to_goal / final_approach_zone, 1.2);
        agent_force_weight *= repulsive_reduction;
    }

    // Obstacles - 장애물 척력은 유지
    for (const auto& obstacle : obstacles) {
        double dist_VO = (state.position - obstacle.position).norm();
        if (dist_VO <= d_o) {
            double q_O = 4.0;
            if (dist_VO < min_safe_distance * 1.5) {
                q_O *= std::exp(min_safe_distance / dist_VO);
            }
            Vector3d f_obs = cal_force(k_VO, q_O, q_V, obstacle.position, state.position);
            u_repulsive += f_obs;
        }
    }

    // Agents - 목표 지점 근처에서 급격히 감소하는 척력
    for (size_t id = 0; id < number_of_agents; id++) {
        if (id == agent_id) continue;
        
        Vector3d relative_pos = state.position - agent_states[id].position;
        double dist_VA = relative_pos.norm();
        
        if (dist_VA <= d_o) {
            double q_A = 80.0;
            double distance_factor = 1.0;
            
            // 최소 안전거리 이내일 때만 강한 척력 유지
            if (dist_VA < min_safe_distance * 1.4) {
                distance_factor = std::exp(2.0 * (min_safe_distance / dist_VA - 1.0));
                distance_factor = std::min(distance_factor,6.0);
            } else if (dist_to_goal < final_approach_zone) {
                // 안전거리 밖에서는 목표점 근처에서 척력 크게 감소
                distance_factor *= 1;
            }
            
            // 상대 속도 기반 가중치도 목표점 근처에서 감소
            Vector3d relative_velocity = state.velocity - agent_states[id].velocity;
            double velocity_factor = 1.0 + relative_velocity.norm() * 0.5;
            if (dist_to_goal < final_approach_zone) {
                velocity_factor *= dist_to_goal / final_approach_zone;
            }
            
            Vector3d f_agent = cal_force(k_VO, q_A * distance_factor, q_V, 
                                       agent_states[id].position, state.position);
            
            // 최종 척력에 모든 가중치 적용
            u_repulsive += (velocity_factor * f_agent * agent_force_weight);
        }
        double base_damping = 5.0;
        double damping_gain = base_damping;
        if (dist_VA < min_safe_distance * 1.2) {
          damping_gain *= std::pow((1+dist_VA), 2);
        }
    }

    // 감쇠력 - 목표점 근처에서 감소
    double base_damping = 5.0;
    double damping_gain = base_damping;
    if (dist_to_goal < final_approach_zone) {
        damping_gain *= std::pow(dist_to_goal / final_approach_zone, 0.5);
    }
    
    
    Vector3d u_damp = -damping_gain * state.velocity;

    // 힘 결합 - 목표점 근처에서 인력 비중 증가
    if (dist_to_goal < final_approach_zone) {
        // 거리에 따라 인력의 비중을 증가
        double goal_weight = 1.0 + 2.0 * (1.0 - dist_to_goal / final_approach_zone);
        u = goal_weight * u_goal + u_repulsive + u_damp;
    } else {
        u = u_goal + u_repulsive + u_damp;
    }

    // 가속도 제한
    const double max_control = 50.0;
    if (u.norm() > max_control) {
        u = u.normalized() * max_control;
    }

    return u;
}

void ApfAgent::publish_marker_pose() {
  visualization_msgs::msg::MarkerArray marker_array;
  
  for (size_t id = 0; id < number_of_agents; id++) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = this->get_clock()->now();
    marker.ns = "agent";
    marker.id = static_cast<int>(id);
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;

    // 각 agent의 위치를 agent_states에서 가져옵니다.
    marker.pose.position.x = agent_states[id].position.x();
    marker.pose.position.y = agent_states[id].position.y();
    marker.pose.position.z = 0.0;

    marker.scale.x = radius * 2.0;
    marker.scale.y = radius * 2.0;
    marker.scale.z = radius * 2.0;
    marker.color.a = 0.3;
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    marker_array.markers.push_back(marker);
  }

  for (size_t obs_id = 0; obs_id < number_of_obstacles; obs_id++) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = this->get_clock()->now();
    marker.ns = "obstacle";
    marker.id = static_cast<int>(obs_id);
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    
    marker.pose.position.x = obstacles[obs_id].position.x();
    marker.pose.position.y = obstacles[obs_id].position.y();
    marker.pose.position.z = obstacles[obs_id].position.z();
    marker.pose.orientation.w = 1.0;

    marker.scale.x = 2 * obstacles[obs_id].radius;
    marker.scale.y = 2 * obstacles[obs_id].radius;
    marker.scale.z = 2 * obstacles[obs_id].radius;
    marker.color.a = 0.5;
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;

    marker_array.markers.emplace_back(marker);
  }
  
  pub_pose->publish(marker_array);
}


} // namespace apf


// next task: use cal_force at Apf_control