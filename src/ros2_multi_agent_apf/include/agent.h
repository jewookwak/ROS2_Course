#ifndef APF_AGENT_H
#define APF_AGENT_H

#include "rclcpp/rclcpp.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker_array.hpp"
#include "std_msgs/msg/string.hpp"
#include "yaml-cpp/yaml.h"
#include <Eigen/Dense>
#include <utility>
#include <cmath>

namespace apf {
typedef Eigen::Vector3d Vector3d;
typedef std::vector<Eigen::Vector3d> Vector3ds;

class State {
public:
  Vector3d position;
  Vector3d velocity;
};

class Obstacle {
public:
  Vector3d position;
  double radius;
};

class ApfAgent : public rclcpp::Node {
private:
  rclcpp::TimerBase::SharedPtr timer_tf, timer_pub;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_pose;
  //arrival time
  static std::vector<bool> is_arrived;  // 각 에이전트의 도착 여부
  static std::vector<double> arrival_times;  // 각 에이전트의 도착 시간
  static size_t arrived_agents_count;  // 도착한 에이전트 수
  double start_time;  // 각 에이전트의 출발 시간
  
  // 도착 관련 새로운 멤버
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr arrival_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr arrival_sub_;
  std::vector<bool> agents_arrived_;
  size_t total_arrived_;
  double arrival_time_;
  bool has_arrived_;
  std::vector<double> all_arrival_times_;
  // collision check
  static std::vector<bool> collision_checked;  // 각 에이전트의 충돌 체크 여부
  static int total_collisions;                 // 총 충돌 횟수
  // delayed start
  std::vector<bool> is_started;     // 각 에이전트의 출발 상태
  std::vector<double> start_times;  // 각 에이전트의 출발 시간
  const double agent_delay = 0.05;   // 에이전트 간 출발 시간 간격 (0.5초)
  // Simulator
  std::string frame_id = "world"; // source frame id for tf broadcast 
  double dt = 0.02;               // timer period (s)
  double start_time_;  // 초 단위로 저장
  bool start_time_initialized_ = false;
  // Agent
  size_t agent_id = 0;  // each agent has unique agent_id
  State state;          // agent's state
  Vector3d start, goal; // agent's start and goal positions
  double radius = 0.15; // (m)
  double max_acc = 6;   // (m/s^2)
  // parameter
  double alpha; // 파라미터 alpha
  double beta;  // 파라미터 beta
  double influence_distance;
  // Apf_controller parameters
  const double k_VG = 1.0;     // Goal attraction gain
  const double q_VG = 1.0;     // Goal charge
  const double k_VO = 1.0;     // Obstacle repulsion gain
  const double q_VO = 1.0;     // Obstacle charge
  const double d_o = 2.0;      // Obstacle influence distance
  const double k_damp = 0.5;   // Damping coefficient
  // const double max_acc = 1.0;  // Maximum acceleration
  // base_damping 상수 추가 (이미 있는 상수들 근처에 추가)
  const double base_damping = 4.0;

  // calculate_avoidance 함수 선언 추가 (is_collision_likely 함수 선언 아래에 추가)
  std::pair<Vector3d, double> calculate_avoidance(
      const Vector3d& relative_pos, 
      const Vector3d& relative_vel,
      double dist_VA, 
      bool near_center,
      size_t other_id);

  // 새로운 상수 멤버들
  const double max_velocity = 20.0;
  const double min_safe_distance = radius * 2.0 + 1.0;
  const double arrival_zone = 0.3;
  const double slow_zone = 1.0;
  const double final_approach_zone = 1.0;
  const double transition_distance = 2.6;
  const double transition_width = 0.4;

  // 충돌 예측 함수
  bool is_collision_likely(const Vector3d& other_pos, const Vector3d& other_vel, 
                          double prediction_time = 1.0);

  // Environment
  size_t number_of_agents = 0;
  size_t number_of_obstacles = 0;
  std::vector<State> agent_states;     // List of all agent states
  Vector3ds agent_positions;       // agent's positions, including the current agent itself
  std::vector<Obstacle> obstacles; // obstacles list

  void timer_tf_callback();
  
  void timer_pub_callback();
  
  void listen_tf();

  void update_state();

  void broadcast_tf();

  Vector3d apf_controller();

  void publish_marker_pose();

  Vector3d cal_force(double k, double q_T, double q_V, Vector3d target_position, Vector3d AGV_position);

  void check_collisions();

  void print_collision_statistics();
  // 도착 메시지 콜백
  void arrival_callback(const std_msgs::msg::String::SharedPtr msg);
  
  // 도착 메시지 생성 함수
  std::string create_arrival_message(size_t agent_id, double arrival_time);
  
  // 도착 메시지 파싱 함수
  std::pair<size_t, double> parse_arrival_message(const std::string& msg);

public:
  ApfAgent();
};

} // namespace apf

#endif // APF_AGENT_H
