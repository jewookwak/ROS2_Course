// #include "rclcpp/rclcpp.hpp"
// #include "geometry_msgs/msg/twist.hpp"
// #include <turtlesim/msg/pose.hpp>

// using namespace std::chrono_literals;
// using std::placeholders::_1;

// struct State {
//     double x = 0;
//     double y = 0;
//     double theta = 0;
// };

// class TurtlebotController : public rclcpp::Node {
// public:
//   TurtlebotController() : Node("turtlebot_controller"), current_target_index(0) {
//     publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10);
//     subscription_ = this->create_subscription<turtlesim::msg::Pose>(
//       "turtle1/pose", 10, std::bind(&TurtlebotController::topic_callback, this, _1));
//     timer_ = this->create_wall_timer(
//       100ms, std::bind(&TurtlebotController::timer_callback, this));

//     // Define the target positions for each corner of the square path
//     target_positions = {{0.0, 0.0}, {2.0, 0.0}, {2.0, 2.0}, {0.0, 2.0}, {0.0, 0.0}};
//   }

// private:
//   void timer_callback() {
//     geometry_msgs::msg::Twist cmd;

//     double target_x = target_positions[current_target_index][0];
//     double target_y = target_positions[current_target_index][1];
    
//     double dx = target_x - turtlebot_state.x;
//     double dy = target_y - turtlebot_state.y;
//     double distance = sqrt(dx * dx + dy * dy);
//     double angle_to_target = atan2(dy, dx);

//     if (distance > 0.1) {
//       // Move towards the target position
//       cmd.linear.x = 1.0;  // Moving speed
//       cmd.angular.z = angle_to_target - turtlebot_state.theta;  // Rotate towards target
//     } else {
//       // Stop moving and update to the next target
//       cmd.linear.x = 0.0;
//       cmd.angular.z = 0.0;
      
//       // Move to the next target position in the path
//       current_target_index = (current_target_index + 1) % target_positions.size();
//     }

//     RCLCPP_INFO(this->get_logger(), "Turtlebot state: (x: %f, y: %f, theta: %f)", turtlebot_state.x, turtlebot_state.y, turtlebot_state.theta);
//     publisher_->publish(cmd);
//   }

//   void topic_callback(const turtlesim::msg::Pose &msg) {
//     turtlebot_state.x = msg.x;
//     turtlebot_state.y = msg.y;
//     turtlebot_state.theta = msg.theta;
//   }

//   rclcpp::TimerBase::SharedPtr timer_;
//   rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
//   rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscription_;
//   State turtlebot_state;

//   int current_target_index;
//   std::vector<std::vector<double>> target_positions;
// };


// int main(int argc, char ** argv)
// {
//   rclcpp::init(argc, argv);
//   rclcpp::spin(std::make_shared<TurtlebotController>());
//   rclcpp::shutdown();
//   return 0;
// }

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <turtlesim/msg/pose.hpp>
#include <iostream>

using namespace std::chrono_literals;
using std::placeholders::_1;
const double PI = 3.14;

struct State {
  double x = 0;
  double y = 0;
  double theta = 0;
};

class TurtlebotController : public rclcpp::Node{
  public:
  TurtlebotController() : Node("turtlebot_controller"), step(0){
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel",1);
    subscription_ = this->create_subscription<turtlesim::msg::Pose>("turtle1/pose", 1, std::bind(&TurtlebotController::topic_callback, this, _1));
    timer_ = this->create_wall_timer(100ms, std::bind(&TurtlebotController::timer_callback, this));
  }


  private:
    void check_arrival(double cur_x, double cur_y, double cur_theta, int step) {
        double goal_x, goal_y;
        double init_x = 5.544445, init_y = 5.544445;
        double error_margin = 0.01;
        // double last_x = init_x,last_y = init_y;

        // 목표 위치 설정
        switch (step) {
            case 0:  // 오른쪽으로 이동: (2,0)
                goal_x = 2.0;
                goal_y = 0.0;
                break;
            case 1:  // 위로 이동: (2,2)
                goal_x = 2.0;
                goal_y = 2.0;
                break;
            case 2:  // 왼쪽으로 이동: (0,2)
                goal_x = 0.0;
                goal_y = 2.0;
                break;
            case 3:  // 아래로 이동: (0,0)
                goal_x = 0.0;
                goal_y = 0.0;
                break;
            default:
                return; // 잘못된 단계인 경우 함수를 종료
        }

        // 이동 명령 설정
        geometry_msgs::msg::Twist cmd;

        switch (step) {
            case 0:  // 오른쪽으로 이동
                RCLCPP_INFO(this->get_logger(), "case0");
                if (cur_x - init_x -goal_x<-error_margin) {  // 목표 위치에 도달하지 않은 경우
                    cmd.linear.x = 1*fabs(cur_x - init_x -goal_x);
                    cmd.angular.z = 0.0;
                }
                else if(cur_x - init_x -goal_x>error_margin){
                    cmd.linear.x = -1*fabs(cur_x - init_x -goal_x);
                    cmd.angular.z = 0.0;
                }                    
                else {  // 목표 위치에 도달한 경우 각도 조정
                    cmd.linear.x = 0.0;
                    if (cur_theta-M_PI / 2<-error_margin) {
                        cmd.angular.z = 1*fabs(M_PI/2-cur_theta);
                    }
                    else if (cur_theta-M_PI / 2 >error_margin) {
                        cmd.angular.z = -1*fabs(M_PI/2-cur_theta);
                    } 
                    else {
                        cmd.angular.z = 0.0;
                    }
                }
                break;

            case 1:  // 위로 이동
                RCLCPP_INFO(this->get_logger(), "case1");
                if (cur_y- init_y -goal_y<-error_margin ) {
                    cmd.linear.x = 1*fabs(cur_y- init_y -goal_y);
                    cmd.angular.z = 0.0;
                } 
                else if(cur_y- init_y -goal_y>error_margin ){
                    cmd.linear.x = -1*fabs(cur_y- init_y -goal_y);
                    cmd.angular.z = 0.0;
                }
                else {
                    cmd.linear.x = 0.0;
                    if (cur_theta-M_PI<-error_margin  && cur_theta>0 ) {
                        cmd.angular.z = 1.0*fabs(M_PI-cur_theta);
                    }
                    else if (cur_theta+M_PI>error_margin && cur_theta <0){
                        cmd.angular.z = -1.0*fabs(cur_theta+M_PI);
                    }
                    else {
                        cmd.angular.z = 0.0;
                    }
                }
                break;

            case 2:  // 왼쪽으로 이동
                RCLCPP_INFO(this->get_logger(), "case2");
                if (cur_x -init_x -goal_x>error_margin) {
                    cmd.linear.x = 1*fabs(cur_x -init_x -goal_x);
                    cmd.angular.z = 0.0;
                } 
                else if(cur_x -init_x -goal_x<-error_margin){
                    cmd.linear.x = -1*fabs(cur_x -init_x -goal_x);
                    cmd.angular.z = 0.0;
                }
                else {
                    cmd.linear.x = 0.0;
                    if(cur_theta>=0 && cur_theta-M_PI<=0)
                    {
                        cmd.angular.z = 1*fabs(cur_theta +M_PI);
                    }                   
                    else if (cur_theta<0 && cur_theta +M_PI/2 < -error_margin) {
                        cmd.angular.z = 1*fabs(cur_theta +M_PI/2);
                    } 
                    else if (cur_theta +M_PI/2> error_margin){
                        cmd.angular.z = -1*fabs(cur_theta +M_PI/2);
                    }
                    else {
                        cmd.angular.z = 0.0;
                    }
                }
                break;

            case 3:  // 아래로 이동
                RCLCPP_INFO(this->get_logger(), "case3");
                if (cur_y-init_y -goal_y > error_margin ) {
                    cmd.linear.x = 1*fabs(cur_y-init_y -goal_y);
                    cmd.angular.z = 0.0;
                }
                else if(cur_y-init_y -goal_y < -error_margin ){
                    cmd.linear.x = -1*fabs(cur_y-init_y -goal_y);
                    cmd.angular.z = 0.0;
                } 
                else {
                    cmd.linear.x = 0.0;
                    if (cur_theta < error_margin) {
                        cmd.angular.z = 1*fabs(cur_theta);
                    }
                    else if (cur_theta>-error_margin){
                        cmd.angular.z = -1*fabs(cur_theta);
                    } 
                    else {
                        cmd.angular.z = 0.0;
                    }
                }
                break;
        }

        publisher_->publish(cmd);
    }

    void timer_callback() {
      geometry_msgs::msg::Twist cmd;
    //   cmd.linear.x = 2.0; //v
    //   cmd.angular.z = 1.5; //w

      RCLCPP_INFO(this->get_logger(), "Turtlebot state: (x: %f, y: %f, theta: %f)", turtlebot_state.x,turtlebot_state.y,turtlebot_state.theta);
      publisher_->publish(cmd);
    }
    
    void topic_callback(const turtlesim::msg::Pose &msg){      
      turtlebot_state.x = msg.x;
      turtlebot_state.y = msg.y;
      turtlebot_state.theta = msg.theta;

      double init_x = 5.544445, init_y = 5.544445;
      double error =0.01;

      // RCLCPP_INFO(this->get_logger(), "fmod(turtlebot_state.theta,M_PI): %f", fmod(turtlebot_state.theta,M_PI));
      if (fabs(turtlebot_state.x-init_x) <= error && fabs(turtlebot_state.y-init_y)<= error&& fabs(turtlebot_state.theta)<= error){
        RCLCPP_INFO(this->get_logger(), "step: %i", 0);
        step = 0;
      }
      else if (fabs(turtlebot_state.x -(init_x+2))<= error&& fabs(turtlebot_state.y -(init_y+0))<= error&& fabs(turtlebot_state.theta-M_PI / 2) <= error){
        RCLCPP_INFO(this->get_logger(), "step: %i", 1);
        RCLCPP_INFO(this->get_logger(), "x_error: %f", fabs(turtlebot_state.x -(init_x+2)));
        RCLCPP_INFO(this->get_logger(), "y_error: %f", fabs(turtlebot_state.y -(init_y+2)));
        RCLCPP_INFO(this->get_logger(), "theta_error: %f", fabs(fmod(turtlebot_state.theta,2*M_PI)- M_PI / 2));
        step = 1;
      }
      else if (fabs(turtlebot_state.x -(init_x+2)) <= error && fabs(turtlebot_state.y -(init_y+2))<= error && (fabs(turtlebot_state.theta-M_PI)<= error || fabs(turtlebot_state.theta+M_PI) <=error)){
        RCLCPP_INFO(this->get_logger(), "step: %i", 2);
        RCLCPP_INFO(this->get_logger(), "x_error: %f", fabs(turtlebot_state.x -(init_x+0)));
        RCLCPP_INFO(this->get_logger(), "y_error: %f", fabs(turtlebot_state.y-(init_y+2)));
        RCLCPP_INFO(this->get_logger(), "theta_error: %f", fabs(fmod(turtlebot_state.theta,2*M_PI) - M_PI*3 / 2));
        step = 2;
      }
      else if (fabs(turtlebot_state.x -(init_x+0))<= error && fabs(turtlebot_state.y-(init_y+2)) <= error && fabs(turtlebot_state.theta+M_PI/2) <= error){
        RCLCPP_INFO(this->get_logger(), "step: %i", 3);
        step = 3;
      }
      // if ((turtlebot_state.x <= init_x+error || turtlebot_state.x >= init_x-error )&& (turtlebot_state.y <= init_y+error || turtlebot_state.y >= init_y-error)&& (fmod(turtlebot_state.theta,M_PI) <= 0+error ||fmod(turtlebot_state.theta,M_PI) >= 0-error)){
      //   RCLCPP_INFO(this->get_logger(), "step: %i", 0);
      //   step = 0;
      // }
      // else if ((turtlebot_state.x <= init_x+2 +error || turtlebot_state.x >= init_x+2 -error)&& (turtlebot_state.y <= init_y+0 +error || turtlebot_state.y >= init_y+0 -error)&& (fmod(turtlebot_state.theta,M_PI)<= M_PI / 2 +error || fmod(turtlebot_state.theta,M_PI)>= M_PI / 2 -error )){
      //   RCLCPP_INFO(this->get_logger(), "step: %i", 1);
      //   step = 1;
      // }
      // else if (turtlebot_state.x >= init_x+2 && turtlebot_state.y >= init_y+2 && fmod(turtlebot_state.theta,M_PI) >= M_PI){
      //   RCLCPP_INFO(this->get_logger(), "step: %i", 2);
      //   step = 2;
      // }
      // else if (turtlebot_state.x <= init_x+0 && turtlebot_state.y >= init_y+2 && fmod(turtlebot_state.theta,M_PI) >= M_PI*3 / 2){
      //   RCLCPP_INFO(this->get_logger(), "step: %i", 3);
      //   step = 3;
      // }

      check_arrival(turtlebot_state.x,turtlebot_state.y, turtlebot_state.theta, step);
    }
  
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscription_;
  State turtlebot_state;
  int step;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TurtlebotController>());
  rclcpp::shutdown();
  return 0;
}
