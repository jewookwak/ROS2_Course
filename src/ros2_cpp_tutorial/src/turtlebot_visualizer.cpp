#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include <turtlesim/msg/pose.hpp>
#include <vector>
using namespace std::chrono_literals;
using std::placeholders::_1;

struct State {
  double x = 0;
  double y = 0;
  double theta = 0;
};

class TurtlebotVisualizer : public rclcpp::Node {
public:
  TurtlebotVisualizer() : Node("turtlebot_visualizer") {
    publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("turtle1/marker", 10);
    subscription_ = this->create_subscription<turtlesim::msg::Pose>(
        "turtle1/pose", 10, std::bind(&TurtlebotVisualizer::topic_callback, this, _1));
    timer_ = this->create_wall_timer(
        50ms, std::bind(&TurtlebotVisualizer::timer_callback, this));
  }

private:
  std::vector<double> way_x; // make array
  std::vector<double> way_y;
  void timer_callback() {
    visualization_msgs::msg::MarkerArray marker_array;
    // Set marker parameters
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = this->get_clock()->now();
    marker.ns = "marker_namespace";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = turtlebot_state.x;
    marker.pose.position.y = turtlebot_state.y;
    marker.pose.position.z = 0.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = sin(turtlebot_state.theta * 0.5);
    marker.pose.orientation.w = cos(turtlebot_state.theta * 0.5);
    marker.scale.x = 0.3;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    geometry_msgs::msg::Point p;
    for(size_t i=0;i<way_x.size();++i){
      p.x = way_x[i];
      p.y = way_y[i];
      p.z = 0.0;
      // RCLCPP_INFO(this->get_logger(), "point: (x: %f, y: %f, z: %f)", p.x, p.y, p.z);
      marker.points.push_back(p);
      // marker.points.emplace_back(p)
    }
    
    // only if using a MESH_RESOURCE marker type:
    //  marker.mesh_resource = "package://<package_name>/meshes/base_v0/base.dae";

    marker_array.markers.emplace_back(marker);
    publisher_->publish(marker_array);
  }

  void topic_callback(const turtlesim::msg::Pose &msg) {
    turtlebot_state.x = msg.x;
    turtlebot_state.y = msg.y;
    turtlebot_state.theta = msg.theta;
    RCLCPP_INFO(this->get_logger(), "Turtlebot state: (x: %f, y: %f, theta: %f)", turtlebot_state.x, turtlebot_state.y, turtlebot_state.theta);
    way_x.push_back(turtlebot_state.x);
    way_y.push_back(turtlebot_state.y);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_;
  rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscription_;
  State turtlebot_state;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TurtlebotVisualizer>());
  rclcpp::shutdown();
  return 0;
}
