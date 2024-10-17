#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
using std::placeholders::_1;

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class FastAgent : public rclcpp::Node
{
  public:
    FastAgent()
    : Node("fast_agent"), count_(0)
    {
      publisher_ = this->create_publisher<std_msgs::msg::String>("fast_topic", 10);
      timer_ = this->create_wall_timer(
        500ms, std::bind(&FastAgent::timer_callback, this));
      subscription_ = this->create_subscription<std_msgs::msg::String>(
        "slow_topic", 10, std::bind(&FastAgent::topic_callback, this, _1));
    }

  private:
    void timer_callback()
    {
      auto message = std_msgs::msg::String();
      message.data = "Fast timer " + std::to_string(count_++);
      RCLCPP_INFO(this->get_logger(), "[Fast] Publishing: '%s'", message.data.c_str());
      publisher_->publish(message);
    }

    void topic_callback(const std_msgs::msg::String & msg) const
    {
      RCLCPP_INFO(this->get_logger(), "[Fast] I heard: '%s'", msg.data.c_str());
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FastAgent>());
  rclcpp::shutdown();
  return 0;
}