#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/empty.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

class LedToggleService : public rclcpp::Node
{
public:
    LedToggleService() : Node("led_toggle_service"), led_on_(false)
    {
        service_ = this->create_service<std_srvs::srv::Empty>(
            "toggle_led", std::bind(&LedToggleService::toggle_led_callback, this, _1, _2));
        RCLCPP_INFO(this->get_logger(), "LED Toggle Service Ready");
    }

private:
    void toggle_led_callback(
        const std::shared_ptr<std_srvs::srv::Empty::Request> request,
        std::shared_ptr<std_srvs::srv::Empty::Response> response)
    {
        (void)request; // Empty request, ignore it
        (void)response; // Empty response, ignore it

        led_on_ = !led_on_;
        std::string status = led_on_ ? "ON" : "OFF";
        RCLCPP_INFO(this->get_logger(), "LED is now %s", status.c_str());
    }

    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr service_;
    bool led_on_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LedToggleService>());
    rclcpp::shutdown();
    return 0;
}
