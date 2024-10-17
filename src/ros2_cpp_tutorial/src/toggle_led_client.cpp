#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/empty.hpp"

class LedToggleClient : public rclcpp::Node
{
public:
    LedToggleClient() : Node("led_toggle_client")
    {
        client_ = this->create_client<std_srvs::srv::Empty>("toggle_led");

        while (!client_->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_INFO(this->get_logger(), "Service not available, waiting...");
        }

        send_request();
    }

private:
    void send_request()
    {
        auto request = std::make_shared<std_srvs::srv::Empty::Request>();

        auto future = client_->async_send_request(request);

        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) ==
            rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_INFO(this->get_logger(), "Service call completed.");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to call service toggle_led");
        }
    }

    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr client_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LedToggleClient>());
    rclcpp::shutdown();
    return 0;
}
