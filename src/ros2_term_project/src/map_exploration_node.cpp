#include "map_exploration_env.hpp"

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<exploration::RobotEnvironment>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}