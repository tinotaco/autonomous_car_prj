#include "motorclient/motorclient.hpp"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MotorClient>());
    rclcpp::shutdown();
    return 0;
}