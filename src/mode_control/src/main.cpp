#include "rclcpp/rclcpp.hpp"
#include "mode_control_node.hpp"
#include <iostream>

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ModeControlNode>());
    rclcpp::shutdown();
    return 0;
}