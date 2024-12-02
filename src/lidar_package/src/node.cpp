
#include "rplidar_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include <iostream>

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RPLidarNode>());
    rclcpp::shutdown();
    return 0;
}

