#include "rclcpp/rclcpp.hpp"
#include "hectorslam.hpp"


int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HectorSlamNode>(5, 20, 20));
    rclcpp::shutdown();
    return 0;
}