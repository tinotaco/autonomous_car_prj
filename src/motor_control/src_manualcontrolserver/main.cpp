#include "rclcpp/rclcpp.hpp"
#include "manualcontrolserver/remotecontrol_node.hpp"


int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RemoteControlServer>());
    rclcpp::shutdown();
    return 0;
}