#include "mode_control_node.hpp"
#include <fcntl.h>

ModeControlNode::ModeControlNode() : Node("mode_control"), mode_type(manual_mode) {
    publisher_ = this->create_publisher<std_msgs::msg::String>("/modecontrol", 10);

    timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&ModeControlNode::publish_mode, this));

    tcgetattr(STDIN_FILENO, &termios_original);
    termios_new = termios_original;
    termios_new.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &termios_new);

    RCLCPP_INFO(this->get_logger(), "Setup Mode Control Node complete");

    //Code for nonblocking of read function
    //int fcntlflags = fcntl(STDIN_FILENO, F_GETFL, 0);
    //fcntl(STDIN_FILENO, F_SETFL, fcntlflags | O_NONBLOCK);
}

ModeControlNode::~ModeControlNode() {
    tcsetattr(STDIN_FILENO, TCSANOW, &termios_original);
}

void ModeControlNode::publish_mode() {
    RCLCPP_INFO(this->get_logger(), "In Publish mode function");
    char key;
    auto ros_msg = std_msgs::msg::String();
    if (read(STDIN_FILENO, &key, 1) > 0) {
        if (key == 'a') {
            mode_type = autonomous_mode;
        }
        if (key == 'm') {
            mode_type = manual_mode;
        }
        tcflush(STDIN_FILENO, TCIFLUSH);
    }
    std::string msg;
    switch (mode_type) {
        case manual_mode:
            msg = "manual";
            break;
        case autonomous_mode:
            msg = "autonomous";
            break;
    }
    ros_msg.data = msg;
    RCLCPP_INFO(this->get_logger(), "Publishing Node with Mode: %s", msg.c_str());
    publisher_->publish(ros_msg);
}

