#include "motorclient/motorclient.hpp"
#include "fcntl.h"
#include <thread>
#include <chrono>
#include <termios.h>
#include <unistd.h>


MotorClient::MotorClient() : Node("motorclient_node"), mode("manual") {
    
    subscription_ = this->create_subscription<std_msgs::msg::String>("/modecontrol", 10, std::bind(&MotorClient::subscription_update, this, std::placeholders::_1));
    client_ = this->create_client<custom_interfaces::srv::MotorControlRequestResponse>("/motorcontrol");
    timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&MotorClient::controlinputs_get, this));
    while (fd_motorcontrol < 0) {
        fd_motorcontrol = open(this->motorcontrolport.c_str(), O_RDWR | O_NOCTTY);
        if (fd_motorcontrol < 0) {
            RCLCPP_INFO(this->get_logger(), "Connecting to Arduino");
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    struct termios tty;

    if (tcgetattr(fd_motorcontrol, &tty) < 0) {
        RCLCPP_INFO(this->get_logger(), "Not able to get tcgetattr");
    }

    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cflag |= CREAD | CLOCAL;
    tty.c_lflag &= ~ECHO; // Disable echo
    tty.c_lflag &= ~ECHOE; // Disable erasure
    tty.c_lflag &= ~ECHONL; // Disable new-line echo
    tty.c_lflag &= ~ISIG;
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL);
    tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
    tty.c_oflag &= ~ONLCR;

    cfsetispeed(&tty, B9600);
    cfsetospeed(&tty, B9600);
    if (tcsetattr(fd_motorcontrol, TCSANOW, &tty) != 0) {
        RCLCPP_INFO(this->get_logger(), "Problems setting attribute");
    }
    //std::string write_val = "<120,0>";
    //int resp = write(fd, write_val.c_str(), write_val.length());
    //if (resp <= 0) {
    //    RCLCPP_INFO(this->get_logger(), "Has not sent any data");
    //}
    //tcflush(fd, TCIFLUSH);


    while (!client_->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
            RCLCPP_INFO(this->get_logger(), "Not able to find service. And also interrupted. Exiting");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Waiting again for service");
    }

    RCLCPP_INFO(this->get_logger(), "Initialized the MotorClient Node");
}

void MotorClient::subscription_update(const std_msgs::msg::String::SharedPtr msg) {
    if (msg->data == "manual") {
        mode = "manual";
    } else if (msg->data == "autonomous") {
        mode = "autonomous";
    } else {
        mode = "none";
    }
}

void MotorClient::controlinputs_get() {
    if (mode == "manual") {
        auto request = std::make_shared<custom_interfaces::srv::MotorControlRequestResponse::Request>();

        auto response_callback = [this](rclcpp::Client<custom_interfaces::srv::MotorControlRequestResponse>::SharedFuture future) {
            auto response = future.get();
            RCLCPP_INFO(this->get_logger(), "Steering angle is: [%d]. Speed is: [%d]", response->steering_angle, response->speed_percent);
            std::string write_val = "<" + std::to_string(response->steering_angle) + "," + std::to_string(response->speed_percent) + ">"; 
            write(fd_motorcontrol, write_val.c_str(), write_val.length());
        };
        auto response = client_->async_send_request(request, response_callback);
        
    } else {
        return; //This is for autonomous control
    }
}