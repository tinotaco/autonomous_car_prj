#include "manualcontrolserver/remotecontrol_node.hpp"

#include <linux/input.h>
#include <fcntl.h>
#include <libevdev-1.0/libevdev/libevdev.h>

RemoteControlServer::RemoteControlServer() : rclcpp::Node("remote_control_server"), is_enabled_(true) {
    subscription_ = this->create_subscription<std_msgs::msg::String>("/modecontrol", 10, std::bind(&RemoteControlServer::subscription_update, this, std::placeholders::_1));
    service_ = this->create_service<custom_interfaces::srv::MotorControlRequestResponse>("/motorcontrol", std::bind(&RemoteControlServer::service_callback, this, std::placeholders::_1, std::placeholders::_2));

    remote_connect();
    timer_ = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&RemoteControlServer::update_latest_events, this));

    RCLCPP_INFO(this->get_logger(), "Setup Mode Control Node completes");
    
    // This is for sending information that a Slam iteration should be carried out
    publisherslammode_ = this->create_publisher<std_msgs::msg::String>("/slammodepublish", 10);
}

RemoteControlServer::~RemoteControlServer() {
   //tcsetattr(STDIN_FILENO, TCSANOW, &termios_original);
}

void RemoteControlServer::subscription_update(const std_msgs::msg::String::SharedPtr msg) {
    if (msg->data == "manual") {
        is_enabled_ = true;
    } else {
        is_enabled_ = false;
    }
    RCLCPP_INFO(this->get_logger(), "Received Topic: is_enabled_ [%i]", is_enabled_);
}


void RemoteControlServer::service_callback(const std::shared_ptr<custom_interfaces::srv::MotorControlRequestResponse::Request> request,
                                std::shared_ptr<custom_interfaces::srv::MotorControlRequestResponse::Response> response) {
    (void)request;
    RCLCPP_INFO(this->get_logger(), "Service Callback has been called");
    if (is_enabled_) {
        response->speed_percent = this->speed_percent;
        response->steering_angle = this->steer_angle;
    }
    return;
}


void RemoteControlServer::remote_connect() {
    int fd;
    int rc = 1;

    fd = open("/dev/input/event2", O_RDONLY | O_NONBLOCK);
    if (fd == -1) {
        RCLCPP_INFO(this->get_logger(), "Failed to open file");
    }
    rc = libevdev_new_from_fd(fd, &this->dev);
    if (rc < 0) {
        RCLCPP_INFO(this->get_logger(), "Failed to init libevdev (%s)\n", strerror(-rc));
    }

    RCLCPP_INFO(this->get_logger(), "Input Device Name is: %s", libevdev_get_name(dev));
}


void RemoteControlServer::update_latest_events() {
    struct input_event ev;
    int rc = libevdev_next_event(dev, LIBEVDEV_READ_FLAG_NORMAL, &ev);
    auto msg = std_msgs::msg::String();
    while (rc == LIBEVDEV_READ_STATUS_SUCCESS) {
        //RCLCPP_INFO(this->get_logger(), "Type: %i, Code: %i", ev.type, ev.code);
        switch (ev.type) {
            case EV_ABS:
                switch(ev.code) {
                    case ABS_X:
                        this->steer_angle = ((int)ev.value - 120) / 3;

                        RCLCPP_INFO(this->get_logger(), "Updating steer angle");
                        break;
                    case ABS_Y:
                        this->speed_percent = (255 - 2*ev.value);
                        RCLCPP_INFO(this->get_logger(), "Updating speed percent");
                        break;
                    default:
                        break;
                }
                break;
            case EV_KEY:
                switch (ev.code) {
                    case BTN_NORTH:
                        msg.data = "slam";
                        if (ev.value == 1) {
                            RCLCPP_INFO(this->get_logger(), "Triangle has been pressed");
                            this->publisherslammode_->publish(msg);
                        }
                }

            default:
                break;
        }
    rc = libevdev_next_event(dev, LIBEVDEV_READ_FLAG_NORMAL, &ev);
    }
    RCLCPP_INFO(this->get_logger(), "Ending Updating event");
}