
#include "custom_interfaces/srv/motor_control_request_response.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <termios.h>
#include <unistd.h>

class RemoteControlServer : public rclcpp::Node {
    public:

        RemoteControlServer();
        ~RemoteControlServer();

        void subscription_update(const std_msgs::msg::String::SharedPtr msg);

        void service_callback(const std::shared_ptr<custom_interfaces::srv::MotorControlRequestResponse::Request> request,
                                std::shared_ptr<custom_interfaces::srv::MotorControlRequestResponse::Response> response);

        void remote_connect();

        void update_latest_events();
    
    private:
        rclcpp::Service<custom_interfaces::srv::MotorControlRequestResponse>::SharedPtr service_;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
        
        rclcpp::TimerBase::SharedPtr timer_;
        bool is_enabled_;
        struct termios termios_original;
        struct termios termios_new;
        struct libevdev *dev = NULL;

        int speed_percent;
        int steer_angle;

        // This is for sending information that a Slam iteration should be carried out
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisherslammode_;
};
