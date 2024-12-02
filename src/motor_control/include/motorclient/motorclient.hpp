#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "custom_interfaces/srv/motor_control_request_response.hpp"

class MotorClient : public rclcpp::Node {
    public:
        MotorClient();

        void subscription_update(const std_msgs::msg::String::SharedPtr msg);

        void controlinputs_get();
    private:
        std::string mode;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
        rclcpp::Client<custom_interfaces::srv::MotorControlRequestResponse>::SharedPtr client_;
        rclcpp::TimerBase::SharedPtr timer_;
        std::string motorcontrolport = "/dev/ttyACM0";
        int fd_motorcontrol = -1;
};