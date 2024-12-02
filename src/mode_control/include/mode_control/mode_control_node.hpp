#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <termios.h>
#include <unistd.h>

enum modetype_t {
    manual_mode,
    autonomous_mode,
};

class ModeControlNode : public rclcpp::Node {
    public:
        ModeControlNode();
        ~ModeControlNode();

        void publish_mode();
    
    private:

        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
        rclcpp::TimerBase::SharedPtr timer_;

        struct termios termios_original;
        struct termios termios_new;
        int mode_type;
};