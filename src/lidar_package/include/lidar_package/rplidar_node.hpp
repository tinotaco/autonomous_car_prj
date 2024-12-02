
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "rplidar.h"
#include "rplidar_cmd.h"
#include <sensor_msgs/msg/laser_scan.hpp>

constexpr double deg_2_rad(double x)
{
  return x * M_PI / 180.0;
}

static float getAngle(const rplidar_response_measurement_node_hq_t& node)
{
    return node.angle_z_q14 * 90.f / 16384.f;
}


class RPLidarNode : public rclcpp::Node {
    public:
        RPLidarNode();

        void getDeviceInfo();

        void startScan();

        void stopScan();

        void publish_loop();

        void publish_scan(double time_increment, const std::unique_ptr<rplidar_response_measurement_node_hq_t[]>& pub_nodes, int node_count);

    private:
        const std::string serial_port = "/dev/ttyUSB0";
        uint32_t baudrate = 115200;
        rp::standalone::rplidar::RPlidarDriver *drv = nullptr;
        sl_lidar_response_device_info_t dev_info;
        std::string scan_mode;
        rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
        std::string frame_id_ = "laser_frame";
        rclcpp::TimerBase::SharedPtr timer_;
};

