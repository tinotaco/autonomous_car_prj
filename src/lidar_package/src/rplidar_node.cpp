
#include "sl_types.h"
#include "rplidar_node.hpp"



RPLidarNode::RPLidarNode() : rclcpp::Node("rplidar_node") {
    RCLCPP_INFO(this->get_logger(),
    "RPLIDAR running on ROS 2 package rplidar_ros. SDK Version: '%s'", RPLIDAR_SDK_VERSION);
    this->drv = rp::standalone::rplidar::RPlidarDriver::CreateDriver(sl::CHANNEL_TYPE_SERIALPORT);
    
    //Look at this again in future
    if (IS_FAIL(drv->connect((char *)serial_port.c_str(), baudrate, 0))) {
        //This also for some reason doesnt work yet
        RCLCPP_ERROR(this->get_logger(), "Failed to connect to device");
        std::cout << "We here in node generation" << std::endl;
        return;
    }

    if (!drv->isConnected()) {
        // This for some reason doesnt work yet
        RCLCPP_ERROR(this->get_logger(), "Is not connected");
    }


    publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("scan", 10);
    this->tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    // ***** Broadcast LaserScan transform
    //Use Quaternion to set proper Quaternion values
    tf2::Quaternion quat;

    quat.setRPY(0, 0, 0);

    // Message for the Transform
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = this->get_clock()->now();
    tf_msg.header.frame_id = this->parent_frame_id_;
    tf_msg.child_frame_id = this->frame_id_;

    tf_msg.transform.translation.x = 0;
    tf_msg.transform.translation.y = 0;
    tf_msg.transform.translation.z = 0;

    tf_msg.transform.rotation = tf2::toMsg(quat);
    tf_broadcaster_->sendTransform(tf_msg);

    this->startScan();
    timer_ = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&RPLidarNode::publish_loop, this));
}

void RPLidarNode::getDeviceInfo() {
    drv->getDeviceInfo(this->dev_info, drv->DEFAULT_TIMEOUT);
    RCLCPP_INFO(this->get_logger(), "Device Info gotten: \n Model: 0x%x \n Firmware Version: 0x%x \n Hardware Version: 0x%x \n",
                                    dev_info.model,
                                    dev_info.firmware_version,
                                    dev_info.hardware_version);
    return;
}

void RPLidarNode::startScan() {
    drv->startScan(false, true);
}


void RPLidarNode::stopScan() {

}

void RPLidarNode::publish_loop() {
    // Introduce Check to see if in Publish loop mode
    // Here

    // End here
    size_t count = 360 * 8;
    
    auto nodes = std::make_unique<rplidar_response_measurement_node_hq_t[]>(count);

    u_result op_result;
    
    rclcpp::Time start_scan_time = this->now();
    op_result = drv->grabScanDataHq(nodes.get(), count);
    rclcpp::Time end_scan_time = this->now();

    double time_increment = (end_scan_time - start_scan_time).nanoseconds() * 1E-9;
    if (op_result != RESULT_OK) {
        return;
    }

    op_result = drv->ascendScanData(nodes.get(), count);
    RCLCPP_INFO(this->get_logger(), "RESULT IS: 0x%x", op_result);
    if (op_result == RESULT_OK) {
        int i = 0;
        int start_node;
        int end_node;
        float min_angle_val = 195 * 16384.f / 90.f; //Min Angle in Q14 format
        float max_angle_val = 165 * 16384.f / 90.f; //Max angle in Q14 format
        while (nodes[i++].dist_mm_q2 == 0) {}
        int start_valid = i;
        i = count - 1;
        while (nodes[i--].dist_mm_q2 == 0) {}
        int end_valid = i;
        i = 0;
        while (nodes[i++].angle_z_q14 <= min_angle_val) {}
        start_node = i-1;
        i = count - 1;
        while (nodes[i--].angle_z_q14 >= max_angle_val) {}
        end_node = i+1;
        int size = abs(end_node - start_node)+1;
        int valid_node_count;
        int temp_node;
        if (end_node <= start_node) {
            temp_node = start_node;
            start_node = end_node;
            end_node = temp_node;
        }
        auto valid_nodes = std::make_unique<rplidar_response_measurement_node_hq_t[]>(start_node + (end_valid - end_node) + 1);
        //RCLCPP_INFO(this->get_logger(), "Start Node: %i, End Node: %i, End Valid: %i. Array size is: %i", start_node, end_node, end_valid, start_valid + (end_valid - end_node) + 1);
        int y = 0;
        for (int x = end_node; x != start_node; y++) {
            valid_nodes[y] = nodes[x];
            //RCLCPP_INFO(this->get_logger(), "X is: %i, Valid Nodes Angle is: %f, Cound is: %i", x, getAngle(valid_nodes[y]), y);
            x = (x +1)%end_valid;
        }
        valid_node_count = y;
        
        // publish the scan here
        this->publish_scan(time_increment, valid_nodes, valid_node_count);

    }
}

void RPLidarNode::publish_scan(double scan_time, const std::unique_ptr<rplidar_response_measurement_node_hq_t[]>& pub_nodes, int node_count) {
    sensor_msgs::msg::LaserScan scan_msg;

    scan_msg.header.stamp = this->now();
    scan_msg.header.frame_id = frame_id_;
    //RCLCPP_INFO(this->get_logger(), "Angle MAx Before in Degrees: %f", pub_nodes[0].angle_z_q14);
    //RCLCPP_INFO(this->get_logger(), "Angle Min Before in Degrees: %f", pub_nodes[node_count - 1].angle_z_q14);
    double angle_min = deg_2_rad(getAngle(pub_nodes[0]));
    //angle_max = 2*M_PI;
    double angle_max = deg_2_rad(getAngle(pub_nodes[node_count-1]));
    //angle_min = 0;
    RCLCPP_INFO(this->get_logger(), "Min Angle is: %f. Max angle is: %f", getAngle(pub_nodes[0]), getAngle(pub_nodes[node_count-1]));
    RCLCPP_INFO(this->get_logger(), "Angle Max is: %f", angle_max);
    RCLCPP_INFO(this->get_logger(), "Angle Min is: %f", angle_min);
    scan_msg.angle_min = angle_min;
    scan_msg.angle_max = angle_max;

    scan_msg.angle_increment =
    ((2*M_PI - scan_msg.angle_min) + (scan_msg.angle_max) )/ (double)(node_count - 1);
    RCLCPP_INFO(this->get_logger(), "Angle Max: %f, Angle Min: %f, Node count: %i, Increment: %f", scan_msg.angle_max, scan_msg.angle_min, node_count-1, scan_msg.angle_increment);
    scan_msg.scan_time = scan_time;
    scan_msg.time_increment = scan_time / (double)(node_count - 1);

    scan_msg.range_min = 0.15f;
    scan_msg.range_max = 5;

    scan_msg.intensities.resize(node_count);
    scan_msg.ranges.resize(node_count);

    for (size_t i = 0; i < node_count; i++) {
      float read_value = (float) pub_nodes[i].dist_mm_q2 / 4.0f / 1000;
      if (read_value == 0.0) {
        scan_msg.ranges[i] = std::numeric_limits<float>::infinity();
      } else {
        scan_msg.ranges[i] = read_value;
      }
      scan_msg.intensities[i] = (float) (pub_nodes[i].quality >> 2);
    }
    //RCLCPP_INFO(this->get_logger(), "Publishing Scan at: %d with %d points", scan_msg.header.stamp, node_count);
    RCLCPP_INFO(this->get_logger(), "Going to Publish scane");
    publisher_->publish(scan_msg);
}
