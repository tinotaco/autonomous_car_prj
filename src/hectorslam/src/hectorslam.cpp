#include "hectorslam.hpp"

HectorSlamNode::HectorSlamNode(int resolution, int height, int width) : rclcpp::Node("hectorslamnode"), OccupancyGrid(resolution, height, width) {
    this->subscriberslamtrigger_ = this->create_subscription<std_msgs::msg::String>("/slammodepublish", 10, std::bind(&HectorSlamNode::slam_iteration, this, std::placeholders::_1));

    this->subscriberlaserscan_ = this->create_subscription<sensor_msgs::msg::LaserScan>("scan", 10, std::bind(&HectorSlamNode::get_newest_laserscan, this, std::placeholders::_1));

    this->publisherpose_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/pose_rc", 10);
    this->tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    this->set_posemsg(0, 0, 0, this->pose);
}

void HectorSlamNode::slam_iteration(const std_msgs::msg::String::SharedPtr msg) {
    (void)msg;
    auto start = this->get_clock()->now();
    RCLCPP_INFO(this->get_logger(), "STARTING NEW SLAM ITERATION at: %ld (nanoseconds)", start.nanoseconds());
    update_occupancymap(this->latest_laserscan, this->pose);
    auto end = this->get_clock()->now();
    RCLCPP_INFO(this->get_logger(), "ENDING SLAM ITERATION at: %ld (nanoseconds)", end.nanoseconds());
}

void HectorSlamNode::get_newest_laserscan(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    this->latest_laserscan.clear();
    int laserscan_size = msg->ranges.size();
    pose_laserpoint new_point;
    float angle_temp = msg->angle_min;
    coordinates point; 
    for (int i = 0; i < laserscan_size; i++) {
        new_point.rel_coord.x = msg->ranges[i] * std::cos(angle_temp);
        new_point.rel_coord.y = -msg->ranges[i] * std::sin(angle_temp);
        point = this->transform_toglobalcoord(this->pose, new_point);
        if (point.x > -this->width/2 && point.y > -this->height/2 && point.x < this->width/2 && point.y < this->height/2) {
            //If the point is plausible (in boundary), it gets added to the latest laserscan points
            this->latest_laserscan.push_back(point);
        }
        //RCLCPP_INFO(this->get_logger(), "New point: %f, %f", point.x, point.y); //REMOVE
        angle_temp+= msg->angle_increment;
    }
}

/*
void HectorSlamNode::update_map(const std::vector<pose_laserpoint> &scan, const geometry_msgs::msg::Pose &pose) {
    (void)pose;
    occcell_update update_point;
    coordinates point;
    //Update only the cells that are occupied
    for (int i = 0; i< (int)scan.size(); i++) {
        //transform to global point using the current pose
        point = transform_toglobalcoord(this->pose, scan[i]);
        
        // get occupancy grid cell to update
        
        // getoccupancygrid_update(point, true, update_point);

        //Perform Bresenham's Line Algorithm for each laserscan point
    }

    // *** Start Filling in Free Spaces
    // Start with Pose square


    //update_point = pose_point;
    lambda_diagonalupdate();
    //Do from 0-45 degrees:
}
*/
/*
bool HectorSlamNode::getoccupancygrid_update(coordinates &point, bool occupied, occcell_update &map_update) {
    //Conversion into Occupancy Grid reference
    //RCLCPP_INFO(this->get_logger(), "X is: %f, y is: %f", point.x, point.y);

    float x = point.x + this->width / 2;
    float y = point.y + this->height / 2;

    // This can be optimized
    if (x >= this->width || x<= 0 || y <= 0 || y>= this->height) {
        //RCLCPP_INFO(this->get_logger(), "Passing this point: x is: %f, y is: %f, max width: %i, max height: %i", x, y, this->width/2, this->height/2);
        return false;
    }
    //assertm(x >= this->width || y >= this->height || x<0 || y<0, "Converted Occupancy Grid Point (%d, %d) not in allowable (width,height)->(%d, %d)", x, y, this->width, this->height);

    // Now need to get the matrix rowƒ
    int grid_cell_x = (int)(this->resolution * x);
    int grid_cell_y = (int)(this->resolution * y);

    //RCLCPP_INFO(this->get_logger(), "Updating Grid Cell x: %i, y: %i", grid_cell_x, grid_cell_y);

    map_update = {grid_cell_x, grid_cell_y, occupied};

    // Update the Map
    this->update_occupancygridcell(map_update);

    return true;
}
*/