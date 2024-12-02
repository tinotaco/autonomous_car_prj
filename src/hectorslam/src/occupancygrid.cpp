#include "occupancygrid.hpp"
#include <cstdint>
#include <math.h>

#define L_OCC (2.197)
#define L_FREE (-0.8472)


OccupancyGrid::OccupancyGrid(int resolution, int height, int width) : Node("occupancygrid") {
    // Init Val for the occupancy grid
    this->init_val = 0.5;

    // Resolution in cell/m, height and width in meters
    this->logodd_matrix = Eigen::MatrixXd::Constant(resolution*height, resolution*width, this->init_val);
    logodd_matrix(resolution*height-1, resolution*width-1) = 0;
    
    publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("occupancygridvals", 10);
    this->resolution = resolution;
    this->height = height;
    this->width = width;
    timer_ = this->create_wall_timer(std::chrono::milliseconds(5000), std::bind(&OccupancyGrid::publish_occupancygrid, this));
}

void OccupancyGrid::publish_occupancygrid() {
    nav_msgs::msg::OccupancyGrid msg;
    int cells_row = resolution*width;
    int cells_column = resolution*height;

    //RCLCPP_INFO(this->get_logger(), "Wer here. Cells Row: %i, Cells Column: %i", cells_row, cells_column);
    for (int i = 0; i<cells_row; i++) {
        for (int j = 0; j<cells_column; j++) {
            //RCLCPP_INFO(this->get_logger(), "Data at: %i, Value: %i", i*(cells_row)+j, (int)logodd_matrix(i,j));
            int8_t matrix_val = 100 * (1 - (1/std::exp(logodd_matrix(i, j))));
            msg.data.push_back(matrix_val);
        }
    }
    msg.header.frame_id = "base_link";
    msg.header.stamp = this->get_clock()->now();
    msg.info.resolution = 1/(float32_t)resolution;
    msg.info.width = (uint32_t)cells_column;
    msg.info.height = (uint32_t)cells_row;

    msg.info.origin.position.x = -this->width/2;
    msg.info.origin.position.y = -this->height/2;
    msg.info.origin.position.z = 0;

    msg.info.origin.orientation.x = 0;
    msg.info.origin.orientation.y = 0;
    msg.info.origin.orientation.z = 0;
    msg.info.origin.orientation.w = 1;

    RCLCPP_INFO(this->get_logger(), "Publishing Occupancy Grid");
    publisher_->publish(msg);
}

void OccupancyGrid::update_occupancygridcell(occcell_update &cell_update) {
    float update_val;
    if (cell_update.occupied) {
        update_val = L_OCC;
    } else {
        update_val = L_FREE;
    }
    logodd_matrix(cell_update.cell_row, cell_update.cell_column) += update_val;
}
