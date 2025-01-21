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

    this->max_row = height * resolution - 1;
    this->max_column = width * resolution - 1;
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
            //int8_t matrix_val = 100 * (1 - (1/std::exp(logodd_matrix(i, j))));
            double explogodd = std::exp(logodd_matrix(i,j));
            int8_t matrix_val = 100 * (explogodd / (1 + explogodd));
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
    // TODO: check if bounding is the way to go
    if (cell_update.occupied) {
        if (logodd_matrix(cell_update.cell.cell_row, cell_update.cell.cell_column) >= 5) {
            return;
        }
        update_val = L_OCC;
    } else {
        if (logodd_matrix(cell_update.cell.cell_row, cell_update.cell.cell_column) <= -5) {
            return;
        }
        update_val = L_FREE;
    }
    RCLCPP_INFO(this->get_logger(), "Updating Cell: (%i, %i)", cell_update.cell.cell_row, cell_update.cell.cell_column);
    logodd_matrix(cell_update.cell.cell_row, cell_update.cell.cell_column) += update_val;
}

bool OccupancyGrid::occupancygrid_state(int &cell_row, int &cell_column) {
    //returns true if occupied (value >= 0.5)
    if (logodd_matrix(cell_row, cell_column) > 0.5) {
        return true;
    } else {
        return false;
    }
}

void OccupancyGrid::update_occupancygridcell_frompoint(coordinates &point, bool occupied) {
    // Get Grid Value from the given coordinate point
    occcell_update update;
    update.cell = this->get_gridcell(point.x, point.y);
    update.occupied = occupied;

    this->update_occupancygridcell(update);
}


void OccupancyGrid::mapfill_laserpoint(coordinates laserpoint, const geometry_msgs::msg::Pose &pose) {
    //Bresenham Line Algorithm (only works for integers)
    //RCLCPP_INFO(this->get_logger(), "Laserpoint is: (x,y)->(%f, %f)", laserpoint.x, laserpoint.y);
    gridcell laserpoint_cell = this->get_gridcell(laserpoint.x, laserpoint.y);
    gridcell pose_cell = this->get_gridcell(pose.position.x, pose.position.y);
    //RCLCPP_INFO(this->get_logger(), "Grid Laserpoint: (x,y)->(%i, %i), Pose Grid: (x,y)->(%i, %i)", laserpoint_cell.cell_column, laserpoint_cell.cell_row, pose_cell.cell_column, pose_cell.cell_row);
    occcell_update update;
    int dx = laserpoint_cell.cell_column - pose_cell.cell_column;
    int dy = laserpoint_cell.cell_row - pose_cell.cell_row;
    RCLCPP_INFO(this->get_logger(), "dx is: %i, dy is: %i", dx, dy);
    
    if (dx == 0) {
        //Vertical, fill in lines to the top/bottom until reach laser endpoint
        //TODO
        return;
    }
    if (dy == 0) {
        //Horizontal, fill in lines to the side until reach laser endpoint
        //TODO
        return;
    }
    int increment = 0;

    // *****************************
    auto bresenham = [&update, increment, this](int ind_0, int ind_1, int dep_0, int ind_delt, int dep_delt, int &updatecell_dep, int &updatecell_ind) {
        int dep_curr = dep_0;
        int D = 2*dep_delt - ind_delt;
        for (int i = ind_0; i < ind_1; i++) {
            if (D>0) {
                dep_curr += increment;
                D += 2*(dep_delt - ind_delt);
            } else {
                D += 2*dep_delt;
            }
            updatecell_dep = dep_curr;
            updatecell_ind = i;
            this->update_occupancygridcell(update);
        }
    };

    if (abs(dy) < abs(dx)) {
        if (dy < 0) {
            increment = -1;
            //negative ind_delt needs to be passed in these cases
            if (dx<0) {
                bresenham(laserpoint_cell.cell_column, pose_cell.cell_column, laserpoint_cell.cell_row, dx, -dy, update.cell.cell_column, update.cell.cell_row);
            } else {
                bresenham(pose_cell.cell_column, laserpoint_cell.cell_column, pose_cell.cell_row, -dx, dy, update.cell.cell_column, update.cell.cell_row);
            }
        } else {
            increment = 1;
            if (dx<0) {
                bresenham(laserpoint_cell.cell_column, pose_cell.cell_column, laserpoint_cell.cell_row, -dx, -dy, update.cell.cell_column, update.cell.cell_row);
            } else {
                bresenham(pose_cell.cell_column, laserpoint_cell.cell_column, pose_cell.cell_row, dx, dy, update.cell.cell_column, update.cell.cell_row);
            }

        }
    } else {
        if (dx < 0) {
            //negative ind_delt needs to be passed in these cases
            increment = -1;
            if (dy < 0) {
                bresenham(laserpoint_cell.cell_row, pose_cell.cell_row, laserpoint_cell.cell_column, dy, -dx, update.cell.cell_row, update.cell.cell_column);
            } else {
                bresenham(pose_cell.cell_row, laserpoint_cell.cell_row, pose_cell.cell_column, -dy, dx, update.cell.cell_row, update.cell.cell_column);
            }
        } else {
            increment = 1;
            if (dy<0) {
                bresenham(laserpoint_cell.cell_row, pose_cell.cell_row, laserpoint_cell.cell_column, -dy, -dx, update.cell.cell_row, update.cell.cell_column);
            } else {
                bresenham(pose_cell.cell_row, laserpoint_cell.cell_row, pose_cell.cell_column, dy, dx, update.cell.cell_row, update.cell.cell_column);
            }
        }
    }

    // ***************************

    /*
    if (abs(dy) < abs(dx)) {
        update.occupied = false;
        //Case that x is increased/decreased, y needs to be found because slope is not larger than 1
        if (dy < 0) {
            increment = -1;
            dy = -dy;
        } else {
            increment = 1;
        }
        step = (dx<0) ? -1 : 1;
        int D = 2 * dy - dx;
        int y = pose_cell.cell_row;
        RCLCPP_INFO(this->get_logger(), "We here 3a: (%i, %i)", laserpoint_cell.cell_row, laserpoint_cell.cell_column);
        for (int i = pose_cell.cell_column; i != laserpoint_cell.cell_column; i+=step) {
            if (D > 0) {
                y += increment;
                D += 2*(dy-dx);
            } else {
                D = D + 2*dy;
            }
            //TODO: also check this below if this is correct
            update.cell = {i, y};
            //RCLCPP_INFO(this->get_logger(), "Updating Cell: (row, column)->(%i, %i)", y, i);
            this->update_occupancygridcell(update);
        }
    } else {
        // Case that y is increased/decreased, x needs to be found because slope is larger than 1
        if (dx < 0) {
            increment = -1;
            dx = -dx;
        } else {
            increment = 1;
        }
        step = (dy < 0) ? -1 : 1;
        int D = 2 * dx - dy;
        int x = pose_cell.cell_column;
        RCLCPP_INFO(this->get_logger(), "We here 3b: (%i, %i)", laserpoint_cell.cell_row, laserpoint_cell.cell_column);
        RCLCPP_INFO(this->get_logger(), "dx is: %i", dx);
        for (int i = pose_cell.cell_row; i != laserpoint_cell.cell_row; i+=step) {
            RCLCPP_INFO(this->get_logger(), "Updating Row: %i of %i", i, laserpoint_cell.cell_row);
            RCLCPP_INFO(this->get_logger(), "Value of D is: %i", D);
            if (D > 0) {
                x += increment;
                D += 2*(dx-dy);
            } else {
                D = D + 2*dx;
            }
            //TODO: also check this below if this is correct
            update.cell = {x, i};
            //RCLCPP_INFO(this->get_logger(), "Updating Cell: (row, column)->(%i, %i)", i, x);
            this->update_occupancygridcell(update);
        }
    }
    */

    
    update.occupied = true;
    update.cell = {laserpoint_cell.cell_column, laserpoint_cell.cell_row};

    this->update_occupancygridcell(update);
    return;
}

void OccupancyGrid::update_occupancymap(const std::vector<coordinates> global_laserscan, const geometry_msgs::msg::Pose &pose) {
    for (int i=0; i < (int)global_laserscan.size(); i++) {
    //for (int i=0; i < 5; i++) {
        //Update Laser Endpoints as Occupied Points
        RCLCPP_INFO(this->get_logger(), "%i/%i Global Laserscan is: %f, %f", i, (int)global_laserscan.size(), global_laserscan[i].x, global_laserscan[i].y);
        this->mapfill_laserpoint(global_laserscan[i], pose);
    }
}

gridcell OccupancyGrid::get_gridcell(double x, double y) {
    //RCLCPP_INFO(this->get_logger(), "X is: %f; y is: %f", x, y);
    //TODO: Need to switch back columns for rows and see how this works out
    return {(int)((y+(double)this->width/2) * this->resolution), (int)((x + (double)this->height/2) * this->resolution)};
}
