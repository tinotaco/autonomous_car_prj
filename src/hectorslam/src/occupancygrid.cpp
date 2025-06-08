#include "occupancygrid.hpp"
#include <cstdint>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/msg/quaternion.hpp>
#include <math.h>

#define L_OCC 2//(2.197)
#define L_FREE -2//(-0.8472)


OccupancyGrid::OccupancyGrid(int resolution, int height, int width) : Node("occupancygrid") {
    // Init Val for the occupancy grid
    this->init_val = 0;

    // Resolution in cell/m, height and width in meters
    this->logodd_matrix = Eigen::MatrixXd::Constant(resolution*height, resolution*width, this->init_val);
    logodd_matrix(0, 0) = 2;
    
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
    //Occupancy grid message in row-major order. X should be columns while Y should be rows
    for (int i = 0; i<cells_row; i++) {
        for (int j = 0; j<cells_column; j++) {
            //RCLCPP_INFO(this->get_logger(), "Data at: %i, Value: %i", i*(cells_row)+j, (int)logodd_matrix(i,j));
            //int8_t matrix_val = 100 * (1 - (1/std::exp(logodd_matrix(i, j))));
            double explogodd = std::exp(logodd_matrix(i,j));
            int8_t matrix_val = 100 * (explogodd / (1 + explogodd));
            //RCLCPP_INFO(this->get_logger(), "Data at: %i, Value: %i. Matrix val: %i", i*(cells_row)+j, (int)logodd_matrix(i,j), matrix_val);
            msg.data.push_back(matrix_val);
        }
    }
    msg.header.frame_id = "base_link";
    msg.header.stamp = this->get_clock()->now();
    msg.info.resolution = 1/(float32_t)resolution;
    msg.info.width = (uint32_t)cells_column;
    msg.info.height = (uint32_t)cells_row;

    msg.info.origin.position.x = -this->width/2 - 1/(2.*resolution);
    msg.info.origin.position.y = -this->height/2 - 1/(2.*resolution);
    msg.info.origin.position.z = 0;
    tf2::Quaternion quat;
    quat.setRPY(0, 0, 0);
    msg.info.origin.orientation = tf2::toMsg(quat);

    //RCLCPP_INFO(this->get_logger(), "Publishing Occupancy Grid");
    publisher_->publish(msg);
}

void OccupancyGrid::update_occupancygridcell(occcell_update &cell_update, Eigen::MatrixXd &grid) {
    float update_val;
    //RCLCPP_INFO(this->get_logger(), "Updating cell row: %i, column: %i", cell_update.cell.cell_row, cell_update.cell.cell_column);
    if (cell_update.occupied) {
        if (grid(cell_update.cell.cell_row, cell_update.cell.cell_column) >= 10) {
            return;
        }
        update_val = L_OCC;
    } else {
        // Skip over point if a previously occupied value can be detected or if the free value is below a certain boundary
        float gridval = grid(cell_update.cell.cell_row, cell_update.cell.cell_column);
        if (gridval <= -10 || gridval > 0) {
            return;
        }
        update_val = L_FREE;
    }
    //RCLCPP_INFO(this->get_logger(), "Updating Cell: (%i, %i)", cell_update.cell.cell_row, cell_update.cell.cell_column);
    grid(cell_update.cell.cell_row, cell_update.cell.cell_column) += update_val;
}

bool OccupancyGrid::occupancygrid_state(int &cell_row, int &cell_column) {
    //returns true if occupied (value >= 0.5)
    if (logodd_matrix(cell_row, cell_column) > 0.5) {
        return true;
    } else {
        return false;
    }
}

void OccupancyGrid::mapfill_laserpointedge(coordinates laserpoint, Eigen::MatrixXd &grid) {
    gridcell laserpoint_cell = this->get_gridcell(laserpoint.x, laserpoint.y);

    occcell_update update;

    update.occupied = true;
    update.cell = {laserpoint_cell.cell_column, laserpoint_cell.cell_row};
    //RCLCPP_INFO(this->get_logger(), "Laserpoint. Coord: x: %f, %f, Cell column: %i, row: %i", laserpoint.x, laserpoint.y, laserpoint_cell.cell_column, laserpoint_cell.cell_row);
    this->update_occupancygridcell(update, grid);
    return;
}

void OccupancyGrid::mapfill_laserpointfreespace(coordinates laserpoint, const twodpose_t &pose, Eigen::MatrixXd &grid) {
    //Bresenham Line Algorithm (only works for integers)
    gridcell laserpoint_cell = this->get_gridcell(laserpoint.x, laserpoint.y);
    gridcell pose_cell = this->get_gridcell(pose.x, pose.y);

    int dx = laserpoint_cell.cell_column - pose_cell.cell_column;
    int dy = laserpoint_cell.cell_row - pose_cell.cell_row;
    
    int increment = 0;
    occcell_update update;
    // *****************************
    auto bresenham = [&update, &increment, &grid, this](int ind_0, int ind_1, int dep_0, int ind_delt, int dep_delt, int &updatecell_ind, int &updatecell_dep, bool reversed_points) {
        //RCLCPP_INFO(this->get_logger(), "Laserpoint: ind_0: %i, ind_1: %i, dep_0: %i, ind_delt: %i, dep_delt: %i, increment: %i", ind_0, ind_1, dep_0, ind_delt, dep_delt, increment);
        int dep_curr = dep_0;
        int D = 2*dep_delt - ind_delt;
        int i = ind_0;
        // If 
        if (reversed_points) {
            if (D>0) {
                dep_curr += increment;
                D += 2*(dep_delt - ind_delt);
            } else {
                D += 2*dep_delt;
            }
        }
        for (; i < ind_1; i++) {
            if (D>0) {
                dep_curr += increment;
                D += 2*(dep_delt - ind_delt);
            } else {
                D += 2*dep_delt;
            }
            updatecell_dep = dep_curr;
            updatecell_ind = i;
            update.occupied = false;
            this->update_occupancygridcell(update, grid);
        }
    };
    
    if (abs(dy) < abs(dx)) {
        if (dy < 0) {
            if (dx<0) {
                increment = 1;
                bresenham(laserpoint_cell.cell_column, pose_cell.cell_column, laserpoint_cell.cell_row, -dx, -dy, update.cell.cell_column, update.cell.cell_row, true);
            } else {
                increment = -1;
                bresenham(pose_cell.cell_column, laserpoint_cell.cell_column, pose_cell.cell_row, dx, -dy, update.cell.cell_column, update.cell.cell_row, false);
            }
        } else {
            if (dx<0) {
                increment = -1;
                bresenham(laserpoint_cell.cell_column, pose_cell.cell_column, laserpoint_cell.cell_row, -dx, dy, update.cell.cell_column, update.cell.cell_row, true);
            } else {
                increment = 1;
                bresenham(pose_cell.cell_column, laserpoint_cell.cell_column, pose_cell.cell_row, dx, dy, update.cell.cell_column, update.cell.cell_row, false);
            }

        }
    } else {
        if (dx < 0) {
            if (dy < 0) {
                increment = 1;
                bresenham(laserpoint_cell.cell_row, pose_cell.cell_row, laserpoint_cell.cell_column, -dy, -dx, update.cell.cell_row, update.cell.cell_column, true);
            } else {
                increment = -1;
                bresenham(pose_cell.cell_row, laserpoint_cell.cell_row, pose_cell.cell_column, dy, -dx, update.cell.cell_row, update.cell.cell_column, false);
            }
        } else {
            if (dy<0) {
                increment = -1;
                bresenham(laserpoint_cell.cell_row, pose_cell.cell_row, laserpoint_cell.cell_column, -dy, dx, update.cell.cell_row, update.cell.cell_column, true);
            } else {
                increment = 1;
                bresenham(pose_cell.cell_row, laserpoint_cell.cell_row, pose_cell.cell_column, dy, dx, update.cell.cell_row, update.cell.cell_column, false);
            }
        }
    } 
    // ***************************
}

void OccupancyGrid::update_occupancymap(const std::vector<coordinates> global_laserscan, const twodpose_t &pose) {
    Eigen::MatrixXd temp_matrix = Eigen::MatrixXd::Constant(this->resolution*this->height, this->resolution*this->width, 0);
    for (int i=0; i < (int)global_laserscan.size(); i++) {
        this->mapfill_laserpointedge(global_laserscan[i], temp_matrix);
    }
    for (int j=0; j < (int)global_laserscan.size(); j++) {
        this->mapfill_laserpointfreespace(global_laserscan[j], pose, temp_matrix);
    }
    this->update_overlayoccupancygrid(temp_matrix);
}

void OccupancyGrid::update_overlayoccupancygrid(const Eigen::MatrixXd &grid) {
    for (int i = 0; i < logodd_matrix.rows(); i++) {
        for (int j = 0; j < logodd_matrix.cols(); j++) {
            float new_val = logodd_matrix(i, j) + grid(i, j);
            if (new_val > 5) {
                logodd_matrix(i,j) = 5;
            } else if (new_val <= -5) {
                logodd_matrix(i,j) = -5;
            } else {
                logodd_matrix(i, j) = new_val;
                continue;
            }
        }
    }
}

gridcell OccupancyGrid::get_gridcell(float x, float y) {
    //RCLCPP_INFO(this->get_logger(), "X is: %f; y is: %f", x, y);
    return {(int)((x + (float)this->width/2) * this->resolution + 0.5), (int)((y + (float)this->height/2)*this->resolution + 0.5)}; // the 0.5 for the rounding
}

gridedges OccupancyGrid::get_pointgrid_edges(coordinates pt) {
    //float x_gridref = (pt.x + this->width/2) * this->resolution;
    //float y_gridref = (pt.y + this->height/2) * this->resolution;
    gridcell pointgridref = get_gridcell(pt.x, pt.y);

    float P00, P01, P10, P11;
    float x_g0, x_g1, y_g0, y_g1;

    x_g0 = pointgridref.cell_column;
    x_g1 = pointgridref.cell_column + 1;

    y_g0 = pointgridref.cell_row;
    y_g1 = pointgridref.cell_row + 1;

    //See if it works with logodd value instead of probability value
    P00 = (std::exp(logodd_matrix(y_g0, x_g0)) / (1 + std::exp(logodd_matrix(y_g0, x_g0))));
    P10 = (std::exp(logodd_matrix(y_g0, x_g1)) / (1 + std::exp(logodd_matrix(y_g0, x_g1))));
    P01 = (std::exp(logodd_matrix(y_g1, x_g0)) / (1 + std::exp(logodd_matrix(y_g1, x_g0))));
    P11 = (std::exp(logodd_matrix(y_g1, x_g1)) / (1 + std::exp(logodd_matrix(y_g1, x_g1))));
    //RCLCPP_INFO(this->get_logger(), "P00: %f, P01: %f, P10: %f, P11: %f, x_g0: %f, x_g1: %f, y_g0: %f, y_g1: %f", P00, P01, P10, P11, x_g0, x_g1, y_g0, y_g1); REMOVE
    return {P00, P10, P01, P11, x_g0, x_g1, y_g0, y_g1};
}

Eigen::Matrix<float, 1, 2> OccupancyGrid::map_pointgradient(coordinates point, gridedges &edges) {
    gridcell gridref = this->get_gridcell(point.x, point.y);
    float x_gridref = gridref.cell_column;
    float y_gridref = gridref.cell_row;
    
    float delta_x = edges.x_g1 - edges.x_g0;
    float delta_y = edges.y_g1 - edges.y_g0;

    float grad_x = (y_gridref-edges.y_g0)/delta_y * (edges.P11-edges.P01)
                + (edges.y_g1-y_gridref)/delta_y * (edges.P10 - edges.P00);

    float grad_y = (x_gridref - edges.x_g0)/delta_x * (edges.P11 - edges.P10)
                + (edges.x_g1 - x_gridref)/delta_x * (edges.P01 - edges.P00);
            
    return {grad_x, grad_y};
}

float OccupancyGrid::map_interpolatepointval(coordinates point, gridedges &edges) {
    static int int_cycle = 0;
    //float x_gridref = (point.x + this->width/2) * this->resolution;
    //float y_gridref = (point.y + this->height/2) * this->resolution;
    gridcell gridref = this->get_gridcell(point.x, point.y);
    float x_gridref = gridref.cell_column;
    float y_gridref = gridref.cell_row;

    float delta_y = edges.y_g1-edges.y_g0;
    float delta_x = edges.x_g1-edges.x_g0;

    //RCLCPP_INFO(this->get_logger(), "Delta_y: %f, Delta_x: %f", delta_y, delta_x); REMOVE
    //Can still be simplified
    if (delta_x == 0 || delta_y == 0) {
        RCLCPP_INFO(this->get_logger(), "WARNING: EITHER DELTAX OR DELTAY IS EQUAL TO 0");
    }
    float xx0_deltax = (x_gridref-edges.x_g0)/delta_x;
    float x1x_deltax = (edges.x_g1-x_gridref)/delta_x;
    float yy0_deltay = (y_gridref-edges.y_g0)/delta_y;
    float y1y_deltay = (edges.y_g1-y_gridref)/delta_y;

    float res = yy0_deltay * (xx0_deltax * edges.P11 + x1x_deltax * edges.P01)
        + y1y_deltay * (xx0_deltax * edges.P10 + x1x_deltax * edges.P00);
    if (int_cycle == 10) {
        int_cycle = 0;
        //RCLCPP_INFO(this->get_logger(), "Point is: x: (%f), y: (%f)", point.x, point.y);
        //RCLCPP_INFO(this->get_logger(), "X_gridref: (%f), y_gridref: (%f)", x_gridref, y_gridref);
        //RCLCPP_INFO(this->get_logger(), "x_g0: (%f), x_g1: (%f), y_g0: (%f), y_g1: (%f)", edges.x_g0, edges.x_g1, edges.y_g0, edges.y_g1);
        //RCLCPP_INFO(this->get_logger(), "P11: (%f), P10: (%f), P01: (%f), P11: (%f)", edges.P11, edges.P10, edges.P01, edges.P11);
        //RCLCPP_INFO(this->get_logger(), "xx0_deltax: (%f), x1x_deltax: (%f), yy0_deltay: (%f), y1y_deltay: (%f)", xx0_deltax, x1x_deltax, yy0_deltay, y1y_deltay);
        //RCLCPP_INFO(this->get_logger(), "Interpolate Val: %f", res); //REMOVE
    }
    int_cycle++;
    
    return res;
}
