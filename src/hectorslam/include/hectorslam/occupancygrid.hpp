
#include "rclcpp/rclcpp.hpp"
#include <Eigen/Dense>
#include <nav_msgs/msg/occupancy_grid.hpp>

typedef struct {
    int cell_column;
    int cell_row;
} gridcell;

typedef struct {
    double x;
    double y;
} coordinates;

typedef struct {
    gridcell cell;
    bool occupied;
} occcell_update;

class OccupancyGrid : public virtual rclcpp::Node {
    public:
        OccupancyGrid(int resolution, int height, int width);

        void publish_occupancygrid();

        void update_occupancygridcell(occcell_update &cell_update);

        void update_occupancygridcell_frompoint(coordinates &point, bool occupied);

        bool occupancygrid_state(int &cell_row, int &cell_column);

        void update_occupancymap(const std::vector<coordinates> global_laserscan, const geometry_msgs::msg::Pose &pose);

    protected:
        Eigen::MatrixXd logodd_matrix;
        float init_val;

        gridcell get_gridcell(double x, double y);

        // Height, Width in m
        int height;
        int width;

        // Resolution in cell/m, height and width in meters
        int resolution;

        // Max row and column of the grid
        int max_row;
        int max_column;
    
    private:
        rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publisher_;
        rclcpp::TimerBase::SharedPtr timer_;

        void mapfill_laserpoint(coordinates laserpoint, const geometry_msgs::msg::Pose &pose);
};