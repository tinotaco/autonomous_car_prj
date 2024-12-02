
#include "rclcpp/rclcpp.hpp"
#include <Eigen/Dense>
#include <nav_msgs/msg/occupancy_grid.hpp>

typedef struct {
    int cell_row;
    int cell_column;
    bool occupied;
} occcell_update;


class OccupancyGrid : public virtual rclcpp::Node {
    public:
        OccupancyGrid(int resolution, int height, int width);

        void publish_occupancygrid();

        void update_occupancygridcell(occcell_update &cell_update);


    protected:
        Eigen::MatrixXd logodd_matrix;
        float init_val;

        // Height, Width in m
        int height;
        int width;

        // Resolution in cell/m, height and width in meters
        int resolution;
    
    private:
        rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publisher_;
        rclcpp::TimerBase::SharedPtr timer_;
};