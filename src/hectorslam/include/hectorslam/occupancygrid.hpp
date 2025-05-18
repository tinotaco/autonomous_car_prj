
#include "rclcpp/rclcpp.hpp"
#include "coordinate_types.hpp"
#include <Eigen/Dense>
#include <nav_msgs/msg/occupancy_grid.hpp>

class OccupancyGrid : public virtual rclcpp::Node {
    public:
        OccupancyGrid(int resolution, int height, int width);

        void publish_occupancygrid();

        void update_occupancygridcell(occcell_update &cell_update);

        void update_occupancygridcell_frompoint(coordinates &point, bool occupied);

        bool occupancygrid_state(int &cell_row, int &cell_column);

        void update_occupancymap(const std::vector<coordinates> global_laserscan, const twodpose_t &pose);

    protected:
        Eigen::MatrixXd logodd_matrix;
        float init_val;

        gridcell get_gridcell(float x, float y);

        Eigen::Matrix<float, 1, 2> map_pointgradient(coordinates point, gridedges &edges);

        float map_interpolatepointval(coordinates point, gridedges &edges);

        gridedges get_pointgrid_edges(coordinates pt);

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

        void mapfill_laserpoint(coordinates laserpoint, const twodpose_t &pose);
};