#include "rclcpp/rclcpp.hpp"
#include "occupancygrid.hpp"
#include "std_msgs/msg/string.hpp"
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <math.h>

typedef struct {
    double x;
    double y;
} coordinates;

typedef struct {
    coordinates rel_coord;
} pose_laserpoint;

class HectorSlamNode : public OccupancyGrid, public virtual rclcpp::Node {
    public:
        HectorSlamNode(int resolution, int height, int width);

        void slam_iteration(const std_msgs::msg::String::SharedPtr msg);

        void get_newest_laserscan(const sensor_msgs::msg::LaserScan::SharedPtr msg);

        void update_map(const std::vector<pose_laserpoint> &scan, const geometry_msgs::msg::Pose &pose);

    private:
        // Subscription for another Slam Iteration
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriberslamtrigger_;
        
        // Laser Scan Subscription
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscriberlaserscan_;

        // Publisher to publish Pose in Rviz2
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisherpose_;
        
        // TransformBroadcaster to broadcast transforms
        std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;

        std::vector<pose_laserpoint> latest_laserscan;

        geometry_msgs::msg::Pose pose;

        std::string pose_frame_id_ = "laser_frame"; //Change to rc_frame at some point
        std::string parent_frame_id_ = "base_link";

        void set_posemsg(double x, double y, double angle, geometry_msgs::msg::Pose &pose) {
            //Use Quaternion to set proper Quaternion values
            tf2::Quaternion quat;

            //angle should be in radians, and front should be positive x-axis (or that should be angle 0)
            //angle+= M_PI/2;
            quat.setRPY(0, 0, angle);

            // Message for the Pose marker
            pose.position.x = x;
            pose.position.y = y;
            pose.position.z = 0;

            pose.orientation = tf2::toMsg(quat);


            // Message for the Pose Marker
            geometry_msgs::msg::PoseStamped pose_msg;
            pose_msg.header.stamp = this->get_clock()->now();
            pose_msg.header.frame_id = this->pose_frame_id_;
            pose_msg.pose = this->pose;

            geometry_msgs::msg::TransformStamped tf_msg;

            // Message for the Transform
            tf_msg.header.stamp = this->get_clock()->now();
            tf_msg.header.frame_id = this->parent_frame_id_;
            tf_msg.child_frame_id = this->pose_frame_id_;

            tf_msg.transform.translation.x = x;
            tf_msg.transform.translation.y = y;
            tf_msg.transform.translation.z = 0;

            tf_msg.transform.rotation = pose.orientation;
            
            //Publish/Broadcast Messages
            tf_broadcaster_->sendTransform(tf_msg);
            publisherpose_->publish(pose_msg);
            RCLCPP_INFO(this->get_logger(), "Set Pose Message");
        };

        coordinates transform_toglobalcoord(const geometry_msgs::msg::Pose &pose, const pose_laserpoint &rel_point) {
            coordinates res;
            //TODO: Change to proper rotation
            //float rotation = 2 * std::acos(pose.orientation.z);
            float rotation = 0;
            res.x = std::cos(rotation) * rel_point.rel_coord.x - std::sin(rotation) * rel_point.rel_coord.y + pose.position.x;
            res.y = std::sin(rotation) * rel_point.rel_coord.x + std::cos(rotation) * rel_point.rel_coord.y + pose.position.y;
            // RCLCPP_INFO(this->get_logger(), "Rotation is: %f, Rel point x is: %f, y is: %f, Res x is: %f, y is: %f", rotation, rel_point.rel_coord.x, rel_point.rel_coord.y, res.x, res.y);
            return res;
        };

        bool getoccupancygrid_update(coordinates &point, bool occupied, occcell_update &map_update);
};