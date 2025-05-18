
#include "rclcpp/rclcpp.hpp"
#include "occupancygrid.hpp"
#include "std_msgs/msg/string.hpp"
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <math.h>
#include <Eigen/Dense>


class HectorSlamNode : public OccupancyGrid, public virtual rclcpp::Node {
    public:

        HectorSlamNode(int resolution, int height, int width);

        void slam_iteration(const std_msgs::msg::String::SharedPtr msg);

        void get_newest_laserscan(const sensor_msgs::msg::LaserScan::SharedPtr msg);

        //void update_map(const std::vector<pose_laserpoint> &scan, const geometry_msgs::msg::Pose &pose);

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

        //geometry_msgs::msg::Pose pose;
        twodpose_t pose;

        std::string pose_frame_id_ = "rc_frame"; //Change to rc_frame at some point
        std::string parent_frame_id_ = "base_link";

        void set_posemsg(float x, float y, float angle, twodpose_t &pose) {
            //Use Quaternion to set proper Quaternion values
            tf2::Quaternion quat;

            //angle should be in radians, and front should be positive x-axis (or that should be angle 0)
            //angle+= M_PI/2;
            quat.setRPY(0, 0, angle);

            // Private Variable
            pose.x = x;
            pose.y = y;
            pose.yaw = angle;

            // Message for the Pose Marker
            geometry_msgs::msg::PoseStamped pose_msg;
            pose_msg.header.stamp = this->get_clock()->now();
            pose_msg.header.frame_id = this->pose_frame_id_;
            pose_msg.pose.position.x = x;
            pose_msg.pose.position.y = y;
            pose_msg.pose.position.z = 0;
            pose_msg.pose.orientation = tf2::toMsg(quat);

            geometry_msgs::msg::TransformStamped tf_msg;

            // Message for the Transform
            tf_msg.header.stamp = this->get_clock()->now();
            tf_msg.header.frame_id = this->parent_frame_id_;
            tf_msg.child_frame_id = this->pose_frame_id_;

            tf_msg.transform.translation.x = x;
            tf_msg.transform.translation.y = y;
            tf_msg.transform.translation.z = 0;

            tf_msg.transform.rotation = tf2::toMsg(quat);
            
            //Publish/Broadcast Messages
            tf_broadcaster_->sendTransform(tf_msg);
            publisherpose_->publish(pose_msg);
            RCLCPP_INFO(this->get_logger(), "Set Pose Message");
        };

        coordinates transform_toglobalcoord(const twodpose_t &pose, const pose_laserpoint &rel_point) {
            coordinates res;
            // Convert coordinates from rc_frame to global (base) coordinate frame
            res.x = pose.x + rel_point.rel_coord.x * std::cos(pose.yaw) - rel_point.rel_coord.y * std::sin(pose.yaw);
            res.y = pose.y + rel_point.rel_coord.x * std::sin(pose.yaw) + rel_point.rel_coord.y * std::cos(pose.yaw);
            //RCLCPP_INFO(this->get_logger(), "Rotation is: %f, Rel point x is: %f, y is: %f, Res x is: %f, y is: %f", pose.yaw, rel_point.rel_coord.x, rel_point.rel_coord.y, res.x, res.y);
            return res;
        };

        Eigen::Matrix<float, 2, 3> pointpose_gradient(const twodpose_t &pose, const coordinates &rel_point);

        bool getoccupancygrid_update(coordinates &point, bool occupied, occcell_update &map_update);

        void scan_matching(const std::vector<pose_laserpoint> &laserscan);

        float scan_match_error(const twodpose_t &pose, const std::vector<pose_laserpoint> &laserscan);
};