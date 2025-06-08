#include "hectorslam.hpp"

HectorSlamNode::HectorSlamNode(int resolution, int height, int width) : rclcpp::Node("hectorslamnode"), OccupancyGrid(resolution, height, width) {
    this->subscriberslamtrigger_ = this->create_subscription<std_msgs::msg::String>("/slammodepublish", 10, std::bind(&HectorSlamNode::slam_iteration, this, std::placeholders::_1));

    this->subscriberlaserscan_ = this->create_subscription<sensor_msgs::msg::LaserScan>("scan", 10, std::bind(&HectorSlamNode::get_newest_laserscan, this, std::placeholders::_1));

    this->publisherpose_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/pose_rc", 10);
    this->tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    

    this->set_posemsg(0, 0, 0, this->pose);
}

bool first_iteration = true;

void HectorSlamNode::slam_iteration(const std_msgs::msg::String::SharedPtr msg) {
    if (msg->data == "pos_f") {
        twodpose_t pose_temp = this->pose;
        pose_temp.x += 0.02;
        this->set_posemsg(pose_temp.x, pose_temp.y, pose_temp.yaw, this->pose);
        return;
    }
    if (msg->data == "pos_l") {
        twodpose_t pose_temp = this->pose;
        pose_temp.y -= 0.02;
        this->set_posemsg(pose_temp.x, pose_temp.y, pose_temp.yaw, this->pose);
        return;
    }
    if (msg->data == "pos_r") {
        twodpose_t pose_temp = this->pose;
        pose_temp.y += 0.02;
        this->set_posemsg(pose_temp.x, pose_temp.y, pose_temp.yaw, this->pose);
        return;
    }
    if (msg->data == "pos_b") {
        twodpose_t pose_temp = this->pose;
        pose_temp.x -= 0.02;
        this->set_posemsg(pose_temp.x, pose_temp.y, pose_temp.yaw, this->pose);
        return;
    }
    if (msg->data == "pos_cl") {
        twodpose_t pose_temp = this->pose;
        pose_temp.yaw += 0.035;
        this->set_posemsg(pose_temp.x, pose_temp.y, pose_temp.yaw, this->pose);
        return;
    }
    if (msg->data == "pos_ccl") {
        twodpose_t pose_temp = this->pose;
        pose_temp.yaw -= 0.035;
        this->set_posemsg(pose_temp.x, pose_temp.y, pose_temp.yaw, this->pose);
        return;
    }

    auto start = this->get_clock()->now();
    RCLCPP_INFO(this->get_logger(), "STARTING NEW SLAM ITERATION at: %ld (nanoseconds)", start.nanoseconds());
    std::vector<pose_laserpoint> laserscan = this->latest_laserscan;
    RCLCPP_INFO(this->get_logger(), "Test Laserscan is: %i", (int)laserscan.size());
    this->scan_matching(laserscan);
    if (first_iteration) {
        std::vector<coordinates> global_laserscan;
        for (int i = 0; i < (int)this->latest_laserscan.size(); i++) {
            global_laserscan.push_back(transform_toglobalcoord(this->pose, this->latest_laserscan[i]));
        }
        update_occupancymap(global_laserscan, this->pose);
        first_iteration = false;
    }
    auto end = this->get_clock()->now();
    RCLCPP_INFO(this->get_logger(), "ENDING SLAM ITERATION at: %ld (nanoseconds)", end.nanoseconds());
    int time_diff = (end.nanoseconds() - start.nanoseconds())/1000;
    RCLCPP_INFO(this->get_logger(), "SLAM Iteration took [%i] ms", time_diff);
}

void HectorSlamNode::get_newest_laserscan(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    this->latest_laserscan.clear();
    int laserscan_size = msg->ranges.size();
    //RCLCPP_INFO(this->get_logger(), "Laserscan size is: %i", laserscan_size);
    pose_laserpoint new_point;
    float angle_temp = msg->angle_max;
    for (int i = 0; i < laserscan_size; i++) {
        if (msg->ranges[i] != std::numeric_limits<float>::infinity()) {
            // If the value is not infinity then it gets added to the latest_laserscan points
            
            new_point.rel_coord.x = msg->ranges[i] * std::cos(angle_temp);
            new_point.rel_coord.y = -msg->ranges[i] * std::sin(angle_temp);
            //RCLCPP_INFO(this->get_logger(), "[%i]Ang tmp: %f, Range: %f, Rel x: %f, y: %f", i, angle_temp, msg->ranges[i], new_point.rel_coord.x, new_point.rel_coord.y);
            this->latest_laserscan.push_back(new_point);
        }
        angle_temp-= msg->angle_increment;
    }
}

Eigen::Matrix<float, 2, 3> HectorSlamNode::pointpose_gradient(const twodpose_t &pose, const coordinates &rel_point) {
    Eigen::Matrix<float, 2, 3> res;
    res(0,0) = 1;
    res(0,1) = 0;
    res(0,2) = -std::sin(pose.yaw)*rel_point.x - std::cos(pose.yaw)*rel_point.y;
    res(1,0) = 0;
    res(1,1) = 1;
    res(1,2) = std::cos(pose.yaw)*rel_point.x - std::sin(pose.yaw)*rel_point.y;
    return res;
}

void HectorSlamNode::scan_matching(const std::vector<pose_laserpoint> &laserscan) {
    //also uses the pose
    Eigen::Matrix<float, 3, 3> H;
    Eigen::Matrix<float, 3, 1> gradM_dSdeps_maperr;
    twodpose_t temp_pose = {this->pose.x, this->pose.y, this->pose.yaw};
    RCLCPP_INFO(this->get_logger(), "Temp_pose is: %f, %f, %f", temp_pose.x, temp_pose.y, temp_pose.yaw);
    bool error_correction_success = false;
    float old_error = this->scan_match_error(temp_pose, laserscan);
    float temp_error = old_error;
    for (int iter = 0; iter < 30; iter ++) {
        int num_laserpoints = laserscan.size();
        H.setZero();
        gradM_dSdeps_maperr.setZero();
        for (int j = 0; j < (int)laserscan.size(); j++) {
            coordinates laserpoint_relative = laserscan[j].rel_coord;
            coordinates laserpoint_global = this->transform_toglobalcoord(temp_pose, laserscan[j]);
            //RCLCPP_INFO(this->get_logger(), "Laserpoint %i/%i: x -> %f, y-> %f", j, num_laserpoints, laserpoint.x, laserpoint.y);
            gridedges laserpoint_gridedges = this->get_pointgrid_edges(laserpoint_global);

            Eigen::Matrix<float, 1, 3> gradM_dSdeps = this->map_pointgradient(laserpoint_global, laserpoint_gridedges) * this->pointpose_gradient(temp_pose, laserpoint_relative);
            //RCLCPP_INFO(this->get_logger(), "Point Gradient is: %f, %f, %f", gradM_dSdeps(0,0), gradM_dSdeps(0,1), gradM_dSdeps(0,2));
            H += gradM_dSdeps.transpose() * gradM_dSdeps; //+ //Eigen::MatrixXf::Identity(3,3);
            // RCLCPP_INFO(this->get_logger(), "blaH00: %f, H01: %f, H02: %f, H10: %f, H11: %f, H12: %f, H20: %f, H21: %f, H22: %f", H(0,0), H(0,1), H(0,2), H(1,0), H(1,1), H(1,2), H(2,0), H(2,1), H(2,2));
            float val = (1 - this->map_interpolatepointval(laserpoint_global, laserpoint_gridedges));
            gradM_dSdeps_maperr += gradM_dSdeps.transpose() * val;
            if (j % 50 == 1) {
                Eigen::Matrix<float, 1, 2> test_gradient = this->map_pointgradient(laserpoint_global, laserpoint_gridedges);
                RCLCPP_INFO(this->get_logger(), "Laserpoint glob(x,y)->(%f, %f), gradient: (%f, %f)", laserpoint_global.x, laserpoint_global.y, test_gradient(0,0), test_gradient(0,1));
                RCLCPP_INFO(this->get_logger(), "x_g0: (%f), x_g1: (%f), y_g0: (%f), y_g1: (%f)", laserpoint_gridedges.x_g0, laserpoint_gridedges.x_g1, laserpoint_gridedges.y_g0, laserpoint_gridedges.y_g1);
                RCLCPP_INFO(this->get_logger(), "P00: (%f), P10: (%f), P01: (%f), P11: (%f)", laserpoint_gridedges.P00, laserpoint_gridedges.P10, laserpoint_gridedges.P01, laserpoint_gridedges.P11);
            }
        }
        //RCLCPP_INFO(this->get_logger(), "H is H00: %f, H01: %f, H02: %f, H10: %f, H11: %f, H12: %f, H20: %f, H21: %f, H22: %f", H(0,0), H(0,1), H(0,2), H(1,0), H(1,1), H(1,2), H(2,0), H(2,1), H(2,2));
        //RCLCPP_INFO(this->get_logger(), "gradM_dSdeps_maperr: (%f, %f, %f)", gradM_dSdeps_maperr(0,0), gradM_dSdeps_maperr(1,0), gradM_dSdeps_maperr(2,0));
        Eigen::Matrix<float, 3, 1> delt_pose;
        float det = H.determinant();
        if (det != 0) {
            delt_pose = H.inverse() * gradM_dSdeps_maperr;

            if (delt_pose(2, 0) > 0.3) {
                delt_pose(2,0) = 0.3;
            } else if (delt_pose(2,0) < -0.3) {
                delt_pose(2,0) = -0.3;
            }
        } else {
            delt_pose = {{0}, {0}, {0}};
        }
        bool found_better_error = false;
        RCLCPP_INFO(this->get_logger(), "Initial Delta Pose is: (%f, %f, %f)", delt_pose(0, 0), delt_pose(1, 0), delt_pose(2, 0));
        for (int k = 0; k <= 30; k++) {
            twodpose_t new_pose = {temp_pose.x + delt_pose(0,0), temp_pose.y + delt_pose(1, 0), temp_pose.yaw + delt_pose(2, 0)};
            float new_err = this->scan_match_error(new_pose, laserscan);
            if (new_err < temp_error) {
                //RCLCPP_INFO(this->get_logger(), "Found a new best pose. Old error: (%f), new error: (%f)", temp_error, new_err);
                found_better_error = true;
                error_correction_success = true;
                temp_error = new_err;
                temp_pose = new_pose;
                break;
            } else {
                delt_pose = delt_pose / 2;
                RCLCPP_INFO(this->get_logger(), "No New Delta Pose iter(%i) is: (%f, %f, %f)", k, delt_pose(0, 0), delt_pose(1, 0), delt_pose(2, 0));
                RCLCPP_INFO(this->get_logger(), "Scan Match error is: %f", new_err);
                //RCLCPP_INFO(this->get_logger(), "New error (%f) > temp_error (%f). Halve delt_pose", new_err, temp_error);
                //RCLCPP_INFO(this->get_logger(), "New Delta Pose iter(%i) is: (%f, %f, %f)", k, delt_pose(0, 0), delt_pose(1, 0), delt_pose(2, 0));
            }
        }
        if (!found_better_error) {
            RCLCPP_INFO(this->get_logger(), "No better error found in iteration [%i]", iter);
            break;
        }
    }

    
    //Old Pose
    twodpose_t old_pose = this->pose;

    //Update Pose Value
    if (temp_error < old_error) {
        this->pose.x = temp_pose.x;
        this->pose.y = temp_pose.y;
        this->pose.yaw = temp_pose.yaw;
        RCLCPP_INFO(this->get_logger(), "Updating Pose: [Old Pose: (%f, %f, %f), New Pose: (%f, %f, %f)",
                old_pose.x, old_pose.y, old_pose.yaw, this->pose.x, this->pose.y, this->pose.yaw);
    }
    RCLCPP_INFO(this->get_logger(), "ERROR CORRECTION SUCCESS: %i", error_correction_success);
    RCLCPP_INFO(this->get_logger(), "Old Error: (%f), New Error: (%f)", old_error, this->scan_match_error(this->pose, laserscan));
    twodpose_t optimal_pose = {0, 0, 0};
    RCLCPP_INFO(this->get_logger(), "Optimal Error Value: %f", this->scan_match_error(optimal_pose, laserscan));

    //Update Pose in 
    this->set_posemsg(temp_pose.x, temp_pose.y, temp_pose.yaw, this->pose);
}

float HectorSlamNode::scan_match_error(const twodpose_t &pose, const std::vector<pose_laserpoint> &laserscan) {
    float err = 0;
    for (int i = 0; i < (int)laserscan.size(); i++) {
        coordinates point = this->transform_toglobalcoord(pose, laserscan[i]);
        gridedges edges = this->get_pointgrid_edges(point);
        float err_val = 1 - this->map_interpolatepointval(point, edges);
        //if (i % 20 == 1) {
            //RCLCPP_INFO(this->get_logger(), "Err val at (%f, %f): %f", point.x, point.y, err_val);
        //}
        err += std::pow(err_val, 2);
    }
    return err;
}


