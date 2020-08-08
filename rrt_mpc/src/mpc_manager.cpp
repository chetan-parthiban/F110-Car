// Package headers
#include "mpc/mpc_manager.h"

// ROS libraries
#include <ros/package.h>
#include <geometry_msgs/Pose.h>
#include <visualization_msgs/Marker.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <tf2/impl/utils.h>

// Standard libraries
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <cmath>
#include <chrono>
#include <thread>

// osqp-eigen (QP solver with Eigen wrapper)
#include <OsqpEigen/OsqpEigen.h>

using namespace rrt_mpc;
double wrap_angle(double x) {
    x = fmod(x + M_PI, 2*M_PI);
    if (x < 0)
        x += 2*M_PI;
    return x - M_PI;
}
double rad2deg(const double &a) {
    return a*180/M_PI;
}
double clip(const double &v, const double &min_v, const double &max_v) {
    return std::max(std::min(v, max_v), min_v);
}
Eigen::Vector2d transform_wp_to_car_frame(const double &car_x_in_map_frame, 
                                            const double &car_y_in_map_frame, 
                                            const double &map_yaw, 
                                            const std::vector<double> &closest_wp) {
    Eigen::Matrix2d R_map_in_car_frame = Eigen::Rotation2Dd(map_yaw).toRotationMatrix().transpose();
    Eigen::Vector2d d_car_in_map_frame(car_x_in_map_frame, car_y_in_map_frame);
    
    Eigen::Matrix3d H_map_in_car_frame;
    H_map_in_car_frame.topLeftCorner<2, 2>() = R_map_in_car_frame;
    H_map_in_car_frame.topRightCorner<2, 1>() = -R_map_in_car_frame*d_car_in_map_frame;
    H_map_in_car_frame.bottomLeftCorner<1, 2>().setZero();
    H_map_in_car_frame.bottomRightCorner<1, 1>().setOnes();
    
    Eigen::Vector3d wp_map_frame;
    wp_map_frame << closest_wp[0], closest_wp[1], 1;
    Eigen::Vector2d wp_car_frame = (H_map_in_car_frame*wp_map_frame).head<2>();

    return wp_car_frame;
}
// Constructor
MPCManager::MPCManager(ros::NodeHandle &nh, RRT &rrt): nh_(nh), rrt_(rrt){
    // Get pub/sub params
    std::string drive_topic, vis_topic, odom_topic, opp_topic;
    nh_.getParam("drive_topic", drive_topic);
    nh_.getParam("vis_topic", vis_topic);
    nh_.getParam("odom_topic", odom_topic);
    nh_.getParam("opp_topic", opp_topic);

    // get parameters
    nh_.getParam("vel_min", vel_min_);
    nh_.getParam("vel_max", vel_max_);
    nh_.getParam("max_accel", max_accel_);
    nh_.getParam("max_speed", max_speed_);

    // pure pursuit params
    nh_.getParam("pkg_name", PKG_NAME_);
    nh_.getParam("wp_relative_path", WP_RELATIVE_PATH_);
    nh_.getParam("max_steering_angle", MAX_STEERING_ANGLE_);
    nh_.getParam("percent_waypoints_forward", percent_waypoints_forward_);
    read_waypoints();

    // Initialize MPC Module
    mpc_ = MPC(nh_);
    use_rrt = false;

    // Create ROS subscribers and publishers
    drive_pub_ = nh_.advertise<ackermann_msgs::AckermannDriveStamped>(drive_topic, 1);
    vis_pub_   = nh_.advertise<visualization_msgs::Marker>(vis_topic, 100);
    odom_sub_  = nh_.subscribe(odom_topic, 1, &MPCManager::odom_callback, this);
    opp_sub_  = nh_.subscribe(opp_topic, 1, &MPCManager::opp_callback, this);
    timer = nh_.createTimer(ros::Duration(0.0125), &MPCManager::timer_callback, this);
}

// Control Generation
void MPCManager::odom_callback(const nav_msgs::Odometry::ConstPtr &msg){
    // Setup the current position Vector
    x.setZero();
    x(3) = std::sqrt(std::pow(msg->twist.twist.linear.x, 2) + 
        std::pow(msg->twist.twist.linear.y, 2));
    x(4) = msg->twist.twist.angular.z;
    x(5) = std::atan2(msg->twist.twist.linear.y, msg->twist.twist.linear.x);

    geometry_msgs::PoseStamped geo_pose;
    geo_pose.pose = msg->pose.pose;
    geo_pose.pose.orientation = msg->pose.pose.orientation;
    most_recent_pose_ = geo_pose;
    double max_v;
    if (slow_down) {
        max_v = 4.0;
    }
    else {
        max_v = 4.5;
    }
    // Get the goal Vector
    Eigen::Vector2d wp_car_frame(2);
    if (use_rrt) {
        wp_car_frame = map_frame_to_car_frame(rrt_.target);
        double speed = 2.0 + vel_max_ * (1/(2.5*(std::abs(wp_car_frame.y()/(wp_car_frame.x())))));
        x_r << wp_car_frame.x(), wp_car_frame.y(), 0, clip(speed, 0, max_v), 0, 0;
    }
    else {
        double car_x_in_map_frame = msg->pose.pose.position.x;
        double car_y_in_map_frame = msg->pose.pose.position.y;
        double map_yaw = get_yaw(most_recent_pose_);
        std::vector<double> ref_wp = get_ref_wp(car_x_in_map_frame, car_y_in_map_frame, map_yaw);
        wp_car_frame = transform_wp_to_car_frame(car_x_in_map_frame, car_y_in_map_frame,
                                                                map_yaw,
                                                                ref_wp);
        x_r << wp_car_frame[0], wp_car_frame[1], wrap_angle(ref_wp[2]-map_yaw), clip(ref_wp[3],0,max_v), ref_wp[4], ref_wp[5];                                                  
    }
    // Update dt
    double dt = 0.0125;

    // Compute and publish the control 
    Eigen::Matrix<double, nu, 1>  u = mpc_.carMPC(x, x_r, dt);
    u_star = mpc_.getUStar();
    x_star = mpc_.getXStar();
    next_u = 0;
    ready = true;    
}

void MPCManager::timer_callback(const ros::TimerEvent& ev){
    if (!ready) {return;}

    Eigen::Matrix<double, nu, 1>  u = u_star.segment(next_u * nu, nu); // (next_u + 1) * nu);
    Eigen::Matrix<double, nx, 1>  xt = x_star.segment(next_u * nx, nx); // (next_u + 1) * nx);
    double alpha = 2*max_accel_/max_speed_;
    double v_des = u.x()/alpha + xt(3);
    v_des = std::min(std::max(v_des, vel_min_), vel_max_); // clip v_des
    u.x() = v_des;
    if (next_u < 10) { next_u += 1; }
    ros::Time curr_time = ros::Time::now();
    publish_drive_message(curr_time, u);
}

void MPCManager::opp_callback(const nav_msgs::Odometry::ConstPtr &msg){

    double opp_x = msg->pose.pose.position.x;
    double opp_y = msg->pose.pose.position.y;
    double opp_v = std::sqrt(std::pow(msg->twist.twist.linear.x, 2) + 
        std::pow(msg->twist.twist.linear.y, 2));

    double ego_yaw = get_yaw(most_recent_pose_); 
    double ego_x = most_recent_pose_.pose.position.x; // + std::cos(ego_yaw);
    double ego_y = most_recent_pose_.pose.position.y; // + std::sin(ego_yaw);

    double targ_x = rrt_.target.x();
    double targ_y = rrt_.target.y();
    // if ((std::abs(opp_x - ego_x) < 1.1 && std::abs(opp_y - ego_y) < 1) ){
    slow_down = false;
    if (std::pow(opp_x - ego_x, 2) + std::pow(opp_y - ego_y, 2) > 2.5 && 
        std::pow(opp_x - targ_x, 2) + std::pow(opp_y - targ_y, 2) > 3.0) {
        use_rrt = false;
    }
    if (std::pow(opp_x - ego_x, 2) + std::pow(opp_y - ego_y, 2) < 2.0 || 
        std::pow(opp_x - targ_x, 2) + std::pow(opp_y - targ_y, 2) < 1.0 ) {
        use_rrt = true;
    } 
    if (opp_v > 3.0 && std::pow(opp_x - ego_x, 2) + std::pow(opp_y - ego_y, 2) < 1.5)  {
        slow_down = true;
    }

}

// Utilities 
void MPCManager::publish_drive_message(const ros::Time &curr_time, const Eigen::Matrix<double, nu, 1>  &u) const{
    ackermann_msgs::AckermannDriveStamped drive_msg;
    drive_msg.header.stamp = curr_time;
    drive_msg.header.frame_id = "base_link";
    drive_msg.drive.steering_angle = u.y();
    drive_msg.drive.speed = u.x();
    drive_pub_.publish(drive_msg);
}
void MPCManager::publish_marker_msg(const ros::Time &curr_time, const Eigen::Vector2d &wp_car_frame){
    double marker_size = 0.375;
    static unsigned int marker_id = -1;
    marker_id++;
    marker_id = 0;

    visualization_msgs::Marker marker_msg;
    marker_msg.header.stamp = curr_time;
    marker_msg.ns = "mpc_waypoint";
    marker_msg.id = marker_id;
    marker_msg.type = visualization_msgs::Marker::SPHERE;
    marker_msg.action = visualization_msgs::Marker::ADD;
    
    marker_msg.header.frame_id = "base_link";
    marker_msg.pose.position.x = wp_car_frame[0];
    marker_msg.pose.position.y = wp_car_frame[1];
    marker_msg.pose.position.z = 0.0;

    marker_msg.pose.orientation.x = 0.0;
    marker_msg.pose.orientation.y = 0.0;
    marker_msg.pose.orientation.z = 0.0;
    marker_msg.pose.orientation.w = 1.0;
    
    marker_msg.scale.x = marker_size;
    marker_msg.scale.y = marker_size;
    marker_msg.scale.z = marker_size;

    marker_msg.color.r = 0.0;
    marker_msg.color.g = 0.0;
    marker_msg.color.b = 1.0;
    marker_msg.color.a = 1.0;

    vis_pub_.publish(marker_msg);
}

double MPCManager::get_yaw(const geometry_msgs::PoseStamped &pose){
    tf2::Quaternion q(
                    pose.pose.orientation.x,
                    pose.pose.orientation.y,
                    pose.pose.orientation.z,
                    pose.pose.orientation.w);
    return tf2::impl::getYaw(q);
}
Eigen::Vector2d MPCManager::map_frame_to_car_frame(const Eigen::Vector2d &cp){
    Eigen::Matrix2d R_car_in_map_frame = Eigen::Rotation2Dd(get_yaw(most_recent_pose_)).toRotationMatrix();
    Eigen::Vector2d d_car_in_map_frame(most_recent_pose_.pose.position.x, most_recent_pose_.pose.position.y);

    Eigen::Matrix3d H_car_in_map_frame;
    H_car_in_map_frame.topLeftCorner<2, 2>() = R_car_in_map_frame;
    H_car_in_map_frame.topRightCorner<2, 1>() = d_car_in_map_frame;
    H_car_in_map_frame.bottomLeftCorner<1, 2>().setZero();
    H_car_in_map_frame.bottomRightCorner<1, 1>().setOnes();

    Eigen::Vector3d wp_map_frame;
    wp_map_frame << cp.x(), cp.y(), 1;
    Eigen::Vector2d wp_car_frame = (H_car_in_map_frame.inverse()*wp_map_frame).head<2>();

    return wp_car_frame;
}

// For PurePursuit
void MPCManager::set_wp_path(){
    PKG_PATH_ = ros::package::getPath(PKG_NAME_);
    WP_FULL_PATH_ = PKG_PATH_ + WP_RELATIVE_PATH_; // path to waypoint csv
}
unsigned int MPCManager::get_num_waypoints(){
    std::ifstream file(WP_FULL_PATH_);
    std::string line;
    unsigned int num_wps = 0;
    
    while (getline(file, line))
    {
        num_wps++;
    }

    return num_wps;
}
void MPCManager::read_waypoints(){
    set_wp_path();        
    num_waypoints_ = get_num_waypoints(); // get number of waypoints to properly allocate the vector
    std::vector<double> row(NUM_COLS_IN_WP_);
    wps_.resize(num_waypoints_, std::vector<double>(NUM_COLS_IN_WP_)); // Pre-allocating vector will speed up run time instead of dynamically resizing over and over again
    
    std::ifstream data(WP_FULL_PATH_);
    std::string line;
    int i = 0;
    while(std::getline(data, line))
    {
        std::stringstream lineStream(line);
        std::string cell;
        int j = 0;
        while(std::getline(lineStream, cell, ','))
        {
            row.at(j) = std::stod(cell); // Convert cell from string to double
            j++;
        }
        wps_.at(i) = row;
        i++;
    }
}
std::vector<double> MPCManager::get_ref_wp(const double &x, const double &y, const double &phi){
    // double x_forward = LOOKAHEAD_DISTANCE_ * std::cos(phi) + x;
    // double y_forward = LOOKAHEAD_DISTANCE_ * std::sin(phi) + y;

    double min_distance = HUGE_VAL;
    int closest_wp_idx = -1;

    for (int i = 0; i < wps_.size(); i++)
    {
        double diff_x = wps_[i][0] - x;
        double diff_y = wps_[i][1] - y;

        double dist = std::sqrt(diff_x*diff_x + diff_y*diff_y);
        if (dist < min_distance)
        {
            min_distance = dist;
            closest_wp_idx = i;
        }
    }

    int forward_index = static_cast<int>(num_waypoints_*percent_waypoints_forward_) + closest_wp_idx;
    if (forward_index >= num_waypoints_)
    {
        forward_index -= num_waypoints_;
    }

    return wps_[forward_index];
}
