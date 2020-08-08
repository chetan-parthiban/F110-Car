// Package headers
#include "../include/mpc/mpc_manager.h"

// ROS libraries
#include <ros/package.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <tf2/impl/utils.h>

// Standard libraries
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <cmath>

// osqp-eigen
#include <OsqpEigen/OsqpEigen.h>

using namespace team2_mpc;

void MPCManager::set_wp_path()
{
    PKG_PATH_ = ros::package::getPath(PKG_NAME_);
    WP_FULL_PATH_ = PKG_PATH_ + WP_RELATIVE_PATH_; // path to waypoint csv
}

unsigned int MPCManager::get_num_waypoints()
{
    std::ifstream file(WP_FULL_PATH_);
    std::string line;
    unsigned int num_wps = 0;
    
    while (getline(file, line))
    {
        num_wps++;
    }

    return num_wps;
}

void MPCManager::read_waypoints()
{
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

double get_yaw(const nav_msgs::Odometry::ConstPtr &msg)
{
    tf2::Quaternion q(
                    msg->pose.pose.orientation.x,
                    msg->pose.pose.orientation.y,
                    msg->pose.pose.orientation.z,
                    msg->pose.pose.orientation.w);
    return tf2::impl::getYaw(q);
}

std::vector<double> MPCManager::get_ref_wp(const double &x, const double &y, const double &phi)
{
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

Eigen::MatrixXd transform_from_car_to_map_frame(const double &car_x_in_map_frame, 
                                                const double &car_y_in_map_frame, 
                                                const double &map_yaw, 
                                                const Eigen::MatrixXd &points_in_car_frame)
{
    Eigen::Matrix2d R_car_in_map_frame = Eigen::Rotation2Dd(map_yaw).toRotationMatrix();
    Eigen::Vector2d d_car_in_map_frame(car_x_in_map_frame, car_y_in_map_frame);
    
    Eigen::Matrix3d H_map_in_car_frame;
    H_map_in_car_frame.topLeftCorner<2, 2>() = R_car_in_map_frame;
    H_map_in_car_frame.topRightCorner<2, 1>() = d_car_in_map_frame;
    H_map_in_car_frame.bottomLeftCorner<1, 2>().setZero();
    H_map_in_car_frame.bottomRightCorner<1, 1>().setOnes();

    Eigen::MatrixXd points_in_car_frame3(3, points_in_car_frame.cols());
    points_in_car_frame3.bottomRows<1>().setOnes();

    Eigen::MatrixXd points_in_map_frame = (H_map_in_car_frame*points_in_car_frame3).topRows<2>();

    return points_in_map_frame;
}

Eigen::MatrixXd transform_from_map_to_car_frame(const double &car_x_in_map_frame, 
                                            const double &car_y_in_map_frame, 
                                            const double &map_yaw, 
                                            const Eigen::MatrixXd &points_in_map_frame)
{
    Eigen::Matrix2d R_map_in_car_frame = Eigen::Rotation2Dd(map_yaw).toRotationMatrix().transpose();
    Eigen::Vector2d d_car_in_map_frame(car_x_in_map_frame, car_y_in_map_frame);
    
    Eigen::Matrix3d H_map_in_car_frame;
    H_map_in_car_frame.topLeftCorner<2, 2>() = R_map_in_car_frame;
    H_map_in_car_frame.topRightCorner<2, 1>() = -R_map_in_car_frame*d_car_in_map_frame;
    H_map_in_car_frame.bottomLeftCorner<1, 2>().setZero();
    H_map_in_car_frame.bottomRightCorner<1, 1>().setOnes();

    Eigen::MatrixXd points_in_map_frame3(3, points_in_map_frame.cols());
    points_in_map_frame3.bottomRows<1>().setOnes();

    Eigen::Vector2d wp_car_frame = (H_map_in_car_frame*points_in_map_frame3).topRows<2>();

    return wp_car_frame;
}

Eigen::Vector2d transform_wp_to_car_frame(const double &car_x_in_map_frame, 
                                            const double &car_y_in_map_frame, 
                                            const double &map_yaw, 
                                            const std::vector<double> &closest_wp)
{
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

double clip(const double &v, const double &min_v, const double &max_v)
{
    return std::max(std::min(v, max_v), min_v);
}

void MPCManager::visualize_centerline()
{
    double marker_size = 0.1;
    // static unsigned int marker_id = -1;
    // marker_id++;
    int marker_id = 0;

    std::vector<geometry_msgs::Point> pts(num_waypoints_);
    for (size_t i = 0; i < num_waypoints_; i++)
    {
        geometry_msgs::Point p;
        p.x = wps_[i][0];
        p.y = wps_[i][1];
        p.z = 0;
        pts.at(i) = p;
    }

    visualization_msgs::Marker marker_msg;
    marker_msg.header.stamp = ros::Time::now();
    marker_msg.ns = "mpc_centerline";
    marker_msg.id = marker_id;
    marker_msg.type = visualization_msgs::Marker::LINE_STRIP;
    marker_msg.action = visualization_msgs::Marker::ADD;
    
    marker_msg.header.frame_id = "map";
    marker_msg.pose.position.x = 0.0;
    marker_msg.pose.position.y = 0.0;
    marker_msg.pose.position.z = 0.0;

    marker_msg.pose.orientation.x = 0.0;
    marker_msg.pose.orientation.y = 0.0;
    marker_msg.pose.orientation.z = 0.0;
    marker_msg.pose.orientation.w = 1.0;
    
    marker_msg.points = pts;

    marker_msg.scale.x = marker_size;

    marker_msg.color.r = 0.0;
    marker_msg.color.g = 0.0;
    marker_msg.color.b = 1.0;
    marker_msg.color.a = 1.0;

    vis_wp_pub_.publish(marker_msg);
}

void MPCManager::visualize_wp(const ros::Time &curr_time, const Eigen::Vector2d &wp_car_frame)
{
    double marker_size = 0.25;
    // static unsigned int marker_id = -1;
    // marker_id++;
    int marker_id = 0;

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
    marker_msg.color.g = 1.0;
    marker_msg.color.b = 0.0;
    marker_msg.color.a = 1.0;

    vis_wp_pub_.publish(marker_msg);
}

void MPCManager::visualize_xstar(const ros::Time &curr_time, const Eigen::VectorXd &xstar)
{
    double marker_size = 0.075;
    // static unsigned int marker_id = -1;
    // marker_id++;
    int marker_id = 0;

    std::vector<geometry_msgs::Point> pts;
    for (size_t i = 0; i < xstar.size(); i += nx)
    {
        if (i > 0)
        {
            if (xstar.segment<2>(i).norm() < xstar.segment<2>(i-nx).norm())
            {
                break;
            }
        }
        geometry_msgs::Point p;
        p.x = xstar(i);
        p.y = xstar(i+1);
        p.z = 0;
        pts.push_back(p);
    }
    

    visualization_msgs::Marker marker_msg;
    marker_msg.header.stamp = curr_time;
    marker_msg.ns = "mpc_xstar";
    marker_msg.id = marker_id;
    marker_msg.type = visualization_msgs::Marker::LINE_STRIP;
    marker_msg.action = visualization_msgs::Marker::ADD;
    
    marker_msg.header.frame_id = "base_link";
    marker_msg.pose.position.x = xstar.x();
    marker_msg.pose.position.y = xstar.y();
    marker_msg.pose.position.z = 0.0;

    marker_msg.pose.orientation.x = 0.0;
    marker_msg.pose.orientation.y = 0.0;
    marker_msg.pose.orientation.z = 0.0;
    marker_msg.pose.orientation.w = 1.0;
    
    marker_msg.points = pts;

    marker_msg.scale.x = marker_size;

    marker_msg.color.r = 1.0;
    marker_msg.color.g = 0.0;
    marker_msg.color.b = 0.0;
    marker_msg.color.a = 1.0;

    vis_soln_pub_.publish(marker_msg);
}

void MPCManager::publish_drive_message(const ros::Time &curr_time, const Eigen::Matrix<double, nu, 1> &u) const
{
    ackermann_msgs::AckermannDriveStamped drive_msg;
    drive_msg.header.stamp = curr_time;
    drive_msg.header.frame_id = "base_link";
    drive_msg.drive.steering_angle = u.y();
    drive_msg.drive.speed = u.x();
    drive_pub_.publish(drive_msg);
}

double wrap_angle(double x)
{
    x = fmod(x + M_PI, 2*M_PI);
    if (x < 0)
        x += 2*M_PI;
    return x - M_PI;
}

double rad2deg(const double &a)
{
    return a*180/M_PI;
}

void MPCManager::odom_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
    // Get the current state of the car
    double car_x_in_map_frame = msg->pose.pose.position.x;
    double car_y_in_map_frame = msg->pose.pose.position.y;
    double map_yaw = get_yaw(msg);
    double car_v = std::sqrt(
        std::pow(msg->twist.twist.linear.x, 2) + 
        std::pow(msg->twist.twist.linear.y, 2));
    double yaw_dot = msg->twist.twist.angular.z;
    double slip_angle = std::atan2(msg->twist.twist.linear.y, msg->twist.twist.linear.x);
    
    // Find the current waypoint to track
    std::vector<double> ref_wp = get_ref_wp(car_x_in_map_frame, car_y_in_map_frame, map_yaw);
    // Transform goal point to vehicle frame of reference
    Eigen::Vector2d wp_car_frame = transform_wp_to_car_frame(car_x_in_map_frame,
                                                                car_y_in_map_frame,
                                                                map_yaw,
                                                                ref_wp);

    // Calculate curvature/steering angle
    // double gamma = (2*wp_car_frame[1])/(wp_car_frame.squaredNorm());
    // gamma = clip(gamma, -MAX_STEERING_ANGLE_, MAX_STEERING_ANGLE_);
    
    ros::Time curr_time = ros::Time::now();
    double dt = (curr_time - prev_time_).toSec();
    prev_time_ = curr_time;

    Eigen::Matrix<double, nx, 1>  x;
    // x << car_x_in_map_frame, car_y_in_map_frame, map_yaw, car_v, yaw_dot, slip_angle;
    x << 0, 0, 0, car_v, yaw_dot, slip_angle;
    Eigen::Matrix<double, nx, 1> x_r;
    x_r << wp_car_frame[0], wp_car_frame[1], wrap_angle(ref_wp[2]-map_yaw), ref_wp[3], ref_wp[4], ref_wp[5];
    // x_r << wp_car_frame[0], wp_car_frame[1], wrap_angle(ref_wp[2]-map_yaw), ref_wp[3], 0, 0;

    Eigen::Matrix<double, nu, 1> u = mpc_.carMPC(x, x_r, dt);
    
    // Publish messages
    visualize_wp(curr_time, wp_car_frame);
    visualize_xstar(curr_time, mpc_.getXStar());
    publish_drive_message(curr_time, u);
}

MPCManager::MPCManager(ros::NodeHandle &nh): nh_(nh)
{
    std::string drive_topic, vis_wp_topic, vis_soln_topic, odom_topic;
    nh_.getParam("drive_topic", drive_topic);
    nh_.getParam("vis_wp_topic", vis_wp_topic);
    nh_.getParam("vis_soln_topic", vis_soln_topic);
    nh_.getParam("odom_topic", odom_topic);

    nh_.getParam("pkg_name", PKG_NAME_);
    nh_.getParam("wp_relative_path", WP_RELATIVE_PATH_);

    nh_.getParam("max_steering_angle", MAX_STEERING_ANGLE_);

    nh_.getParam("percent_waypoints_forward", percent_waypoints_forward_);

    read_waypoints();

    mpc_ = MPC(nh_);

    // Create ROS subscribers and publishers
    drive_pub_    = nh_.advertise<ackermann_msgs::AckermannDriveStamped>(drive_topic, 1);
    vis_wp_pub_   = nh_.advertise<visualization_msgs::Marker>(vis_wp_topic, 1);
    vis_soln_pub_ = nh_.advertise<visualization_msgs::Marker>(vis_soln_topic, 1);
    odom_sub_     = nh_.subscribe(odom_topic, 1, &MPCManager::odom_callback, this);
    
    ros::Duration(0.75).sleep();
    visualize_centerline();
}