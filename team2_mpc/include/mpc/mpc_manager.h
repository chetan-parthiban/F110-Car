// Package libraries
#include "mpc.h"

// Standard libraries
#include <vector>

// ROS libraries
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

// Eigen libraries
#include <Eigen/Geometry>

using namespace team2_mpc;

class MPCManager
{
private:
    ros::NodeHandle nh_;
    ros::Publisher vis_wp_pub_;
    ros::Publisher vis_soln_pub_;
    ros::Publisher drive_pub_;
    ros::Subscriber odom_sub_;

    // Package files params
    std::string PKG_NAME_;
    std::string WP_RELATIVE_PATH_;

    // Car params
    double MAX_STEERING_ANGLE_;

    // Reference waypoint params
    int NUM_COLS_IN_WP_ = 6;
    // int NUM_COLS_IN_WP_ = 4;

    std::string PKG_PATH_;
    std::string WP_FULL_PATH_;
    std::vector<std::vector<double>> wps_;

    unsigned int num_waypoints_;
    double percent_waypoints_forward_;

    MPC mpc_;
    ros::Time prev_time_ = ros::Time();

    void set_wp_path();
    
    unsigned int get_num_waypoints();
    
    void read_waypoints();

    std::vector<double> get_ref_wp(const double &x, const double &y, const double &phi);

    void visualize_centerline();
    
    void visualize_wp(const ros::Time &curr_time, const Eigen::Vector2d &wp_car_frame);

    void visualize_xstar(const ros::Time &curr_time, const Eigen::VectorXd &xstar);

    void publish_drive_message(const ros::Time &curr_time, const Eigen::Matrix<double, nu, 1> &u) const;
    
    void odom_callback(const nav_msgs::Odometry::ConstPtr &msg);

public:
    MPCManager(ros::NodeHandle &nh);
};