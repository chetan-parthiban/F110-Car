// ROS libraries
#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <tf2/impl/utils.h>

// Standard libraries
#include <vector>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <cmath>

// Eigen libraries
#include <Eigen/Geometry>
#include <Eigen/Dense>

class PurePursuit
{
public:
    // Constructor
    PurePursuit() = default;
    PurePursuit(ros::NodeHandle &nh);  

    // Main Entry Point  
    Eigen::VectorXd get_control(Eigen::VectorXd &pos);

    std::vector<std::vector<double>> waypoints;
    

private:
    // ros pub/subs
    ros::NodeHandle nh_;

    // waypoint params
    std::string PKG_NAME;
    std::string WP_RELATIVE_PATH;
    int NUM_COLS_IN_WP;
    std::string PKG_PATH;
    std::string WP_FULL_PATH;

    // waypoint functions
    void set_wp_path();
    unsigned int get_num_waypoints();
    void read_waypoints();

    // pure pursuit params
    double LOOKAHEAD_DISTANCE;
    double MAX_STEERING_ANGLE;

    // utility functions
    double clip(const double &v, const double &min_v, const double &max_v);
    double get_yaw(const nav_msgs::Odometry::ConstPtr &msg);
    std::vector<double> get_closest_wp(const double &x, const double &y, const double &phi);
    Eigen::Vector2f transform_wp_to_car_frame(const double &car_x_in_map_frame, 
                                              const double &car_y_in_map_frame, 
                                              const double &map_yaw, 
                                              const std::vector<double> &closest_wp);
};