// ROS libraries
#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/Pose.h>
#include <visualization_msgs/Marker.h>
#include <tf2/impl/utils.h>
#include <nav_msgs/Odometry.h>

// Standard libraries
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <cmath>

// osqp-eigen
#include "predictions/pure_pursuit.h"

class Predictor{
public:
    // Constructor
    Predictor() = default;
    Predictor(ros::NodeHandle &nh, PurePursuit &pp);

    // Outside Entry Point
    std::vector<Eigen::VectorXd> get_path(const nav_msgs::Odometry::ConstPtr &msg);
    std::vector<Eigen::VectorXd> get_path();
    //interfacing
    double get_dt();
    bool prediction_ready;

private:
    // ros pubs/subs
    ros::NodeHandle nh_;
    ros::Publisher vis_pub_;
    ros::Subscriber odom_sub_;   

    // nonlinear updater params
    PurePursuit &controller;

    // predictor params
    double dt;
    int steps;
    double linear_weight_decay;
    double wheelbase;
    double max_speed;
    double max_accel;
    double max_decel;
    bool do_visualization;
    //added for rrt
    nav_msgs::Odometry::ConstPtr current_pos;
    
    
    

    // callbacks
    void odom_callback(const nav_msgs::Odometry::ConstPtr &msg);


    // Prediction functions
    Eigen::VectorXd update(Eigen::VectorXd &opp_pos, int step);
    Eigen::VectorXd update_linear(Eigen::VectorXd &opp_pos);
    Eigen::VectorXd update_long(Eigen::VectorXd &opp_pos);
    double compute_accel(double v, double v_des);

    // Visualization
    void visualize_path(std::vector<Eigen::VectorXd> &points);
};

double get_yaw(const nav_msgs::Odometry::ConstPtr &msg);