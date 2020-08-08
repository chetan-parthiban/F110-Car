// Package libraries
#include "mpc.h"
#include "../rrt/rrt.h"

// Standard libraries
#include <vector>

// ROS libraries
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

// Eigen libraries
#include <Eigen/Geometry>

using namespace rrt_mpc;

class MPCManager
{
public:
    // Constructor
    MPCManager(ros::NodeHandle &nh, RRT &rrt);
private:
    // ros pubs and subs
    ros::NodeHandle nh_;
    ros::Publisher vis_pub_;
    ros::Publisher drive_pub_;
    ros::Subscriber odom_sub_;
    ros::Subscriber opp_sub_;
    ros::Timer timer;
    geometry_msgs::PoseStamped most_recent_pose_;

    // States for mpc
    Eigen::Matrix<double, nx, 1> x = Eigen::Matrix<double, nx, 1>::Zero();
    Eigen::Matrix<double, nx, 1> x_r = Eigen::Matrix<double, nx, 1>::Zero();
    ros::Time prev_time_ = ros::Time();

    // States for publishing
    int next_u = 0;
    Eigen::VectorXd u_star;
    Eigen::VectorXd x_star;
    bool ready = false;
    double vel_min_;
    double vel_max_;
    double max_accel_;
    double max_speed_;

    // mpc and RRT modules
    MPC mpc_;
    RRT& rrt_;
    bool use_rrt = false;
    bool slow_down = false;
    
    // main callback
    void odom_callback(const nav_msgs::Odometry::ConstPtr &msg);
    void timer_callback(const ros::TimerEvent& ev);
    void opp_callback(const nav_msgs::Odometry::ConstPtr &msg);

    // utilities
    void publish_marker_msg(const ros::Time &curr_time, const Eigen::Vector2d &wp_car_frame);
    void publish_drive_message(const ros::Time &curr_time, const Eigen::Matrix<double, nu, 1>  &u) const;
    double get_yaw(const geometry_msgs::PoseStamped &pose);
    Eigen::Vector2d map_frame_to_car_frame(const Eigen::Vector2d &cp);

    // for pure_pursuit
    std::string PKG_NAME_;
    std::string WP_RELATIVE_PATH_;
    std::string PKG_PATH_;
    std::string WP_FULL_PATH_;
    int NUM_COLS_IN_WP_ = 6;
    double MAX_STEERING_ANGLE_;
    
    std::vector<std::vector<double>> wps_;
    unsigned int num_waypoints_;
    double percent_waypoints_forward_;

    void set_wp_path();
    unsigned int get_num_waypoints();
    void read_waypoints();
    std::vector<double> get_ref_wp(const double &x, const double &y, const double &phi);
};



