// ESE 680
// RRT assignment
// Author: Hongrui Zheng

// This file contains the class definition of tree nodes and RRT
// Before you start, please read: https://arxiv.org/pdf/1105.1186.pdf

// ros
#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf2/impl/utils.h>

// standard
#include <math.h>
#include <vector>
#include <array>
#include <iostream>
#include <fstream>
#include <iterator>
#include <string>
#include <algorithm>
#include <boost/algorithm/string.hpp>
#include <random>
#include <Eigen/Geometry>
#include <unordered_set>





//custom
#include "occ_grid.h"


// Struct defining the Node object in the RRT tree.
// More fields could be added to thiis struct if more info needed.
// You can choose to use this or not
typedef struct Node {
    Eigen::Vector2d point;
    double cost; // only used for RRT*
    int parent; // index of parent node in the tree vector
    bool is_root = false;
    std::unordered_set<int> children;
    int ind;
    
} Node;


class RRT {
public:
    RRT(ros::NodeHandle &nh);
    virtual ~RRT();
private:

    ros::NodeHandle nh_;

    // ros pub/sub
    // TODO: add the publishers and subscribers you need
    ros::Subscriber test_sub;
    ros::Subscriber pf_sub_;
    ros::Subscriber scan_sub_;
    ros::Publisher occgrid_pub;
    ros::Publisher tree_pub;
    ros::Publisher line_pub;
    ros::Publisher nav_pub;
    ros::Publisher best_line_pub;
    

    // tf stuff
    tf::TransformListener listener;

    // TODO: create RRT params
    //occu vars
    OccGrid occ_grid_;
    geometry_msgs::PoseStamped most_recent_pose_;
    bool pose_init = false;
    double min_scan_dist;
    double lidar_offset;
    double x_sample;
    double y_sample;

    double RRTs_connection_dist;

    

    
    
    // random generator, use this
    std::mt19937 gen;
    std::uniform_real_distribution<> x_dist;
    std::uniform_real_distribution<> y_dist;

    double steer_dist;
    std::vector<std::vector<std::vector<int>>> adjacencies;

    std::vector<Eigen::Vector2d> waypoints;

    // drive parameters
    Eigen::Vector2d current_goal;
    
    double car_intersection_radius;
    double rrt_close_to_goal_dist;
    int tree_nodes;

    int max_iter;

    double speed;
    double slowdown;

    

    std::vector<Node> previous_path;
    bool previous_path_init = false;
    

    // callbacks
    // where rrt actually happens
    void pf_callback(const geometry_msgs::PoseStamped &pose_msg);
    void test_callback(const nav_msgs::Odometry &pose_msg);

    // updates occupancy grid
    void scan_callback(const sensor_msgs::LaserScan::ConstPtr& scan_msg);

    Eigen::Vector2d dist2car(double angle, double dist, double lidar_off){
        
        Eigen::Vector2d r;
        r[0] = std::cos(angle)*dist + lidar_off;
        r[1] = std::sin(angle)*dist;
        return r;
    };




    // RRT methods
    void run_RRT();
    Eigen::Vector2d sample();
    int nearest(std::vector<Node> &tree, Eigen::Vector2d &sampled_point);
    Node steer(Node &nearest_node,int nearest_index, Eigen::Vector2d &sampled_point);
    bool check_collision(Node &nearest_node, Node &new_node);
    bool is_goal(Node &latest_added_node, double goal_x, double goal_y);
    std::vector<Node> find_path(std::vector<Node> &tree, Eigen::Vector2d goal);
    std::vector<Node> smooth_path(const std::vector<Node> &path);
    // RRT* methods
    double cost(std::vector<Node> &tree, Node &node);
    double line_cost(Node &n1, Node &n2);
    std::vector<int> near(std::vector<Node> &tree, Node &node);
    void RRTstar(std::vector<Node> &tree, int i);
    void update_cost_rec(std::vector<Node> &tree, int ind);
    void visualize_best_path(std::vector<Node> nodes);
    void visualize_waypoints();
    void visualize_tree(std::vector<Node> tree);
    void update_adjacencies(int node, Eigen::Vector2d p);
    Eigen::Vector2i adj_ind(Eigen::Vector2d pos);
    std::vector<int> find_neighbors(std::vector<Node> &tree, int index, double dist);
    void add_node(std::vector<Node> &tree, Node node);

    std::vector<Node> recover_best_path();

    void save_best_path(std::vector<Node> best_path);
    double get_steering_angle(Eigen::Vector2d targ);

    bool is_on_segment(Eigen::Vector2d ep1, Eigen::Vector2d ep2, Eigen::Vector2d pt);

    std::vector<Eigen::Vector2d> find_intersections(geometry_msgs::PoseStamped pose, double L);

    Eigen::Vector2d construct_mb_line(Eigen::Vector2d p1, Eigen::Vector2d p2);

    Eigen::Vector2d find_best_valid_point(std::vector<Eigen::Vector2d> pt);
    Eigen::Vector2d world_to_car(geometry_msgs::PoseStamped pose, Eigen::Vector2d pt);

    Eigen::Vector2d get_current_goal(geometry_msgs::PoseStamped pose);

    void initialize_points();

    //utility
    Eigen::Vector2d transform_scan_to_map_frame(const geometry_msgs::PoseStamped &pose, const Eigen::Vector2d &wp);
    double get_yaw(const geometry_msgs::PoseStamped &pose);
    Eigen::Vector2d car_frame_to_map_frame(const Eigen::Vector2d &cp);
    Eigen::Vector2d map_frame_to_car_frame(const Eigen::Vector2d &cp);

    double angle_between_vectors(Eigen::Vector2d v1, Eigen::Vector2d v2);



};


