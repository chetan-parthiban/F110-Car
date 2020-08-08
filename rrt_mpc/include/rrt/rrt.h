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
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Geometry"
#include <unordered_set>

// custom
#include "occ_grid.h"
#include "predictions/predictor.h"


// Struct defining the Node object in the RRT tree.
typedef struct Node {
    Eigen::Vector2d point;
    double cost; 
    int parent; 
    bool is_root = false;
    std::unordered_set<int> children;
    int ind;
} Node;


class RRT {
public:
    // Constructors and destructors
    RRT() = default;
    RRT(ros::NodeHandle &nh, Predictor &pred);
    virtual ~RRT();
    // Interface to get next waypoint
    void get_target(const nav_msgs::Odometry::ConstPtr &pose_msg); 
    Eigen::Vector2d target;

private:
    ros::NodeHandle nh_;

    // ros pubs and subs
    ros::Subscriber scan_sub_;
    ros::Publisher occgrid_pub;
    ros::Publisher tree_pub;
    ros::Publisher line_pub;
    ros::Publisher best_line_pub;  
    ros::Subscriber odom_sub_;

    // Occupancy grid
    OccGrid occ_grid_;
    geometry_msgs::PoseStamped most_recent_pose_;
    bool pose_init = false;
    double min_scan_dist;
    double lidar_offset;
    double x_sample;
    double y_sample;
    void scan_callback(const sensor_msgs::LaserScan::ConstPtr& scan_msg); // update the grid
    
    // Waypoints
    std::vector<Eigen::Vector2d> waypoints;
    void initialize_points(); // initialize the waypoints

    // random generator initialization
    std::mt19937 gen;
    std::uniform_real_distribution<> x_dist;
    std::uniform_real_distribution<> y_dist;

    // Select Goal Waypoint
    double car_intersection_radius;
    std::vector<Eigen::Vector2d> find_intersections(geometry_msgs::PoseStamped pose, double L);
    Eigen::Vector2d find_best_valid_point(std::vector<Eigen::Vector2d> pt);
    Eigen::Vector2d construct_mb_line(Eigen::Vector2d p1, Eigen::Vector2d p2);
    bool is_on_segment(Eigen::Vector2d ep1, Eigen::Vector2d ep2, Eigen::Vector2d pt);
    Eigen::Vector2d current_goal;

    // RRT variables
    double steer_dist;
    double RRTs_connection_dist;
    double rrt_close_to_goal_dist;
    int tree_nodes;
    int max_iter;
    
    // RRT Top Level Functions
    Eigen::Vector2d run_RRT();    
    void RRTstar(std::vector<Node> &tree, int i);
    
    // RRT Utilities
    Eigen::Vector2d sample();
    bool check_collision(Node &nearest_node, Node &new_node);
    int nearest(std::vector<Node> &tree, Eigen::Vector2d &sampled_point);
    Node steer(Node &nearest_node,int nearest_index, Eigen::Vector2d &sampled_point);
    void add_node(std::vector<Node> &tree, Node node);

    // RRT* Utilities
    double cost(std::vector<Node> &tree, Node &node);
    double line_cost(Node &n1, Node &n2);
    void update_cost_rec(std::vector<Node> &tree, int ind);
    std::vector<int> find_neighbors(std::vector<Node> &tree, int index, double dist);

    // Path Utilities
    std::vector<Node> find_path(std::vector<Node> &tree, Eigen::Vector2d goal);
    std::vector<Node> smooth_path(const std::vector<Node> &path);
    std::vector<Node> local_path;

    // Tracking the previously used path
    std::vector<Node> recover_best_path();
    void save_best_path(std::vector<Node> best_path);
    std::vector<Node> previous_path;
    bool previous_path_init = false;

    // Transformations
    Eigen::Vector2d world_to_car(geometry_msgs::PoseStamped pose, Eigen::Vector2d pt);
    Eigen::Vector2d transform_scan_to_map_frame(const geometry_msgs::PoseStamped &pose, const Eigen::Vector2d &wp);
    double get_yaw(const geometry_msgs::PoseStamped &pose);
    Eigen::Vector2d car_frame_to_map_frame(const Eigen::Vector2d &cp);
    Eigen::Vector2d map_frame_to_car_frame(const Eigen::Vector2d &cp);
    double angle_between_vectors(Eigen::Vector2d v1, Eigen::Vector2d v2);

    // Visualizations
    bool do_visualization;
    void visualize_best_path(std::vector<Node> nodes);
    void visualize_waypoints();
    void visualize_tree(std::vector<Node> tree);

    //PREDICTION ADDITION
    bool check_dynamic_intersection(Eigen::Vector2d start_pos, double start_time, Eigen::Vector2d end_pos, double end_time, std::vector<Eigen::Vector2d> oppo_pred, double oppo_dt);
    double closest_distance_on_segment(Eigen::Vector2d P0, Eigen::Vector2d Q0,Eigen::Vector2d u, Eigen::Vector2d v, double time_length);
    std::vector<Eigen::Vector2d> current_prediction;
    double current_prediction_dt;

    double dynamic_collision_distance;
    double nominal_speed;
    bool predictions_ready = false;
    Predictor &predictor;
    //END PREDICTION
};


