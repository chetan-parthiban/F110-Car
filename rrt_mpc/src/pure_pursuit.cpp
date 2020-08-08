#include "predictions/pure_pursuit.h"

// Constructor
PurePursuit::PurePursuit(ros::NodeHandle &nh): nh_(nh) {
    // get waypoint params
    nh_.getParam("pkg_name", PKG_NAME);
    nh_.getParam("wp_relative_path", WP_RELATIVE_PATH);
    nh_.getParam("NUM_COLS_IN_WP", NUM_COLS_IN_WP);

    // get pure pursuit params
    nh_.getParam("LOOKAHEAD_DISTANCE", LOOKAHEAD_DISTANCE);
    nh_.getParam("MAX_STEERING_ANGLE", MAX_STEERING_ANGLE);

    // initialize waypoints
    read_waypoints();
}

// Main Entry Point 
Eigen::VectorXd PurePursuit::get_control(Eigen::VectorXd &pos){
    // Get the current state of the car
    double car_x_in_map_frame = pos(0);
    double car_y_in_map_frame = pos(1);
    double map_yaw = pos(2);

    // Find the current waypoint to track
    std::vector<double> closest_wp = get_closest_wp(car_x_in_map_frame, 
                                                    car_y_in_map_frame,
                                                    map_yaw);
    
    // Transform goal point to vehicle frame of reference
    Eigen::Vector2f wp_car_frame = transform_wp_to_car_frame(car_x_in_map_frame,
                                                                car_y_in_map_frame,
                                                                map_yaw,
                                                                closest_wp);

    // Calculate curvature/steering angle
    double gamma = (2*wp_car_frame[1])/(wp_car_frame.squaredNorm());
    gamma = clip(gamma, -MAX_STEERING_ANGLE, MAX_STEERING_ANGLE);
    
    // return drive message
    Eigen::VectorXd control(2);
    control << gamma, closest_wp.back();
    return control;
}

// Waypoint Functions
void PurePursuit::set_wp_path(){
    PKG_PATH = ros::package::getPath(PKG_NAME);
    WP_FULL_PATH = PKG_PATH + WP_RELATIVE_PATH; // path to waypoint csv
}
unsigned int PurePursuit::get_num_waypoints(){
    std::ifstream file(WP_FULL_PATH);
    std::string line;
    unsigned int num_wps = 0;
    
    while (getline(file, line)) {
        
        num_wps++;
    }

    return num_wps;
}
void PurePursuit::read_waypoints(){
    set_wp_path();        
    unsigned int num_wps = get_num_waypoints(); // get number of waypoints to properly allocate the vector
    std::vector<double> row(NUM_COLS_IN_WP);
    waypoints.resize(num_wps, std::vector<double>(NUM_COLS_IN_WP)); // Pre-allocating vector will speed up run time instead of dynamically resizing over and over again
    ROS_INFO_STREAM("Pure pursuit read_waypoints full waypoint path: " + WP_FULL_PATH);
    std::ifstream data(WP_FULL_PATH);
    std::string line;
    int i = 0;
    while(std::getline(data, line)) {
        std::stringstream lineStream(line);
        std::string cell;
        int j = 0;
        while(std::getline(lineStream, cell, ',')) {
            row.at(j) = std::stof(cell); // Convert cell from string to double
            j++;
        }
        
        waypoints.at(i) = row;
        i++;
    }
    ROS_INFO_STREAM(waypoints.size());
}

// Utility functions
double PurePursuit::clip(const double &v, const double &min_v, const double &max_v) {return std::max(std::min(v, max_v), min_v);}
double PurePursuit::get_yaw(const nav_msgs::Odometry::ConstPtr &msg){
    tf2::Quaternion q(
                    msg->pose.pose.orientation.x,
                    msg->pose.pose.orientation.y,
                    msg->pose.pose.orientation.z,
                    msg->pose.pose.orientation.w);
    return tf2::impl::getYaw(q);
}
Eigen::Vector2f PurePursuit::transform_wp_to_car_frame(const double &car_x_in_map_frame, 
                                            const double &car_y_in_map_frame, 
                                            const double &map_yaw, 
                                            const std::vector<double> &closest_wp) {
    Eigen::Matrix2f R_map_in_car_frame = Eigen::Rotation2Df(map_yaw).toRotationMatrix().transpose();
    Eigen::Vector2f d_car_in_map_frame(car_x_in_map_frame, car_y_in_map_frame);
    
    Eigen::Matrix3f H_map_in_car_frame;
    H_map_in_car_frame.topLeftCorner<2, 2>() = R_map_in_car_frame;
    H_map_in_car_frame.topRightCorner<2, 1>() = -R_map_in_car_frame*d_car_in_map_frame;
    H_map_in_car_frame.bottomLeftCorner<1, 2>().setZero();
    H_map_in_car_frame.bottomRightCorner<1, 1>().setOnes();
    
    Eigen::Vector3f wp_map_frame;
    wp_map_frame << closest_wp[0], closest_wp[1], 1;
    Eigen::Vector2f wp_car_frame = (H_map_in_car_frame*wp_map_frame).head<2>();

    return wp_car_frame;
}
std::vector<double> PurePursuit::get_closest_wp(const double &x, const double &y, const double &phi) {
    double x_forward = LOOKAHEAD_DISTANCE * std::cos(phi) + x;
    double y_forward = LOOKAHEAD_DISTANCE * std::sin(phi) + y;

    double min_distance = HUGE_VAL;
    int closest_wp_idx = -1;

    for (int i = 0; i < waypoints.size(); i++) {
        double diff_x = waypoints[i][0] - x_forward;
        double diff_y = waypoints[i][1] - y_forward;

        double dist = std::sqrt(diff_x*diff_x + diff_y*diff_y);
        if (dist < min_distance) {
            min_distance = dist;
            closest_wp_idx = i;
        }
    }

    return waypoints[closest_wp_idx];
}