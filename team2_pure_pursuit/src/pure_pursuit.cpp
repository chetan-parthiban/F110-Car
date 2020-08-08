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

class PurePursuit
{
private:
    ros::NodeHandle n;
    ros::Publisher vis_pub;
    ros::Publisher drive_pub;
    ros::Subscriber odom_sub;

    const float LOOKAHEAD_DISTANCE = 1.25;
    const float MAX_STEERING_ANGLE = 0.4189;
    const std::string PKG_NAME = "team2_pure_pursuit";
    const std::string WP_RELATIVE_PATH = "/data/skirk.csv";
    const int NUM_COLS_IN_WP = 4;

    std::string PKG_PATH;
    std::string WP_FULL_PATH;
    std::vector<std::vector<float>> wps;

    void set_wp_path()
    {
        PKG_PATH = ros::package::getPath(PKG_NAME);
        WP_FULL_PATH = PKG_PATH + WP_RELATIVE_PATH; // path to waypoint csv
    }
    
    unsigned int get_num_waypoints()
    {
        std::ifstream file(WP_FULL_PATH);
        std::string line;
        unsigned int num_wps = 0;
        
        while (getline(file, line))
        {
            num_wps++;
        }

        return num_wps;
    }
    
    void read_waypoints()
    {
        set_wp_path();        
        unsigned int num_wps = get_num_waypoints(); // get number of waypoints to properly allocate the vector
        std::vector<float> row(NUM_COLS_IN_WP);
        wps.resize(num_wps, std::vector<float>(NUM_COLS_IN_WP)); // Pre-allocating vector will speed up run time instead of dynamically resizing over and over again
        
        std::ifstream data(WP_FULL_PATH);
        std::string line;
        int i = 0;
        while(std::getline(data, line))
        {
            std::stringstream lineStream(line);
            std::string cell;
            int j = 0;
            while(std::getline(lineStream, cell, ','))
            {
                row.at(j) = std::stof(cell); // Convert cell from string to float
                j++;
            }
            wps.at(i) = row;
            i++;
        }
    }

    float get_yaw(const nav_msgs::Odometry::ConstPtr &msg)
    {
        tf2::Quaternion q(
                        msg->pose.pose.orientation.x,
                        msg->pose.pose.orientation.y,
                        msg->pose.pose.orientation.z,
                        msg->pose.pose.orientation.w);
        return tf2::impl::getYaw(q);
    }

    std::vector<float> get_closest_wp(const float &x, const float &y, const float &phi)
    {
        float x_forward = LOOKAHEAD_DISTANCE * std::cos(phi) + x;
        float y_forward = LOOKAHEAD_DISTANCE * std::sin(phi) + y;

        float min_distance = HUGE_VAL;
        int closest_wp_idx = -1;

        for (int i = 0; i < wps.size(); i++)
        {
            float diff_x = wps[i][0] - x_forward;
            float diff_y = wps[i][1] - y_forward;

            float dist = std::sqrt(diff_x*diff_x + diff_y*diff_y);
            if (dist < min_distance)
            {
                min_distance = dist;
                closest_wp_idx = i;
            }
        }

        return wps[closest_wp_idx];
    }

    Eigen::Vector2f transform_wp_to_car_frame(const float &car_x_in_map_frame, 
                                              const float &car_y_in_map_frame, 
                                              const double &map_yaw, 
                                              const std::vector<float> &closest_wp)
    {
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

    float clip(const float &v, const float &min_v, const float &max_v)
    {
        if (v <= min_v)
            return min_v;
        else if (v >= max_v)
            return max_v;
        else
            return v;
    }

    void publish_marker_msg(const ros::Time &curr_time, const Eigen::Vector2f &wp_car_frame)
    {
        float marker_size = LOOKAHEAD_DISTANCE/3.0;
        static unsigned int marker_id = -1;
        marker_id++;

        visualization_msgs::Marker marker_msg;
        marker_msg.header.stamp = curr_time;
        marker_msg.ns = "pure_pursuit";
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

        vis_pub.publish(marker_msg);

        visualization_msgs::Marker marker_msg_del;
        marker_msg_del.header.stamp = curr_time;
        marker_msg_del.header.frame_id = marker_msg.header.frame_id;
        marker_msg_del.ns = marker_msg.ns;
        marker_msg_del.id = marker_id - 100;
        marker_msg_del.action = visualization_msgs::Marker::DELETE;
        vis_pub.publish(marker_msg_del);
    }

    void publish_drive_message(const ros::Time &curr_time, const float &gamma, const std::vector<float> &closest_wp)
    {
        ackermann_msgs::AckermannDriveStamped drive_msg;
        drive_msg.header.stamp = curr_time;
        drive_msg.header.frame_id = "base_link";
        drive_msg.drive.steering_angle = gamma;
        drive_msg.drive.speed = 3; // closest_wp.back();
        drive_pub.publish(drive_msg);
    }

public:
    PurePursuit()
    {
        read_waypoints();
        
        n = ros::NodeHandle("~");

        std::string odom_topic, drive_topic, vis_topic;
        odom_topic = "/opp_odom";
        drive_topic = "/opp_drive";
        vis_topic = "/waypoint_vis";
        
        // Create ROS subscribers and publishers
        drive_pub = n.advertise<ackermann_msgs::AckermannDriveStamped>(drive_topic, 1);
        vis_pub   = n.advertise<visualization_msgs::Marker>(vis_topic, 100);
        odom_sub  = n.subscribe(odom_topic, 1, &PurePursuit::pose_callback, this);   
    }

    void pose_callback(const nav_msgs::Odometry::ConstPtr &msg)
    {
        // Get the current state of the car
        float map_yaw = get_yaw(msg);
        float car_x_in_map_frame = msg->pose.pose.position.x;
        float car_y_in_map_frame = msg->pose.pose.position.y;
        
        // Find the current waypoint to track
        std::vector<float> closest_wp = get_closest_wp(car_x_in_map_frame, 
                                                       car_y_in_map_frame,
                                                       map_yaw);
        
        // Transform goal point to vehicle frame of reference
        Eigen::Vector2f wp_car_frame = transform_wp_to_car_frame(car_x_in_map_frame,
                                                                 car_y_in_map_frame,
                                                                 map_yaw,
                                                                 closest_wp);

        // Calculate curvature/steering angle
        float gamma = (2*wp_car_frame[1])/(wp_car_frame.squaredNorm());
        gamma = clip(gamma, -MAX_STEERING_ANGLE, MAX_STEERING_ANGLE);
        
        // Publish messages
        ros::Time curr_time = ros::Time::now();
        publish_marker_msg(curr_time, wp_car_frame);
        publish_drive_message(curr_time, gamma, closest_wp);
    }

};

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "team2_pure_pursuit_node");
    PurePursuit pp;
    ros::spin();
    return 0;
}