#include <ros/ros.h>
#include <ros/console.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <ackermann_msgs/AckermannDriveStamped.h>

#include <cmath>
#include <algorithm>
#include <limits>

class Safety
{
// The class that handles emergency braking
private:
    ros::NodeHandle nh_;
    double speed_;
    float ttc_threshold_;
    
    // create ROS subscribers and publishers
    ros::Publisher bool_pub_;
    ros::Publisher brake_pub_;
    ros::Publisher min_ttc_pub_;

    ros::Subscriber scan_sub_;
    ros::Subscriber odom_sub_;

public:
    Safety()
    {
        nh_ = ros::NodeHandle("~");
        speed_ = 0.0;
        ttc_threshold_ = 0.35;

        // get topic names
        std::string bool_topic, brake_topic, scan_topic, odom_topic, min_ttc_topic;
        nh_.getParam("brake_bool_topic", bool_topic);
        nh_.getParam("brake_drive_topic", brake_topic);
        nh_.getParam("min_ttc_topic", min_ttc_topic);
        nh_.getParam("scan_topic", scan_topic);
        nh_.getParam("odom_topic", odom_topic);
        
        /*
        One publisher should publish to the /brake topic with an
        ackermann_msgs/AckermannDriveStamped brake message.

        One publisher should publish to the /brake_bool topic with a
        std_msgs/Bool message.

        You should also subscribe to the /scan topic to get the
        sensor_msgs/LaserScan messages and the /odom topic to get
        the nav_msgs/Odometry messages

        The subscribers should use the provided odom_callback and 
        scan_callback as callback methods

        NOTE that the x component of the linear velocity in odom is the speed
        */

        // create ROS subscribers and publishers
        bool_pub_  = nh_.advertise<std_msgs::Bool>(bool_topic, 100);
        brake_pub_  = nh_.advertise<ackermann_msgs::AckermannDriveStamped>(brake_topic, 100);
        min_ttc_pub_ = nh_.advertise<std_msgs::Float32>(min_ttc_topic, 100);
        scan_sub_ = nh_.subscribe(scan_topic, 100, &Safety::scan_callback, this);
        odom_sub_ = nh_.subscribe(odom_topic, 100, &Safety::odom_callback, this);
        
    }
    void odom_callback(const nav_msgs::Odometry::ConstPtr &odom_msg)
    {
        // update current speed
        speed_ = odom_msg->twist.twist.linear.x;
    }

    void scan_callback(const sensor_msgs::LaserScan::ConstPtr &scan_msg)
    {
        // calculate TTC
        float r;
        float theta;
        float r_dot;
        float ttc;
        float min_ttc = std::numeric_limits<float>::infinity(); // Infinity so it easily gets assigned
        for (int i = 0; i < scan_msg->ranges.size(); i++)
        {
            r = scan_msg->ranges[i];
            
            // Make sure scan value is valid
            if (r >= scan_msg->range_min && r <= scan_msg->range_max)
            {
                theta = scan_msg->angle_min + i*scan_msg->angle_increment;
                r_dot = -speed_ * std::cos(theta);
                ttc = r/std::max(-r_dot, float(0));
                
                if (ttc < min_ttc)
                {
                    min_ttc = ttc;
                }
            }
        }
        
        // Publish min_ttc for debugging purposes
        std_msgs::Float32 min_msg;
        min_msg.data = min_ttc;
        min_ttc_pub_.publish(min_msg);

        // publish drive/brake message
        std_msgs::Bool bool_msg;
        if (min_ttc < ttc_threshold_ && std::abs(speed_) > 0.0)
        {
            bool_msg.data = true;

            ackermann_msgs::AckermannDriveStamped brake_msg;
            brake_msg.drive.speed = 0.0;
            
            brake_pub_.publish(brake_msg);
            bool_pub_.publish(bool_msg);
        }
        else
        {
            bool_msg.data = false;
            bool_pub_.publish(bool_msg);
        }
    }

}; // End Safety class

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "safety_node");
    Safety sn;
    ros::spin();
    return 0;
}