#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
from visualization_msgs.msg import Marker
from tf.transformations import euler_from_quaternion
from tf.transformations import quaternion_from_euler
import tf2_ros
import tf2_geometry_msgs

import numpy as np

class PurePursuit:
    def __init__(self):
        # Create ROS subscribers and publishers.
        odom_topic = '/odom'
        drive_topic = '/nav'
        vis_topic = '/waypoint_vis'

        self.L = 1.25
        self.MAX_STEERING_ANGLE = np.deg2rad(24)
        self.wps = np.genfromtxt('/home/brandon/f110_ws/src/f110_ros/pure_pursuit/data/pure_pursuit_waypoints.csv', delimiter=',')
        
        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer)
        
        self.vis_pub = rospy.Publisher(vis_topic, Marker, queue_size=1)
        self.drive_pub = rospy.Publisher(drive_topic, AckermannDriveStamped, queue_size=1) # Publish to drive
        self.odom_sub = rospy.Subscriber(odom_topic, Odometry, self.odom_callback, queue_size=1)



    def odom_callback(self, msg):
        now = rospy.Time.now()
        
        # find the current waypoint to track using methods mentioned in lecture
        quaternion = np.array([ msg.pose.pose.orientation.x, 
                                msg.pose.pose.orientation.y, 
                                msg.pose.pose.orientation.z, 
                                msg.pose.pose.orientation.w])
        euler = euler_from_quaternion(quaternion)

        car_x_in_map_frame = msg.pose.pose.position.x
        car_y_in_map_frame = msg.pose.pose.position.y
        map_yaw = euler[2]
        closest_wp = self.get_closest_wp(car_x_in_map_frame, car_y_in_map_frame, map_yaw)
        
        # Transform goal point to vehicle frame of reference
        c, s = np.cos(map_yaw), np.sin(map_yaw)
        R_map_in_car_frame = np.array([[c, s],[-s, c]])
        d_car_in_map_frame = np.array([car_x_in_map_frame, car_y_in_map_frame]).reshape(2, 1)
        H_map_in_car_frame = np.block([
            [R_map_in_car_frame, np.matmul(-R_map_in_car_frame, d_car_in_map_frame)],
            [np.zeros([1, 2]), np.ones([1, 1])]
        ])
        wp_car_frame = np.matmul(H_map_in_car_frame, np.concatenate((closest_wp[:2], np.ones(1))))[:2]
        
        # Calculate curvature/steering angle
        gamma = (2*wp_car_frame[1])/(np.linalg.norm(wp_car_frame)**2)
        gamma = np.clip(gamma, a_min=-self.MAX_STEERING_ANGLE, a_max=self.MAX_STEERING_ANGLE)

        rospy.loginfo_throttle(0.5, "map_yaw {}".format(np.rad2deg(map_yaw)))
        rospy.loginfo_throttle(0.5, closest_wp)
        rospy.loginfo_throttle(0.5, wp_car_frame)
        # rospy.loginfo_throttle(0.5, "gamma: {}\n".format(np.rad2deg(gamma)))
        
        marker_size = self.L/3.0
        
        marker_msg = Marker()
        marker_msg.header.stamp = now
        marker_msg.type = Marker.SPHERE
        marker_msg.action = Marker.ADD
        
        marker_msg.header.frame_id = "base_link"
        marker_msg.pose.position.x = wp_car_frame[0]
        marker_msg.pose.position.y = wp_car_frame[1]
        marker_msg.pose.position.z = 0.0

        marker_msg.pose.orientation.x = 0.0
        marker_msg.pose.orientation.y = 0.0
        marker_msg.pose.orientation.z = 0.0
        marker_msg.pose.orientation.w = 1.0

        marker_msg.scale.x = marker_size
        marker_msg.scale.y = marker_size
        marker_msg.scale.z = marker_size
        marker_msg.color.a = 1.0
        marker_msg.color.r = 0.0
        marker_msg.color.g = 0.0
        marker_msg.color.b = 1.0
        self.vis_pub.publish(marker_msg)

        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = now
        drive_msg.header.frame_id = "laser"
        drive_msg.drive.steering_angle = gamma
        drive_msg.drive.speed = closest_wp[-1]*2
        self.drive_pub.publish(drive_msg)

    def get_closest_wp(self, x, y, phi):
        """
        x is the current x position of the car
        y is the current y position of the car
        phi is the orientation of the car
        """
        x_forward = self.L * np.cos(phi) + x
        y_forward = self.L * np.sin(phi) + y

        diff = np.zeros(4)
        diff[0] = x_forward
        diff[1] = y_forward

        wps_diff = self.wps - diff
        dist = np.linalg.norm(wps_diff[:, :2], axis=1)
        closest_wp_idx = np.argmin(dist)
        return self.wps[closest_wp_idx, :]


def main():
    rospy.init_node('pure_pursuit_node')
    pp = PurePursuit()
    rospy.spin()
if __name__ == '__main__':
    main()