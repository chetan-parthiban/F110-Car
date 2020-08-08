#!/usr/bin/env python
from __future__ import print_function
import sys
import math
import numpy as np
# from scipy.ndimage.filters import uniform_filter1d

#ROS Imports
import rospy
from sensor_msgs.msg import Image, LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

class reactive_follow_gap:
    def __init__(self):
        #Topics & Subscriptions,Publishers
        lidarscan_topic = '/scan'
        drive_topic = '/vesc/high_level/ackermann_cmd_mux/input/nav_1'

        self.lidar_sub = rospy.Subscriber(lidarscan_topic, LaserScan, self.lidar_callback, queue_size=1) # Subscribe to LIDAR
        self.drive_pub = rospy.Publisher(drive_topic, AckermannDriveStamped, queue_size=1) # Publish to drive

        # Constants
        self.ANGLE_RANGE = 270 # Hokuyo 10LX has 270 degrees scan
        self.SAFETY_BUBBLE_RADIUS = 0.4
        self.MEAN_WINDOW_SIZE = 5
        self.PREPROCESS_THRESHOLD = 10
        self.SAFETY_ANGLE = np.deg2rad(80)
        
        self.GAP_PERCENTILE = 40
        self.PERCENTILE_TURNING_RANGE = 1.2
        self.STEERING_DAMPER = 0.3
        
        self.MAX_STEERING_ANGLE = np.deg2rad(22.5)
        self.MAX_VELOCITY = 3
        self.MIN_VELOCITY = 1.5
        self.SIGMOID_BUFFER = np.deg2rad(8)
        self.SIGMOID_SLOPE = np.rad2deg(0.5)
        
        # LiDAR data parameters
        self.angle_increment = 0
        self.angle_min = 0
        self.angle_max = 0

        self.closest_angle = 0
        self.closest_range = 0
        self.percentile_range = 0
        self.steering_angle_sat = 0
        self.velocity = self.MAX_VELOCITY

    def angle2index(self, theta, angle_min=None, angle_increment=None):
        if angle_min is None:
            angle_min = self.angle_min
        if angle_increment is None:
            angle_increment = self.angle_increment

        return int(np.round((theta-angle_min)/angle_increment))

    def index2angle(self, index, angle_min=None, angle_increment=None):
        if angle_min is None:
            angle_min = self.angle_min
        if angle_increment is None:
            angle_increment = self.angle_increment

        return angle_min + index*angle_increment
    
    def preprocess_lidar(self, data):
        """ Preprocess the LiDAR scan array. Expert implementation includes:
            1.Setting each value to the mean over some window
            2.Rejecting high values (eg. > 3m)
        """
        self.angle_increment = data.angle_increment
        self.angle_min = data.angle_min
        self.angle_max = data.angle_max
        proc_ranges = np.array(data.ranges)

        with np.errstate(invalid='ignore'): # warning raised when comparing nan values
            invalid_idxs = np.argwhere(np.isnan(proc_ranges)) # get indices of invalid values

        proc_ranges[invalid_idxs] = 10
        proc_ranges[proc_ranges > self.PREPROCESS_THRESHOLD] = 10
        # proc_ranges = uniform_filter1d(proc_ranges, self.MEAN_WINDOW_SIZE)

        return proc_ranges

    def clear_safety_bubble(self, proc_ranges):
        # Find closest point to LiDAR
        # Don't care if an object is close behind the car
        min_safety_idx = self.angle2index(-self.SAFETY_ANGLE)
        max_safety_idx = self.angle2index(self.SAFETY_ANGLE)
        closest_idx = np.argmin(proc_ranges[min_safety_idx:max_safety_idx]) # closest index between safety range
        self.closest_angle = self.index2angle(closest_idx, angle_min=-self.SAFETY_ANGLE) # angle to the closest range
        closest_idx = self.angle2index(self.closest_angle) # change index to match the entir proc_ranges array

        # Eliminate all points inside 'bubble' (set them to zero)
        self.closest_range = proc_ranges[closest_idx]
        theta = np.arctan(self.SAFETY_BUBBLE_RADIUS/self.closest_range)
        t_max = self.closest_angle + theta
        max_idx = self.angle2index(t_max)
        t_min = self.closest_angle - theta
        min_idx = self.angle2index(t_min)
        
        bubble = proc_ranges[min_idx:max_idx+1]
        bubble[bubble < (self.closest_range+self.SAFETY_BUBBLE_RADIUS)] = 0
        proc_ranges[min_idx:max_idx+1] = bubble # technically don't need this line because of pointers
        
        return proc_ranges

    def find_max_gap(self, ranges, func, value):
        """ Return the start index & end index of the max gap in ranges
        """
        # Create an array that is 1 where a is nonzero, and pad each end with an extra 0.
        is_nonzero = np.concatenate(([0], func(ranges, value).view(np.int8), [0]))
        abs_diff = np.abs(np.diff(is_nonzero))
        
        # Gaps start and end where abs_diff is 1.
        # Gaps is an (N, 2) where N is the number of gaps
        # First column is start_idx
        # Second column is first zero index
        gaps = np.where(abs_diff == 1)[0].reshape(-1, 2)


        max_gap_idx = np.argmax(np.diff(gaps)) # Find the biggest gap
        start_idx = gaps[max_gap_idx, 0]
        end_idx = gaps[max_gap_idx, 1] - 1
        return start_idx, end_idx
    
    def find_best_point(self, start_idx, end_idx, proc_ranges):
        """ start_idx & end_idx are start and end indicies of max-gap range, respectively
            Return index of best point in ranges
	        Naive: Choose the furthest point within ranges and go there
        """
        max_gap_range = proc_ranges[start_idx:end_idx+1]
        start_angle = self.index2angle(start_idx)
        self.percentile_range = np.percentile(max_gap_range, self.GAP_PERCENTILE)
        uppper_start_idx, upper_end_idx = self.find_max_gap(max_gap_range, np.greater_equal, self.percentile_range)

        best_idx_in_gap = int(np.round(((upper_end_idx-uppper_start_idx)*0.5) + uppper_start_idx)) # midpoint between the two indices
        best_angle = self.index2angle(best_idx_in_gap, angle_min=start_angle)
        best_idx_in_proc_ranges = self.angle2index(best_angle)

        return best_idx_in_proc_ranges

    def lidar_callback(self, data):
        """ Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message
        """
        # Process scan data
        proc_ranges = self.preprocess_lidar(data)
        
        # Eliminate all points inside 'bubble' (set them to zero)
        proc_ranges = self.clear_safety_bubble(proc_ranges)

        # if self.closest_range < self.CLOSEST_RANGE_LIMIT or self.is_first_run:
            # self.is_first_run = False
            
        # Find max length gap 
        start_idx, end_idx = self.find_max_gap(proc_ranges, np.not_equal, 0)

        # Find the best point in the gap
        best_idx = self.find_best_point(start_idx, end_idx, proc_ranges)
        steering_angle = self.index2angle(best_idx)

        self.steering_angle_sat = np.clip(steering_angle, a_min=-self.MAX_STEERING_ANGLE, a_max=self.MAX_STEERING_ANGLE)
        if self.percentile_range < self.PERCENTILE_TURNING_RANGE:
            self.steering_angle_sat *= self.STEERING_DAMPER
        
        mag_angle = abs(self.steering_angle_sat)
        self.velocity = (self.MIN_VELOCITY-self.MAX_VELOCITY) * \
            (1/(1 + np.exp(-self.SIGMOID_SLOPE*(mag_angle-self.SIGMOID_BUFFER)))) + self.MAX_VELOCITY
        
        self.velocity = np.clip(self.velocity, a_min=0, a_max=self.MAX_VELOCITY)

        # rospy.loginfo_throttle(0.5, "closest_angle: %f"%np.rad2deg(self.closest_angle))
        # rospy.loginfo_throttle(0.5, "closest_range: %f"%self.closest_range)
        # rospy.loginfo_throttle(0.5, "steering_angle: %f\n"%np.rad2deg(self.steering_angle_sat))
        # rospy.loginfo_throttle(0.5, "start_angle: %f"%np.rad2deg(self.index2angle(start_idx)))
        # rospy.loginfo_throttle(0.5, "end_angle: %f\n"%np.rad2deg(self.index2angle(end_idx)))

            
        #Publish Drive message
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = "laser"
        drive_msg.drive.steering_angle = self.steering_angle_sat
        drive_msg.drive.speed = self.velocity
        self.drive_pub.publish(drive_msg)

def main(args):
    rospy.init_node("follow_gap_node", anonymous=True)
    rfgs = reactive_follow_gap()
    rospy.sleep(0.1)
    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)
