#!/usr/bin/env python
from __future__ import print_function
import sys
import numpy as np

#ROS Imports
import rospy
from sensor_msgs.msg import Image, LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive


class WallFollow:
    """ Implement Wall Following on the car
    """
    def __init__(self):
        # PID CONTROL PARAMS
        self.kp = 0.8
        self.ki = 0.0
        self.kd = 0.8e-03

        self.servo_offset = 0.0
        self.prev_error = 0.0
        self.prev_diff = 0.0
        self.prev_time = rospy.Time.now()
        self.integral = 0.0
        self.isDirtyDeriv = True # Use dirty derivative or regular derivative
        self.SIGMA = 0.05 # 20 Hz

        #WALL FOLLOW PARAMS
        self.ANGLE_RANGE = 270 # Hokuyo 10LX has 270 degrees scan
        HALL_WIDTH = 1.75
        self.DESIRED_DISTANCE_LEFT = HALL_WIDTH/2. # meters
        self.CAR_LENGTH = 0.50 # Traxxas Rally is 20 inches or 0.5 meters
        self.MAX_VELOCITY = 1.50
        self.LOOKAHEAD_DIST = self.MAX_VELOCITY/2.
        self.MAX_STEERING_ANGLE = np.deg2rad(24)
        self.LEFT_ORTHOGONAL_BEAM_ANGLE = np.deg2rad(90) # angle that is perpindicular to the car
        self.PHI = np.deg2rad(45) # The angle of 2nd beam relative to left orthagonal beam angle
        self.THETA = self.LEFT_ORTHOGONAL_BEAM_ANGLE-self.PHI # phi relative to the lidar angles
        self.SIN_PHI = np.sin(self.PHI)
        self.COS_PHI = np.cos(self.PHI)

        #Topics & Subs, Pubs
        lidarscan_topic = '/scan'
        drive_topic = '/vesc/high_level/ackermann_cmd_mux/input/nav_1'

        self.lidar_sub = rospy.Subscriber(lidarscan_topic, LaserScan, self.lidar_callback, queue_size=1) # Subscribe to LIDAR
        self.drive_pub = rospy.Publisher(drive_topic, AckermannDriveStamped, queue_size=1) # Publish to drive

    # Saturate input
    def sat(self, u, upper_lim, lower_lim=None):
        if lower_lim is None:
            lower_lim = -upper_lim

        if u > upper_lim:
            u = upper_lim
        elif u < lower_lim:
            u = lower_lim
        return u

    def getRange(self, angle, data):
        # data: single message from topic /scan
        # angle: between -135 to 135 degrees, where 0 degrees is straight
        # Outputs length in meters to object with angle in lidar scan field of view
        
        ranges = np.array(data.ranges)
        num_thetas = int(np.round((data.angle_max - data.angle_min)/data.angle_increment +1))
        thetas = np.linspace(data.angle_min, data.angle_max, num_thetas) # vector of angles
        with np.errstate(invalid='ignore'): # warning raised when comparing nan values
            invalid_idxs = np.argwhere(np.isnan(ranges) | (ranges < data.range_min-10) | (ranges > data.range_max+10)) # get indices of invalid values
        
        thetas[invalid_idxs] = np.inf # change corresponding invalid values in the thetas vector
        angle_idx = np.abs(thetas - angle).argmin() # find index closest to the given angle
        
        return ranges[angle_idx] # return valid range value

    def getAlpha(self, b, data):
        a = self.getRange(self.THETA, data)
        alpha = np.arctan2(b - a*self.COS_PHI, a*self.SIN_PHI) # positive alpha means car is turned towards the wall (left)

        return alpha

    def followLeft(self, data):
        #Follow left wall as per the algorithm
        b = self.getRange(self.LEFT_ORTHOGONAL_BEAM_ANGLE, data)
        alpha = self.getAlpha(b, data)
        
        D_t = b*np.cos(alpha)
        D_lookahead = D_t - self.LOOKAHEAD_DIST*np.sin(alpha)
        error = D_lookahead - self.DESIRED_DISTANCE_LEFT
        
        return error

    def pid_control(self, error, data):
        now = data.header.stamp
        dt = (now - self.prev_time).to_sec()
        self.prev_time = now

        # Dirty derivative applies a low pass filter to the error
        if self.isDirtyDeriv:
            a1 = (2*self.SIGMA - dt) / (2*self.SIGMA + dt)
            a2 = 2 / (2*self.SIGMA + dt)
            edot = a1 * self.prev_diff + a2 * (error-self.prev_error)
        else:
            edot = (error-self.prev_error)/dt
            
        self.integral += (0.5*dt)*(error+self.prev_error)
        
        steering_angle_unsat = self.kp*error + self.ki*self.integral + self.kd*edot
        steering_angle_sat = self.sat(steering_angle_unsat, self.MAX_STEERING_ANGLE)

        # Integrator anti-windup
        if self.ki != 0:
            self.integral += dt/self.ki * (steering_angle_sat-steering_angle_unsat)
        
        self.prev_error = error
        self.prev_diff = edot

        mag_angle = abs(np.rad2deg(steering_angle_sat)) 
        if 0 <= mag_angle < 10:
            velocity = self.MAX_VELOCITY
        elif 10 <= mag_angle < 20:
            velocity = 2*self.MAX_VELOCITY/3
        else:
            velocity = self.MAX_VELOCITY/3
        
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = now
        drive_msg.header.frame_id = "laser"
        drive_msg.drive.steering_angle = steering_angle_sat
        drive_msg.drive.speed = velocity
        self.drive_pub.publish(drive_msg)

    def lidar_callback(self, data):
        """ 
        """
        error = self.followLeft(data)
        #send error to pid_control
        self.pid_control(error, data)

def main(args):
    rospy.init_node("wall_follow_node", anonymous=True)
    wf = WallFollow()
    rospy.sleep(0.1)
    
    rospy.loginfo("Starting wall_follow_node")
    rospy.spin()

if __name__=='__main__':
	main(sys.argv)
