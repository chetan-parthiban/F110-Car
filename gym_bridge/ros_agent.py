import rospy
from ackermann_msgs.msg import AckermannDriveStamped


class RosAgent():

    def __init__(self):
        self.drive_topic = rospy.get_param('opp_drive_topic')
        self.drive_sub = rospy.Subscriber(self.drive_topic, AckermannDriveStamped, self.drive_callback, queue_size=1)
        self.speed = 0
        self.steer = 0

    def plan(self, state):
        return self.speed, self.steer

    def drive_callback(self, drive_msg):
        self.speed = drive_msg.drive.speed
        self.steer = drive_msg.drive.steering_angle