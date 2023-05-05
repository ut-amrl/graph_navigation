import numpy as np
import rospy
import time
from std_msgs.msg import Int64
from geometry_msgs.msg import PoseStamped

STOP_TIME = 28  # seconds


class TourTrajectory:
    def __init__(self):
        self.ahg = (-10.031, 16.859, 0, 1)  # (xloc, yloc, zorient, worient)
        self.nhb = (61.820, -84.904, 0.135, 0.991)
        self.gdc = (80.116, -227.031, 0.707, 0.707)
        self.traj = [self.ahg, self.nhb, self.gdc, self.nhb, self.ahg]
        self.next_goal = 0
        self.stop_detected_first_time = -1

        rospy.Subscriber('/_nav_state', Int64, self.nav_callback, queue_size=1)
        self.pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)

    def nav_callback(self, msg):
        pub_msg = PoseStamped()
        if self.next_goal < len(self.traj):
            if msg.data == 0:  # stopped
                if self.stop_detected_first_time == -1:
                    self.stop_detected_first_time = rospy.get_rostime().secs
                else:
                    if rospy.get_rostime().secs - self.stop_detected_first_time > STOP_TIME:
                        self.stop_detected_first_time = -1
                        # publish next goal
                        pub_msg.pose.position.x = self.traj[self.next_goal][0]
                        pub_msg.pose.position.y = self.traj[self.next_goal][1]
                        pub_msg.pose.orientation.z = self.traj[self.next_goal][2]
                        pub_msg.pose.orientation.w = self.traj[self.next_goal][3]
                        self.pub.publish(pub_msg)
                        time.sleep(4)
                        self.next_goal += 1
            else:
                self.stop_detected_first_time = -1


def setup_ros_node():
    rospy.init_node('tour_trajectory', anonymous=False)
    obj = TourTrajectory()
    time.sleep(1)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down ROS tour trajectory node")


setup_ros_node()
