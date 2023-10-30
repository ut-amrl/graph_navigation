#!/usr/bin/env python
import time
import rospy
import argparse
import roslib
roslib.load_manifest('amrl_msgs')
from amrl_msgs.msg import NavStatusMsg, Localization2DMsg
from std_msgs.msg import Empty


class WaypointNav:
    def __init__(self, stop_time):
        self.stop_time = stop_time
        self.traj = []  # list of tuples (x, y, theta) waypoints
        self.next_goal = 0
        self.stop_detected_first_time = -1
        self.INITIAL = True
        rospy.Subscriber('/navigation_goal_status', NavStatusMsg, self.nav_callback, queue_size=1)
        rospy.Subscriber('/reset_nav_goals', Empty, self.reset_callback, queue_size=1)
        rospy.Subscriber('/set_waypoint', Localization2DMsg, self.set_waypoint_callback, queue_size=1)
        self.pub = rospy.Publisher('/move_base_simple/goal_amrl', Localization2DMsg, queue_size=1)

    def reset_callback(self, msg):
        self.next_goal = 0
        self.stop_detected_first_time = -1
        self.traj = []
        self.INITIAL = True

    def set_waypoint_callback(self, msg):
        self.traj.append((msg.pose.x, msg.pose.y, msg.pose.theta))

    def nav_callback(self, msg):
        if len(self.traj) == 0:
            return
        cur_traj = self.traj  # to protect against asynchronous updates to self.traj
        pub_msg = Localization2DMsg()
        if msg.status == 0:  # stopped
            if self.stop_detected_first_time == -1:
                self.stop_detected_first_time = rospy.get_rostime().secs
            else:
                if (rospy.get_rostime().secs - self.stop_detected_first_time > self.stop_time) or self.INITIAL:
                    if self.INITIAL:
                        time.sleep(20)  # wait for everything to setup properly and waypoints to be set
                    self.INITIAL = False
                    self.stop_detected_first_time = -1
                    # publish next goal
                    pub_msg.pose.x = cur_traj[self.next_goal][0]
                    pub_msg.pose.y = cur_traj[self.next_goal][1]
                    pub_msg.pose.theta = cur_traj[self.next_goal][2]
                    self.pub.publish(pub_msg)
                    time.sleep(4)
                    self.next_goal += 1
                    self.next_goal %= len(cur_traj)
        else:
            self.stop_detected_first_time = -1


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("--stop_time", type=int, default=10, help="Time to stop at each waypoint")
    args = parser.parse_args(rospy.myargv()[1:])

    rospy.init_node('waypoint_nav', anonymous=False)
    obj = WaypointNav(stop_time=args.stop_time)
    time.sleep(1)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down ROS waypoint nav node")
