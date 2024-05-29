import roslib
roslib.load_manifest('graph_navigation')

import rospy
import numpy as np
from amrl_msgs.msg import Localization2DMsg

import argparse





class Namespace:
    pub = None


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-delta", action="store_true")
    parser.add_argument("-x", type=float, default=5.0)
    parser.add_argument("-y", type=float, default=0)

    config = parser.parse_args()

    rospy.init_node("spoof_goal")
    Namespace.pub = rospy.Publisher("/move_base_simple/goal_amrl", Localization2DMsg, queue_size=1)

    while not rospy.is_shutdown():
        goal = Localization2DMsg()
        goal.pose.x = 0.0
        goal.pose.y = 0.0
        goal.pose.theta = 0.0
        Namespace.pub.publish(goal)
        # print("published")

        rospy.sleep(1.0)  # Adjust the sleep duration as needed
