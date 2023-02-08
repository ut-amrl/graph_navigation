import roslib
roslib.load_manifest('graph_navigation')

import rospy
import numpy as np
from amrl_msgs.msg import Localization2DMsg

import argparse


class Namespace:
    pub = None

    delta = False
    x = 0.0
    y = 0.0


def loc_callback(msg: Localization2DMsg):
    pos = (msg.pose.x, msg.pose.y, msg.pose.theta)

    if Namespace.delta:
        x = Namespace.x * np.cos(pos[2]) - Namespace.y * np.sin(pos[2])
        y = Namespace.x * np.sin(pos[2]) + Namespace.y * np.cos(pos[2])

        x = pos[0] + x
        y = pos[1] + y
    else:
        x = Namespace.x
        y = Namespace.y

    goal = (x, y)
    msg = Localization2DMsg()
    msg.pose.x = goal[0]
    msg.pose.y = goal[1]
    msg.pose.theta = pos[2]
    Namespace.pub.publish(msg)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-delta", action="store_true")
    parser.add_argument("-x", type=float, default=5.0)
    parser.add_argument("-y", type=float, default=0)

    config = parser.parse_args()
    Namespace.delta = config.delta
    Namespace.x = config.x
    Namespace.y = config.y

    rospy.init_node("spoof_goal")
    Namespace.pub = rospy.Publisher("/move_base_simple/goal_amrl", Localization2DMsg, queue_size=1)

    l = rospy.Subscriber("/localization", Localization2DMsg, loc_callback)

    rospy.spin()
