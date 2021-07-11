#!/usr/bin/env python
import rospy
import math
import json
from std_msgs.msg import String
from amrl_msgs.msg import Localization2DMsg
import argparse

parser = argparse.ArgumentParser()

parser.add_argument('--loop', action='store_true')
parser.add_argument('--map', type='str', default='UT_Campus')
parser.add_argument('--waypoints', type='str', default='AHG_dock.json', help='json file containing an array of waypoints')

args = parser.parse_args()

waypoints = json.load(args.waypoints)

class WaypointNavigator():
  WAYPOINT_THRESHOLD = 0.75
  def __init__(self, map, waypoints):
    self.map = map
    self.waypoints = waypoints
    self.current_waypoint = 0
    rospy.Subscriber("localization", Localization2DMsg, self.loc_callback)
    self.nav_pub = rospy.Publisher("/move_base_simple/goal", Localization2DMsg, queue_size=1)
    self.goal_msg = Localization2DMsg()
    self.goal_msg.map = map

  def loc_callback(self, loc):
    target_waypoint = waypoints[self.current_waypoint]
    if WaypointNavigator.is_close(target_waypoint, loc.pose):
      self.current_waypoint += 1
      if (self.current_waypoint >= len(waypoints)):
        if (args.loop):
          print("Circuit Complete, restarting...")
          self.current_waypoint = 0
        else:
          print("Completed waypoint navigation, exiting...")
          exit(0)
      self.send_nav_command()

  def send_nav_command(self):
    target_waypoint = waypoints[current_waypoint]

    goal_msg.pose.x = target_waypoint["x"]
    goal_msg.pose.y = target_waypoint["y"]
    goal_msg.pose.theta = target_waypoint["theta"]

    self.nav_pub.send(goal_msg)  

  @classmethod
  def is_close(cls, target, pose):
    target_x = target["x"]
    target_y = target["y"]
    target_theta = target["theta"]
    diff = math.sqrt(pow(pose.x - target_x, 2) + pow(pose.y - target_y, 2)) + abs(pose.theta - target_theta)
    return diff < cls.WAYPOINT_THRESHOLD


def setup_ros_node():
  rospy.init_node('waypoint_navigiator')
  
  WaypointNavigator(args.map, waypoints)

  rospy.spin()


setup_ros_node()



