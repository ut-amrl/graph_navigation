#!/usr/bin/env python

import rospy
import math
from geometry_msgs.msg import Twist
import roslib
roslib.load_manifest('amrl_msgs')
from amrl_msgs.msg import LDOSTwist
from std_msgs.msg import Header
from collections import deque
import time
import argparse


def sine_wave_publisher(args):
    rospy.init_node('sine_wave_command_publisher', anonymous=False)
    cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    ldos_cmd_vel_pub = rospy.Publisher('/ldos/cmd_vel', LDOSTwist, queue_size=10)
    print("Publishing sine wave commands to /cmd_vel and /ldos/cmd_vel... with amplitude: {}, frequency: {}, rate: {}".format(args.amplitude, args.frequency, args.rate))
    rate = rospy.Rate(args.rate)
    amplitude = args.amplitude  # Peak velocity in m/s
    frequency = args.frequency  # Frequency in Hz (one full cycle every 5 seconds)
    start_time = rospy.get_time()

    while not rospy.is_shutdown():
        current_time = rospy.get_time()
        elapsed_time = current_time - start_time
        velocity = amplitude * math.sin(2 * math.pi * frequency * elapsed_time)

        # Create and publish Twist message
        twist_msg = Twist()
        twist_msg.linear.x = velocity
        twist_msg.linear.y = 0.0
        twist_msg.linear.z = 0.0
        twist_msg.angular.x = 0.0
        twist_msg.angular.y = 0.0
        twist_msg.angular.z = 0.0

        ldos_twist_msg = LDOSTwist()
        ldos_twist_msg.linear = twist_msg.linear
        ldos_twist_msg.angular = twist_msg.angular
        ldos_twist_msg.sys_nano_time = int(time.time() * 1e9)

        cmd_vel_pub.publish(twist_msg)
        ldos_cmd_vel_pub.publish(ldos_twist_msg)

        rate.sleep()


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--amplitude', type=float, default=1.0, help='Peak velocity in m/s')
    parser.add_argument('--frequency', type=float, default=0.2, help='Frequency in Hz')
    parser.add_argument('--rate', type=int, default=15, help='Publish rate in Hz')
    args = parser.parse_args(rospy.myargv()[1:])  # Exclude the script name
    try:
        sine_wave_publisher(args)
    except rospy.ROSInterruptException:
        pass
