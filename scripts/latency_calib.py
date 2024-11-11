#!/usr/bin/env python

import rospy
import math
from geometry_msgs.msg import Twist
import roslib
roslib.load_manifest('amrl_msgs')
from amrl_msgs.msg import AckermannCurvatureDriveMsg
from std_msgs.msg import Header
from collections import deque
import time


def sine_wave_publisher():
    rospy.init_node('sine_wave_command_publisher', anonymous=False)
    cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    ackermann_pub = rospy.Publisher('/ackermann_curvature_drive', AckermannCurvatureDriveMsg, queue_size=10)
    latency_pub = rospy.Publisher('/latency_sim', Twist, queue_size=10)

    rate = rospy.Rate(15)

    amplitude = 1.0  # Peak velocity in m/s
    frequency = 0.2  # Frequency in Hz (one full cycle every 5 seconds)
    start_time = rospy.get_time()

    # Initialize latency queue
    latency_queue = deque()
    latency_duration = rospy.get_param("~latency_duration", 0.24)  # Default 0.24 seconds

    prev_latency_duration = latency_duration

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
        cmd_vel_pub.publish(twist_msg)

        # Create and publish AckermannCurvatureDriveMsg message
        ackermann_msg = AckermannCurvatureDriveMsg()
        ackermann_msg.header = Header()
        ackermann_msg.header.stamp = rospy.Time.now()
        ackermann_msg.velocity = velocity
        ackermann_msg.curvature = 0.0  # Straight line motion
        ackermann_pub.publish(ackermann_msg)

        # Update latency duration in case it was changed dynamically
        latency_duration = rospy.get_param("~latency_duration", None)
        if latency_duration is not None and latency_duration != prev_latency_duration:
            rospy.loginfo(f"Latency duration changed to {latency_duration} seconds")
            latency_queue.clear()
            prev_latency_duration = latency_duration
            continue

        # Add current twist message to the latency queue with timestamp
        latency_queue.append((current_time, twist_msg))

        # Check if any message in queue has surpassed the latency duration
        if latency_queue and (current_time - latency_queue[0][0] >= latency_duration):
            _, delayed_msg = latency_queue.popleft()  # Retrieve the delayed message
            latency_pub.publish(delayed_msg)  # Publish the delayed message to latency_sim

        rate.sleep()


if __name__ == '__main__':
    try:
        sine_wave_publisher()
    except rospy.ROSInterruptException:
        pass
