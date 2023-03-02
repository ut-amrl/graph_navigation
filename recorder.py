import signal
import subprocess
from datetime import datetime

import roslib

roslib.load_manifest("graph_navigation")

import rospy
from sensor_msgs.msg import Joy


class Recorder:
    recording = False
    topics = "/bev/single/compressed /joint_states /odometry/filtered /tf /jackal_velocity_controller/cmd_vel /camera/rgb/image_raw/compressed".split()
    proc = None


def joy_callback(msg: Joy):
    startbutton = msg.buttons[1]
    endbutton = msg.buttons[2]

    if startbutton == 1 and not Recorder.recording:
        Recorder.recording = True
        print("Starting recording")
        Recorder.proc = subprocess.Popen(
            [
                "/opt/ros/noetic/bin/rosbag",
                "record",
                *Recorder.topics,
            ]
        )
    elif endbutton == 1 and Recorder.recording:
        Recorder.proc.send_signal(signal.SIGINT)
        print("Finished recording")
        Recorder.count += 1
        Recorder.recording = False


def main():
    rospy.init_node("recorder")

    rospy.Subscriber("/bluetooth_teleop/joy", Joy, joy_callback)
    rospy.spin()

if __name__ == "__main__":
    main()
