import signal
import subprocess
from datetime import datetime

import roslib

roslib.load_manifest("graph_navigation")

import rospy
from std_msgs.msg import Joy


class Recorder:
    recording = False
    count = 0
    date = datetime.now().strftime("%Y-%m-%d")
    topics = "/bev/single/compressed /joint_states /odometry/filtered /tf /jackal_velocity_controller/cmd_vel /camera/rgb/image_raw/compressed".split()
    proc = None


def joy_callback(msg: Joy):
    startbutton = msg.buttons[1]
    endbutton = msg.buttons[2]

    if startbutton == 1 and not Recorder.recording:
        Recorder.recording = True
        print(f"Starting recording {Recorder.date}_{Recorder.count}")
        Recorder.proc = subprocess.Popen(
            [
                "/opt/ros/noetic/bin/rosbag",
                "record",
                "-O",
                f"{Recorder.date}_{Recorder.count}",
                *Recorder.topics,
            ]
        )
    elif endbutton == 1 and Recorder.recording:
        Recorder.proc.send_signal(signal.SIGINT)
        print(f"Finished recording {Recorder.date}_{Recorder.count}")
        Recorder.count += 1
        Recorder.recording = False


def main():
    rospy.init_node("recorder")

    rospy.Subscriber("/bluetooth_teleop/joy", Joy, joy_callback)
    rospy.spin()
