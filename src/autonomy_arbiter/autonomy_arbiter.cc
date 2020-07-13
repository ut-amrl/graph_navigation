//========================================================================
//  This software is free: you can redistribute it and/or modify
//  it under the terms of the GNU Lesser General Public License Version 3,
//  as published by the Free Software Foundation.
//
//  This software is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public License
//  Version 3 in the file COPYING that came with this distribution.
//  If not, see <http://www.gnu.org/licenses/>.
//========================================================================
/*!
\file    autonomy_arbiter.cc
\brief   Gating for autonomy.
\author  Joydeep Biswas, (C) 2020
*/
//========================================================================

#include <signal.h>
#include <stdio.h>
#include <stdlib.h>

#include <iostream>

#include "geometry_msgs/Twist.h"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "std_msgs/Bool.h"

#include "shared/util/helpers.h"
#include "shared/util/timer.h"

// Verbose level flag.
DECLARE_int32(v);

// The source topic to read autonomous commands from.
DEFINE_string(src_topic, "navigation/cmd_vel", "Source ROS topic");
// The destination topic to write autonomous commands to.
DEFINE_string(dest_topic, "cmd_vel", "Destination ROS topic");
// The destination topic to publish autonomy status to.
DEFINE_string(status_topic, "autonomy_arbiter/enabled", "Status ROS topic");
// The topic of the Joystick controller.
DEFINE_string(joystick_topic,
              "bluetooth_teleop/joy",
              "Joystick ROS topic");
// The button used to indicate start of autonomous operation.
DEFINE_uint64(start_btn_idx, 0, "Start button index");

// Drive publisher.
ros::Publisher drive_pub_;

// Status publisher.
ros::Publisher status_pub_;

// Enable drive.
bool enable_drive_ = false;

void DriveCallback(const geometry_msgs::Twist& msg) {
  if (enable_drive_) {
    drive_pub_.publish(msg);
  } else {
    // TODO: Ramp down to 0.
  }
}

void JoystickCallback(const sensor_msgs::Joy& msg) {
  static const double kDebounceTimeout = 0.5;
  static double t_debounce_start_ = 0;
  if (GetMonotonicTime() < t_debounce_start_ + kDebounceTimeout) {
    return;
  }
  bool need_to_debounce = false;
  if (enable_drive_) {
    for (size_t i = 0; i < msg.buttons.size(); ++i) {
      if (msg.buttons[i] == 1) {
        enable_drive_ = false;
        if (FLAGS_v > 0) {
          printf("Drive disabled.\n");
        }
        need_to_debounce = true;
      }
    }
  } else {
    bool start_pressed = false;
    bool all_else_unpressed = true;
    for (size_t i = 0; i < msg.buttons.size(); ++i) {
      if (i == FLAGS_start_btn_idx) {
        start_pressed = msg.buttons[i] == 1;
      } else {
        all_else_unpressed = all_else_unpressed && msg.buttons[i] == 0;
      }
    }
    enable_drive_ = start_pressed && all_else_unpressed;
    if (enable_drive_) {
      need_to_debounce = true;
      if (FLAGS_v > 0) {
        printf("Drive enabled.\n");
      }
    }
  }
  // See if recording should start.
  if (msg.buttons.size() >=15 && 
      msg.buttons[11] == 1) {
    static bool recording = false;
    if (recording && msg.buttons[1] == 1) {
      recording = false;
      if (system("killall rosbag") != 0) {
        printf("Unable to kill rosbag!\n");
      } else {
        printf("Stopped recording rosbag.\n");
      }
      need_to_debounce = true;
    } else if (!recording && msg.buttons[3] == 1) {
      
      printf("Starting recording rosbag...\n");
      if (system("rosbag record /status /velodyne_points /scan /imu/data /jackal_velocity_controller/odom /gps/fix /gps/vel /imu/data_raw /odometry/filtered /odometry/gps /velodyne_2dscan /velodyne_2dscan_high_beams /tf &") != 0) {
        printf("Unable to record\n");
      } else {
        printf("Started recording rosbag.\n");
        recording = true;
      }
      need_to_debounce = true;
    }
  }
      
  if (need_to_debounce) {
    t_debounce_start_ = GetMonotonicTime();
  }
  std_msgs::Bool status_msg;
  status_msg.data = enable_drive_;
  status_pub_.publish(status_msg);
}

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, false);
  // Initialize ROS.
  ros::init(argc, argv, "autonomy_arbiter", ros::init_options::NoSigintHandler);

  if (FLAGS_v > 0) {
    printf("Autonomy arbiter\n");
    printf("Source topic: %s\n", FLAGS_src_topic.c_str());
    printf("Destination topic: %s\n", FLAGS_dest_topic.c_str());
    printf("Joystick topic: %s\n", FLAGS_joystick_topic.c_str());
  }
  ros::NodeHandle n;
  ros::Subscriber joystick_sub =
      n.subscribe(FLAGS_joystick_topic, 1, &JoystickCallback);
  ros::Subscriber drive_sub =
      n.subscribe(FLAGS_src_topic, 1, &DriveCallback);
  drive_pub_ = n.advertise<geometry_msgs::Twist>(FLAGS_dest_topic, 1);
  status_pub_ = n.advertise<std_msgs::Bool>(FLAGS_status_topic, 1);
  ros::spin();
  return 0;
}