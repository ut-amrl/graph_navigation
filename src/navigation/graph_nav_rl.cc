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
\file    graph_nav_rl.cc
\brief   Slim wrapper to simulator ang graph nav for reinforcement learning.
\author  Joydeep Biswas, (C) 2022
*/
//========================================================================

#include <stdio.h>

#include <iostream>

#include "glog/logging.h"

#include "ros/ros.h"

#include "simulator/simulator.h"
#include "navigation/navigation.h"
#include "navigation/graph_nav_rl.h"

#include "amrl_msgs/AckermannCurvatureDriveMsg.h"
#include "amrl_msgs/Localization2DMsg.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/LaserScan.h"

namespace {
Simulator* simulator_ptr_ = nullptr;
ros::NodeHandle* ros_nh_ = nullptr;
}  // namespace

namespace graph_nav_rl {

extern "C" int Test(const char* str) {
  printf("%s\n", str);
  return 42;
}

extern "C"
void Init() {
  int argc = 1;
  char argv0[] = "graph_nav_rl";
  char* argv[] = {argv0};
  ros::init(argc, argv, "ut_automata_rl_simulator");
  ros_nh_ = new ros::NodeHandle();
  if (simulator_ptr_ == nullptr) {
    simulator_ptr_ = new Simulator();
    simulator_ptr_->Init(*ros_nh_);
  }
}

extern "C"
void Step() {
  CHECK(simulator_ptr_ != nullptr);
  amrl_msgs::AckermannCurvatureDriveMsg cmd;
  nav_msgs::Odometry odom_msg;
  sensor_msgs::LaserScan scan_msg;
  amrl_msgs::Localization2DMsg localization_msg;
  simulator_ptr_->Step(cmd, &odom_msg, &scan_msg, &localization_msg);
}

extern "C"
void Reset() {
  CHECK(simulator_ptr_ != nullptr);
  simulator_ptr_->ResetState();
}

}  // namespace graph_nav_rl