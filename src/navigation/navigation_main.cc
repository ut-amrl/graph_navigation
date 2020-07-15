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
\file    navigation_main.cc
\brief   Main entry point for reference Navigation implementation
\author  Joydeep Biswas, (C) 2019
*/
//========================================================================

#include <signal.h>
#include <stdlib.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <vector>

#include "amrl_msgs/Localization2DMsg.h"
#include "amrl_msgs/Pose2Df.h"
#include "glog/logging.h"
#include "gflags/gflags.h"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "gflags/gflags.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "sensor_msgs/LaserScan.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "nav_msgs/Odometry.h"
#include "ros/ros.h"
#include "shared/math/math_util.h"
#include "shared/util/timer.h"
#include "shared/ros/ros_helpers.h"
#include "std_msgs/Bool.h"

#include "navigation.h"

using math_util::DegToRad;
using math_util::RadToDeg;
using navigation::Navigation;
using ros::Time;
using ros_helpers::Eigen3DToRosPoint;
using ros_helpers::Eigen2DToRosPoint;
using ros_helpers::RosPoint;
using ros_helpers::SetRosVector;
using std::string;
using std::vector;
using Eigen::Vector2f;

DEFINE_string(laser_topic, "velodyne_2dscan", "Name of ROS topic for LIDAR data");
DEFINE_string(odom_topic, "jackal_velocity_controller/odom", "Name of ROS topic for odometry data");
DEFINE_string(loc_topic, "localization", "Name of ROS topic for localization");
DEFINE_string(init_topic,
              "initialpose",
              "Name of ROS topic for initialization");
// Name of topic controlling whether to enable autonomous nav or not.
DEFINE_string(enable_topic, "autonomy_arbiter/enabled",
    "ROS topic that indicates whether autonomy is enabled or not.");
DEFINE_string(map,
              "maps/Joydeepb-Home/Joydeepb-Home.navigation.txt",
              "Name of navigation map file");
DECLARE_string(helpon);
DECLARE_int32(v);
DECLARE_double(dt);

bool run_ = true;
sensor_msgs::LaserScan last_laser_msg_;
Navigation* navigation_ = nullptr;

void EnablerCallback(const std_msgs::Bool& msg) {
  if (navigation_) {
    navigation_->Enable(msg.data);
  }
}

void LaserCallback(const sensor_msgs::LaserScan& msg) {
  if (FLAGS_v > 0) {
    printf("Laser t=%f, dt=%f\n",
           msg.header.stamp.toSec(),
           GetWallTime() - msg.header.stamp.toSec());
  }
  // Location of the laser on the robot. Assumes the laser is forward-facing.
  const Vector2f kLaserLoc(0.065, 0);
  static float cached_dtheta_ = 0;
  static float cached_angle_min_ = 0;
  static size_t cached_num_rays_ = 0;
  static vector<Vector2f> cached_rays_;
  if (cached_angle_min_ != msg.angle_min ||
      cached_dtheta_ != msg.angle_increment ||
      cached_num_rays_ != msg.ranges.size()) {
    cached_angle_min_ = msg.angle_min;
    cached_dtheta_ = msg.angle_increment;
    cached_num_rays_ = msg.ranges.size();
    cached_rays_.resize(cached_num_rays_);
    for (size_t i = 0; i < cached_num_rays_; ++i) {
      const float a =
          cached_angle_min_ + static_cast<float>(i) * cached_dtheta_;
      cached_rays_[i] = Vector2f(cos(a), sin(a));
    }
  }
  CHECK_EQ(cached_rays_.size(), msg.ranges.size());
  static vector<Vector2f> point_cloud_;
  point_cloud_.resize(cached_rays_.size());
  for (size_t i = 0; i < cached_num_rays_; ++i) {
    const float r =
      ((msg.ranges[i] > msg.range_min && msg.ranges[i] < msg.range_max) ?
      msg.ranges[i] : msg.range_max);
    point_cloud_[i] = r * cached_rays_[i] + kLaserLoc;
  }
  navigation_->ObservePointCloud(point_cloud_, msg.header.stamp.toSec());
  last_laser_msg_ = msg;
}

void OdometryCallback(const nav_msgs::Odometry& msg) {
  if (FLAGS_v > 0) {
    printf("Odometry t=%f\n", msg.header.stamp.toSec());
  }
  navigation_->UpdateOdometry(msg);
}

void GoToCallbackPS(const geometry_msgs::PoseStamped& msg) {
  const Vector2f loc(msg.pose.position.x, msg.pose.position.y);
  const float angle =
      2.0 * atan2(msg.pose.orientation.z, msg.pose.orientation.w);
  printf("Goal: (%f,%f) %f\u00b0\n", loc.x(), loc.y(), angle);
  navigation_->SetNavGoal(loc, angle);
}

void GoToCallback(const amrl_msgs::Pose2Df& msg) {
  const Vector2f loc(msg.x, msg.y);
  printf("Goal: (%f,%f) %f\u00b0\n", loc.x(), loc.y(), msg.theta);
  navigation_->SetNavGoal(loc, msg.theta);
}


void SignalHandler(int) {
  if (!run_) {
    printf("Force Exit.\n");
    exit(0);
  }
  printf("Exiting.\n");
  run_ = false;
}

void LocalizationCallback(const amrl_msgs::Localization2DMsg& msg) {
  static string map  = "";
  if (FLAGS_v > 0) {
    printf("Localization t=%f\n", GetWallTime());
  }
  navigation_->UpdateLocation(Vector2f(msg.pose.x, msg.pose.y), msg.pose.theta);
  if (map != msg.map) {
    map = msg.map;
    navigation_->UpdateMap(FLAGS_map);
  }
}

void HaltCallback(const std_msgs::Bool& msg) {
  if (navigation_) {
    navigation_->Abort();
  }
}

int main(int argc, char** argv) {
  google::ParseCommandLineFlags(&argc, &argv, false);
  google::InitGoogleLogging(argv[0]);
  signal(SIGINT, SignalHandler);
  // Initialize ROS.
  ros::init(argc, argv, "navigation", ros::init_options::NoSigintHandler);
  ros::NodeHandle n;
  navigation_ = new Navigation(FLAGS_map, &n);

  ros::Subscriber velocity_sub =
      n.subscribe(FLAGS_odom_topic, 1, &OdometryCallback);
  ros::Subscriber localization_sub =
      n.subscribe(FLAGS_loc_topic, 1, &LocalizationCallback);
  ros::Subscriber laser_sub =
      n.subscribe(FLAGS_laser_topic, 1, &LaserCallback);
  ros::Subscriber goto_sub =
      n.subscribe("/move_base_simple/goal", 1, &GoToCallback);
  ros::Subscriber enabler_sub =
      n.subscribe(FLAGS_enable_topic, 1, &EnablerCallback);
  ros::Subscriber halt_sub =
      n.subscribe("halt_robot", 1, &HaltCallback);

  RateLoop loop(1.0 / FLAGS_dt);
  while (run_ && ros::ok()) {
    ros::spinOnce();
    navigation_->Run();
    loop.Sleep();
  }
  delete navigation_;
  return 0;
}
