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
\file    pointcloud_to_laserscan_main.cc
\brief   A point cloud to laserscan convert that actually works without
         crashing or throwing buffer overflow errors.
\author  Joydeep Biswas, (C) 2020
*/
//========================================================================

#include <inttypes.h>
#include <math.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <vector>

//#include "config_reader/config_reader.h"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "math/math_util.h"
#include "ros/ros.h"
#include "ros/ros_helpers.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/point_cloud2_iterator.h"
#include "util/timer.h"

using Eigen::Vector2f;
using Eigen::Vector3f;
using math_util::DegToRad;
using math_util::RadToDeg;
using math_util::Sq;
using ros::Time;
using ros_helpers::Eigen2DToRosPoint;
using ros_helpers::Eigen3DToRosPoint;
using ros_helpers::RosPoint;
using ros_helpers::SetRosVector;
using sensor_msgs::LaserScan;
using sensor_msgs::PointCloud2;
using std::string;
using std::vector;

DECLARE_string(helpon);
DECLARE_int32(v);

sensor_msgs::LaserScan laser_msg_;
float max_height_ = FLT_MAX;
float min_height_ = -FLT_MAX;
float min_sq_range_ = 0;
float max_sq_range_ = FLT_MAX;
string laser_topic_;
string pointcloud_topic_;

ros::Publisher scan_publisher_;

void PointcloudCallback(const sensor_msgs::PointCloud2 &msg) {
  static CumulativeFunctionTimer function_timer_(__FUNCTION__);
  CumulativeFunctionTimer::Invocation invoke(&function_timer_);
  if (FLAGS_v > 1) {
    printf("PointCloud2 message, t=%f\n", msg.header.stamp.toSec());
    for (const auto& field: msg.fields) {
      printf(" Field \"%s\":%d\n", field.name.c_str(), field.datatype);
    }
  }
  laser_msg_.header = msg.header;

  for (float &r : laser_msg_.ranges) {
    r = FLT_MAX;
  }
  // Iterate through pointcloud
  for (sensor_msgs::PointCloud2ConstIterator<float> iter_x(msg, "x"),
       iter_y(msg, "y"), iter_z(msg, "z");
       iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
    if (!isfinite(*iter_x) || !isfinite(*iter_y) || !isfinite(*iter_z)) {
      continue;
    }
    const Vector3f p(*iter_x, *iter_y, *iter_z);

    if (p.z() > max_height_ || p.z() < min_height_) {
      continue;
    }

    const float sq_range = Sq(p.x()) + Sq(p.y());
    if (sq_range < min_sq_range_ || sq_range > max_sq_range_) continue;
    const float angle = atan2(p.y(), p.x());

    const size_t idx = static_cast<size_t>(
        floor((angle - laser_msg_.angle_min) / laser_msg_.angle_increment));

    CHECK_LE(idx, laser_msg_.ranges.size()) << "angle: " << angle;
    if (laser_msg_.ranges[idx] > sq_range) {
      laser_msg_.ranges[idx] = sq_range;
    }
  }

  for (float &r : laser_msg_.ranges) {
    if (r < FLT_MAX) {
      r = sqrt(r);
    } else {
      r = 0;
    }
  }
  scan_publisher_.publish(laser_msg_);
}


void LoadConfig() {

  float angle_min;
  float angle_max;
  float increment;
  float range_min;
  float range_max;
  float height_min;
  float height_max;

  ros::param::get("laser_topic", laser_topic_);
  ros::param::get("pointcloud_topic", pointcloud_topic_);

  std::string ns = "LidarParameters";
  ros::param::get(ns + "/angle_min", angle_min);
  ros::param::get(ns + "/angle_max", angle_max);
  ros::param::get(ns + "/increment", increment);
  ros::param::get(ns + "/range_min", range_min);
  ros::param::get(ns + "/range_max", range_max);
  ros::param::get(ns + "/height_min", height_min);
  ros::param::get(ns + "/height_max", height_max);

  const int num_ranges = floor((angle_max - angle_min) / increment);
  laser_msg_.angle_min = angle_min;
  laser_msg_.angle_max = angle_max;
  laser_msg_.angle_increment = increment;
  laser_msg_.range_min = range_min;
  laser_msg_.range_max = range_max;
  laser_msg_.ranges.resize(num_ranges);
  min_height_ = height_min;
  max_height_ = height_max;
  min_sq_range_ = Sq(range_min);
  max_sq_range_ = Sq(range_max);
}

int main(int argc, char **argv) {
  google::ParseCommandLineFlags(&argc, &argv, false);
  google::InitGoogleLogging(argv[0]);
  ros::init(argc, argv, "pointcloud_to_laserscan");
  ros::NodeHandle n;
  LoadConfig();
  ros::Subscriber pointcloud_sub =
      n.subscribe(pointcloud_topic_, 1, &PointcloudCallback);
  scan_publisher_ = n.advertise<sensor_msgs::LaserScan>(laser_topic_, 1);
  ros::spin();
  return 0;
}