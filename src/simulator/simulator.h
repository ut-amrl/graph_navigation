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
\file    simulator.h
\brief   C++ Interface: Simulator
\author  Joydeep Biswas, (C) 2011
*/
//========================================================================

#include <iostream>
#include <stdio.h>
#include <random>
#include <string>
#include <vector>

#include "eigen3/Eigen/Dense"
#include "geometry_msgs/Point32.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "nav_msgs/Odometry.h"
#include "ros/ros.h"
#include "ros/package.h"
#include "sensor_msgs/LaserScan.h"
#include "tf/transform_broadcaster.h"
#include "tf/transform_datatypes.h"
#include "visualization_msgs/Marker.h"

#include "amrl_msgs/AckermannCurvatureDriveMsg.h"
#include "amrl_msgs/Localization2DMsg.h"

#include "shared/util/random.h"
#include "shared/util/timer.h"
#include "shared/math/geometry.h"
#include "vector_map/vector_map.h"

#ifndef SIMULATOR_H
#define SIMULATOR_H

class AccelLimits{
  public:
    double max_accel;  // acceleration limit from 0 to max_vel
    double max_deccel; // acceleration limit from max_vel to 0
    double max_vel;    // maximum velocity along dimension

  public:
    void Set(double a,double d,double v) {
      max_accel = a;
      max_deccel = d;
      max_vel = v;
    }

    // Return new limits with all parameters scaled by <f>.
    AccelLimits operator*(double f) const {
      AccelLimits r;
      r.Set(max_accel * f, max_deccel * f, max_vel * f);
      return(r);
    }

    // Scale all parameters by <f> in-place.
    AccelLimits& operator*=(double f);

    // Set limits to <al> with all parameters scaled by <f>.
    AccelLimits& set(const AccelLimits &al,double f);
};

class Simulator{
  // Forward velocity of the robot at the instantaneous base_link frame.
  double robot_vel_;
  // Angular velocity of the robot.
  double robot_ang_vel_;
  // Last time data was published.
  double last_publish_time_ = 0.0;

  ros::Subscriber drive_subscriber_;
  ros::Subscriber init_subscriber_;

  ros::Publisher odometry_publisher_;
  ros::Publisher laser_publisher_;
  ros::Publisher map_publisher_;
  ros::Publisher robot_marker_publisher_;
  ros::Publisher true_pose_publisher_;
  ros::Publisher localization_publisher_;
  tf::TransformBroadcaster *tf_broadcaster_;


  sensor_msgs::LaserScan scan_msg_;
  nav_msgs::Odometry odom_msg_;

  vector_map::VectorMap map_;

  visualization_msgs::Marker line_list_marker_;
  visualization_msgs::Marker robot_pos_marker_;

  // True robot location - will be corrupted by actuation error.
  Eigen::Vector2f true_robot_loc_;
  float true_robot_angle_;

  double t_last_cmd_ = 0.0;

  geometry_msgs::PoseStamped truePoseMsg;

  amrl_msgs::AckermannCurvatureDriveMsg last_cmd_;

  amrl_msgs::Localization2DMsg localization_msg_;
  std::string map_name_;

  util_random::Random random_;

  // Odometry-reported robot location - will be according to commands, but
  // starting in arbitrary odometry frame.
  Eigen::Vector2f odom_loc_;
  float odom_angle_;
  bool step_mode_ = false;
  double sim_time_ = 0.0;

private:
  void InitVizMarker(visualization_msgs::Marker& vizMarker,
                     std::string ns,
                     int id,
                     std::string type,
                     geometry_msgs::PoseStamped p,
                     geometry_msgs::Point32 scale,
                     double duration,
                     std::vector<float> color);
  void InitSimulatorVizMarkers();
  void DrawMap();
  void InitalLocationCallback(const amrl_msgs::Localization2DMsg& msg);
  void DriveCallback(const amrl_msgs::AckermannCurvatureDriveMsg& msg);
  void PublishOdometry();
  void PublishLaser();
  void PublishVisualizationMarkers();
  void PublishTransform();
  void Update();

public:
  Simulator();
  ~Simulator();
  void Init(ros::NodeHandle &n);
  void ResetState();
  void Run();
  void RunIteration();
  void SetStepMode(bool step_mode);
  double GetStepInterval() const;

  void Step(const amrl_msgs::AckermannCurvatureDriveMsg& cmd,
            nav_msgs::Odometry* odom_msg,
            sensor_msgs::LaserScan* scan_msg,
            amrl_msgs::Localization2DMsg* localization_msg);
};
#endif //SIMULATOR_H
