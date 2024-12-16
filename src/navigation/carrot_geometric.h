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
\file    carrot_service.h
\brief   Carrot server client interface for local navigation.
\author  Arthur Zhang (C) 2024
*/
//========================================================================
#ifndef API_CARROT_H
#define API_CARROT_H

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>

#include <condition_variable>
#include <mutex>
#include <vector>

#include "amrl_msgs/CarrotPlannerSrv.h"
#include "carrot_base.h"  // Include the parent class header

using navigation::CarrotPlan;
using navigation::Odom;

class CarrotService : public CarrotBase {
 public:
  // Constructor
  explicit CarrotService(const std::string& service_name);

  // Overridden plan function
  CarrotPlan GetCarrot(const Eigen::Vector2f& local_carrot,
                       const Odom& odom) override;

 private:
  ros::NodeHandle nh_;                 // ROS node handle
  ros::ServiceClient service_client_;  // ROS service client

  // Thread-safe shared state
  CarrotPlan latest_carrot_plan_;  // Latest carrot plan
  Odom latest_odom_;               // Latest odometry message
  bool has_carrot_ = false;        // Indicates if a carrot exists
  bool service_request_ongoing_ =
      false;                    // Indicates if a service request is ongoing
  std::mutex mutex_;            // Mutex for thread-safe access
  std::condition_variable cv_;  // Condition variable for blocking

  // Helper functions
  void TransformCarrot(const Odom& odom, CarrotPlan& carrot_plan);
  void RequestCarrotUpdate(const Eigen::Vector2f& local_waypoint,
                           const Odom& odom);
};

#endif  // API_CARROT_H
