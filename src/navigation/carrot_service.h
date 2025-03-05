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

#ifdef ROS1
  #include <ros/ros.h>
  #include <amrl_msgs/CarrotPlannerSrv.h>  
  #include <geometry_msgs/PoseStamped.h>
  #include <nav_msgs/Path.h>
#else
  #include <rclcpp/rclcpp.hpp>
  #include <amrl_msgs/srv/carrot_planner_srv.hpp>
  #include <geometry_msgs/msg/pose_stamped.hpp>
  #include <nav_msgs/msg/path.hpp>
#endif

#include <condition_variable>
#include <mutex>
#include <vector>

#include "carrot_base.h"    // Your parent class
#include "navigation_types.h"  // For Odom, etc.

namespace carrot_ros {
#ifdef ROS1
  using PathMsg = nav_msgs::Path;
#else
  using PathMsg = nav_msgs::msg::Path;
#endif
}

class CarrotService : public CarrotBase {
 public:
  // Constructor
  #ifdef ROS1
    explicit CarrotService(const std::string& service_name);
    ros::NodeHandle nh_;
    ros::ServiceClient service_client_;
  #else
    // For ROS2: accept an existing Node or create your own. Here we assume user passes a Node ptr.
    CarrotService(const std::string& service_name);
    rclcpp::Node::SharedPtr node_;
    rclcpp::Client<amrl_msgs::srv::CarrotPlannerSrv>::SharedPtr service_client_;
  #endif

  // Overridden plan function
  navigation::CarrotPlan GetCarrot(const Eigen::Vector2f& local_carrot,
                                   const navigation::Odom& odom) override;

 private:
  // Thread-safe shared state
  navigation::CarrotPlan latest_carrot_plan_;
  navigation::Odom latest_odom_;
  bool has_carrot_ = false;
  bool service_request_ongoing_ = false;
  std::mutex mutex_;
  std::condition_variable cv_;

  // Helper for the actual service call (request/response).
  bool callCarrotPlannerService(
    #ifdef ROS1
        amrl_msgs::CarrotPlannerSrv &srv
    #else
        amrl_msgs::srv::CarrotPlannerSrv::Request &req,
        amrl_msgs::srv::CarrotPlannerSrv::Response &res
    #endif
  );

  void processCarrotPath(const carrot_ros::PathMsg& path,
    const navigation::Odom& odom);

  // Helper functions
  void TransformCarrot(const navigation::Odom& odom, navigation::CarrotPlan& carrot_plan);
  void RequestCarrotUpdate(const Eigen::Vector2f& local_waypoint,
                           const navigation::Odom& odom);
};

#endif  // API_CARROT_H
