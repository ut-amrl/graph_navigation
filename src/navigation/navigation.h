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
\file    navigation.h
\brief   Interface for reference Navigation class.
\author  Joydeep Biswas, (C) 2019
*/
//========================================================================

#include <deque>
#include <vector>

#include "actionlib_msgs/GoalStatus.h"
#include "eigen3/Eigen/Dense"
#include "geometry_msgs/TwistStamped.h"
#include "nav_msgs/Odometry.h"

#include "eight_connected_domain.h"
#include "graph_domain.h"

#ifndef NAVIGATION_H
#define NAVIGATION_H

namespace ros {
  class NodeHandle;
}  // namespace ros

namespace navigation {

struct PathOption {
  float curvature;
  float clearance;
  float free_path_length;
  float clearance_to_goal;
  float dist_to_goal;
  explicit PathOption(float c) : curvature(c) {}
  PathOption() {}
  Eigen::Vector2f obstruction;
  Eigen::Vector2f closest_point;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
};

class Navigation {
 public:
  explicit Navigation(const std::string& map_file, ros::NodeHandle* n);
  void UpdateMap(const std::string& map_file);
  void UpdateLocation(const Eigen::Vector2f& loc, float angle);
  void UpdateOdometry(const nav_msgs::Odometry& msg);
  void ObservePointCloud(const std::vector<Eigen::Vector2f>& cloud,
                         double time);
  void Run();
  void GetStraightFreePathLength(float* free_path_length,
                                 float* clearance);
  void GetFreePathLength(float curvature,
                         float* free_path_length,
                         float* clearance,
                         Eigen::Vector2f* obstruction);
  void SetNavGoal(const Eigen::Vector2f& loc, float angle);
  bool PlanStillValid();
  void Plan();
  Eigen::Vector2f GetCarrot();
  // Enable or disable autonomy.
  void Enable(bool enable);
  // Indicates whether autonomy is enabled or not.
  bool Enabled() const;
  // Stop all navigation functions.
  void Abort();

 private:

  // Test 1D TOC motion in a straight line.
  void TrapezoidTest();
  // Test driving straight up to the next obstacle.
  void ObstacleTest();
  // Test obstacle avoidance.
  void ObstAvTest();
  // Test planner.
  void PlannerTest();
  // Run obstacle avoidance local planner.
  void RunObstacleAvoidance();
  // Latency testing routine.
  void LatencyTest();
  // Send and queue command for latency compensation.
  void SendCommand(float vel, float curvature);
  // Remove commands older than latest real robot updates (odometry and LIDAR),
  // accounting for latency.
  void PruneLatencyQueue();
  // Perform latency compensation by forward-predicting the commands within the
  // latency interval.
  void ForwardPredict(double t);
  // Run 1D TOC.
  float Run1DTOC(float x_now,
                 float x_target,
                 float v_now,
                 float max_speed,
                 float a_max,
                 float d_max,
                 float dt) const;
  // Come to a halt.
  void Halt();
  // Turn around in-place to face the next waypoint.
  void TurnInPlace();
  // Rule out path options that would violate dynamic constraints.
  void ApplyDynamicConstraints(std::vector<PathOption>* options);
  // Get available path options for next time-step.
  void GetPathOptions(std::vector<PathOption>* options);
  // Draw the robot's outline for visualization.
  void DrawRobot();

  // Current robot location.
  Eigen::Vector2f robot_loc_;
  // Current robot orientation.
  float robot_angle_;
  // Current robot velocity.
  Eigen::Vector2f robot_vel_;
  // Current robot angular speed.
  float robot_omega_;
  // Odometry-reported robot location.
  Eigen::Vector2f odom_loc_;
  // Odometry-reported robot angle.
  float odom_angle_;
  // Newest odometry message received.
  nav_msgs::Odometry latest_odom_msg_;

  // Whether navigation is complete.
  bool nav_complete_;
  // Navigation goal location.
  Eigen::Vector2f nav_goal_loc_;
  // Navigation goal angle.
  float nav_goal_angle_;

  // Indicates whether an odometry message has been received.
  bool odom_initialized_;
  bool loc_initialized_;

  // Odometry-reported starting location.
  Eigen::Vector2f starting_loc_;

  // Point cloud from last laser scan observed.
  std::vector<Eigen::Vector2f> point_cloud_;
  // Point cloud from last laser scan observed, forward predicted for latency compensation.
  std::vector<Eigen::Vector2f> fp_point_cloud_;
  // Time stamp of observation of point cloud.
  double t_point_cloud_;
  // Time stamp of latest odometry message.
  double t_odometry_;

  // Width of the robot.
  const float kRobotWidth;
  // Length of the robot.
  const float kRobotLength;
  // Location of the robot's rear wheel axle relative to the center of the body.
  const float kRearAxleOffset;
  // Assumed free space when there are no obstacles detected along a path.
  const float kMaxFreeLength;
  // Maximum clearance for the robot to care about.
  const float kMaxClearance;

  // Planning domain for A* planner.
  GraphDomain planning_domain_;

  // Previously computed navigation plan.
  std::vector<GraphDomain::State> plan_path_;

  // Local navigation target for obstacle avoidance planner, in the robot's
  // reference frame.
  Eigen::Vector2f local_target_;

  // Message for status publishing
  actionlib_msgs::GoalStatus status_msg_;

  // History of commands sent, to perform latency compensation.
  std::deque<geometry_msgs::TwistStamped> command_history_;

  // Whether to enable autonomous navigation or not.
  bool enabled_;
};

}  // namespace navigation

#endif  // NAVIGATION_H
