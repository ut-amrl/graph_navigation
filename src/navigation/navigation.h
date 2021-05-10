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

#include "config_reader/config_reader.h"
#include "eight_connected_domain.h"
#include "graph_domain.h"
#include "navigation_parameters.h"
#include "introspection.h"
#include "graph_navigation/IntrospectivePerceptionInfo.h"
#include "graph_navigation/IntrospectivePerceptionRawInfo.h"

#ifndef NAVIGATION_H
#define NAVIGATION_H

namespace ros {
  class NodeHandle;
}  // namespace ros

namespace navigation {

inline std::string GetMapPath(const std::string& dir, const std::string& name) {
  return dir + "/" + name + "/" + name + ".navigation.json";
}

inline std::string GetDeprecatedMapPath(const std::string& dir, const std::string& name) {
  return dir + "/" + name + "/" + name + ".navigation.txt";
}

inline std::string GetFailureLogsPath(const std::string& dir,
                                      const std::string& name) {
  return dir + "/" + name + "/" + name + ".failure_logs.json";
}

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

  explicit Navigation();
  void ConvertPathToNavMsgsPath();
  void UpdateMap(const std::string& map_dir, const std::string& map_name);
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
  // Set parameters for navigation.
  void Initialize(const NavigationParameters& params,
                  const std::string& map_file,
                  const std::string& failure_logs_path,
                  ros::NodeHandle* ros_node_handle);
  // Provides information about the closes static edge 
  // in the map to the current pose of the robot
  bool GetClosestStaticEdgeInfo(uint64_t* s0_id, uint64_t* s1_id);
  // Add a new instance of predicted navigation failure
  void AddFailureInstance(
      const graph_navigation::IntrospectivePerceptionRawInfo& msg);
  // Save the set of detected locations of navigation failures to file 
  // (competence-aware mode)
  void SaveFailureInstancesToFile(const std::string& output_path);
  // Load the set of previously detected locations of navigation failures to 
  // file (competence-aware mode)
  void LoadFailureInstancesFromFile(const std::string& file);

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
  // Queries the MDP solver for a plan that takes into account 
  // the probability of navigation failure on each edge. This is
  // used in competence_aware operation mode.
  template <class State, class Visualizer>
  bool QueryMDPSolver(uint64_t start_id,
                      uint64_t goal_id,
                      Visualizer* const viz,
                      std::vector<State>* path);
  // Given the available database of predicted failure locations, get the
  // updated probability of failure for all dynamic edges. This information is
  // to be sent to the MDP solver before requesting a replan
  bool GetDynamicEdgesFailureProb(
      std::vector<graph_navigation::IntrospectivePerceptionInfo>*
          edges_failure_prob);
  // Given the available database of logs of previous successful and 
  // unsuccessful traversals of the static edges, estimates the prob. 
  // of navigation failure for all static edges.
  bool GetStaticEdgesFailureProb(
      std::vector<graph_navigation::IntrospectivePerceptionInfo>*
          edges_failure_prob);

  // Updates the correspondences between logged sources of failure and the 
  // dynamic edges of the navigation graph. It then publishes the updated
  // info on a ROS topic
  void PublishDynamicEdgesFailureInfo();
  // Uses the logged frequency of failures for each static edge to 
  // compute the probability of navigation failure for that edge.
  // For all dynamic edges, it computes an estimate of navigation
  // failure based on the proximity of the individual instances of
  // logged failures. It then publishes all the estimated values on a ROS topic
  void PublishAllEdgesFailureInfo();
  // Updates the state of the robot along the planned path and records 
  // instances of successful navigation
  void UpdatePlanProgress(int plan_progress_idx);


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
  // Whether navigation is aborted.
  bool nav_aborted_;
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

  const std::string maps_dir_;

  // Planning domain for A* planner.
  GraphDomain planning_domain_;

  // Previously computed navigation plan.
  std::vector<GraphDomain::State> plan_path_;

  // Index of the current edge on the plan_path_
  int curr_plan_progress_ = 0;

  // Local navigation target for obstacle avoidance planner, in the robot's
  // reference frame.
  Eigen::Vector2f local_target_;

  // Message for status publishing
  actionlib_msgs::GoalStatus status_msg_;

  // History of commands sent, to perform latency compensation.
  std::deque<geometry_msgs::TwistStamped> command_history_;

  // TODO(srabiee): Prune failure data using a kd-tree structure
  // Logged instances of predicted navigation failure
  std::deque<introspection::FailureData> failure_data_;

  // Whether or not a replan has been requested due to a potential change
  // in the cost of the edges.
  bool replan_requested_ = false;

  // Whether to enable autonomous navigation or not.
  bool enabled_;

  // Navigation parameters.
  NavigationParameters params_;

  // Whether or not things have been initialized.
  bool initialized_;
};

}  // namespace navigation

#endif  // NAVIGATION_H
