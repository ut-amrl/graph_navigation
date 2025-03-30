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
\author  Joydeep Biswas, Jarrett Holtz, Kavan Sikand (C) 2021
*/
//========================================================================

#include <string>

#include <ctime>
#include <deque>
#include <memory>
#include <mutex>
#include <set>
#include <unordered_set>
#include <vector>

// External Libraries 
#include "eigen3/Eigen/Dense"

// AMRL Specific Libraries
#include "carrot_service.h"
#include "config_reader/config_reader.h"
#include "graph_domain.h"
#include "motion_primitives.h"
#include "navigation_parameters.h"
#include "navigation_types.h"
#include "osm_planner.h"
#include "shared/math/gps_util.h"
#include "ros_adapter.h"
#include "state_machine.h"

// #include "amrl_msgs/AckermannCurvatureDriveMsg.h"
// #include "amrl_msgs/GPSMsg.h"
// #include "amrl_msgs/Localization2DMsg.h"
// #include "amrl_msgs/MissionStatusMsg.h"
// #include "amrl_msgs/VisualizationMsg.h"

// #include "eight_connected_domain.h"
// #include "visualization/visualization.h"
// #include "visualization_msgs/Marker.h"
// #include "visualization_msgs/MarkerArray.h"

#ifndef NAVIGATION_H
#define NAVIGATION_H

namespace navigation {

// Forward declaration for the ros adapter
class RosAdapter;


class Navigation {
 public:
  explicit Navigation();
  void ConvertPathToNavMsgsPath();
  void UpdateMap(const std::string& map_file);
  void UpdateLocation(const Eigen::Vector2f& loc, float angle);
  void UpdateOdometry(const Odom& msg);
  void UpdateCommandHistory(Twist twist);
  void ObservePointCloud(const std::vector<Eigen::Vector2f>& cloud,
                         double time);
  void ObserveImage(cv::Mat image, double time);
  bool Run(const double& time, Eigen::Vector2f& cmd_vel, float& cmd_angle_vel);
  void GetStraightFreePathLength(float* free_path_length, float* clearance);
  void GetFreePathLength(float curvature, float* free_path_length,
                         float* clearance, Eigen::Vector2f* obstruction);
  bool IsGoalInFOV(const Eigen::Vector2f& local_goal);
  bool IsGoalAvailable();
  bool IsGoalReached();
  void UpdateGPS(const GPSPoint& msg);
  void SetGPSNavGoals(const vector<GPSPoint>& goals);
  void SetNavGoal(const Eigen::Vector2f& loc, float angle);
  void ResetNavGoals();
  void SetOverride(const Eigen::Vector2f& loc, float angle);
  void Resume();
  bool PlanStillValid();
  bool IntermediatePlanStillValid();

  // GPS Odom Transform helpers
  void GetCompensatedOdomUTMTransform(Eigen::Affine2f& T_tp_utm,
                                      Eigen::Affine2f& T_tp_odom);
  Eigen::Affine2f OdometryToUTMTransform(const Odom& odom,
                                         const GPSPoint& gps_loc);
  int GetNextGPSGlobalGoal(int start_goal_index);
  bool GetNextGPSGoal(gps_util::GPSPoint& goal);

  void Plan(Eigen::Vector2f goal_loc);
  void PlanIntermediate(const Eigen::Vector2f& initial,
                        const Eigen::Vector2f& end);
  std::vector<GraphDomain::State> Plan(const Eigen::Vector2f& initial,
                                       const Eigen::Vector2f& end);
  std::vector<int> GlobalPlan(const Eigen::Vector2f& initial,
                              const Eigen::Vector2f& end);
  std::vector<GPSPoint> GlobalPlan(const GPSPoint& inital,
                                   const std::vector<GPSPoint>& goals);
  bool GetGlobalPlan(std::vector<GPSPoint>& plan) const;
  std::vector<GraphDomain::State> GetPlanPath();
  std::vector<GraphDomain::State> GetGlobalPath();

  // Get the next best global gps goal
  void GetGlobalGoal();

  Eigen::Vector2f GetPathGoal(float target_distance);
  bool GetGlobalCarrot(Eigen::Vector2f& carrot);
  bool GetLocalCarrot(Eigen::Vector2f& carrot);
  bool GetLocalCarrotHeading(Eigen::Vector2f& carrot, bool global);
  bool GetCarrot(Eigen::Vector2f& carrot, bool global, float carrot_dist);
  // Enable or disable autonomy.
  void Enable(bool enable);
  // Indicates whether autonomy is enabled or not.
  bool Enabled() const;
  // Stop all navigation functions.
  void Pause();
  // Set parameters for navigation.
  void Initialize(const NavigationParameters& params, 
    std::shared_ptr<RosAdapter> ros_adapter,
    const string& maps_dir,
    const string& map);
  void InitializeOSM(const OSMPlannerParameters& params);
  // Map obstacles into global costmap
  void LoadVectorMap(const std::string& map_file);

  // Allow client programs to configure navigation parameters
  void SetMaxVel(const float vel);
  void SetMaxAccel(const float accel);
  void SetMaxDecel(const float decel);
  void SetAngAccel(const float accel);
  void SetAngVel(const float vel);
  void SetObstacleMargin(const float margin);
  void SetCarrotDist(const float dist);
  void SetClearanceWeight(const float weight);

  // Getter
  Eigen::Vector2f GetTarget();
  Eigen::Vector2f GetOverrideTarget();
  Eigen::Vector2f GetVelocity();
  float GetAngularVelocity();
  MissionStatus GetMissionStatus();
  std::string GetNavStatus();
  uint8_t GetNavStatusUint8();
  std::vector<Eigen::Vector2f> GetPredictedCloud();
  float GetCarrotDist();
  bool GetCarrotPlan(CarrotPlan &plan);
  float GetObstacleMargin();
  bool GetRobotPose(Eigen::Vector3f& pose);
  float GetRobotWidth();
  float GetRobotLength();
  bool GetVisualizationImage(cv::Mat& image, cv::Mat& bev_image);
  std::vector<std::shared_ptr<motion_primitives::PathRolloutBase>>
  GetLastPathOptions();
  std::shared_ptr<motion_primitives::PathRolloutBase> GetOption();
  bool GetInitialOdom(Odom& odom) const;
  bool GetInitialGPS(GPSPoint& loc) const;

  Eigen::Vector2f GetIntermediateGoal();
  // Get the next best global gps goal
  void ReplanAndSetNextNavGoal(bool replan);

  // Converts a route of GPS points to a route of map points
  std::vector<Vector2d> GPSRouteToMap(const std::vector<GPSPoint>& route);

  protected:
    std::weak_ptr<RosAdapter> ros_adapter_;

 private:
  // Test 1D TOC motion in a straight line.
  void TrapezoidTest(Eigen::Vector2f& cmd_vel, float& cmd_angle_vel);
  // Test driving straight up to the next obstacle.
  void ObstacleTest(Eigen::Vector2f& cmd_vel, float& cmd_angle_vel);
  // Test obstacle avoidance.
  void ObstAvTest(Eigen::Vector2f& cmd_vel, float& cmd_angle_vel);
  // Test OSM Planner.
  void OSMPlannerTest();
  // Test planner.
  void PlannerTest();
  // Test GPS Planner.
  void GPSPlannerTest(Eigen::Vector2f& cmd_vel, float& cmd_angle_vel);
  // Run obstacle avoidance local planner.
  void RunObstacleAvoidance(Eigen::Vector2f& cmd_vel, float& cmd_angle_vel);
  // Latency testing routine.
  void LatencyTest(Eigen::Vector2f& cmd_vel, float& cmd_angle_vel);
  // Removes odometry messages older than the latest GPS update. (accounting for
  // latency)
  void PruneOdometryQueue();
  // Remove commands older than latest real robot updates (odometry and LIDAR),
  // accounting for latency.
  void
  PruneLatencyQueue();  // Perform latency compensation by forward-predicting
                        // the commands within the latency interval.
  void ForwardPredict(double t);
  // Run 1D TOC.
  float Run1DTOC(float x_now, float x_target, float v_now, float max_speed,
                 float a_max, float d_max, float dt) const;
  // Come to a halt.
  void Halt(Eigen::Vector2f& cmd_vel, float& cmd_angle_vel);
  // Turn around in-place to face the next waypoint.
  void TurnInPlace(Eigen::Vector2f& cmd_vel, float& cmd_angle_vel);
  // Rule out path options that would violate dynamic constraints.
  void ApplyDynamicConstraints(std::vector<PathOption>* options);
  // Get available path options for next time-step.
  void GetPathOptions(std::vector<PathOption>* options);
  // Draw the robot's outline for visualization.
  void DrawRobot();
  // Publish a status message
  void PublishNavStatus(const Eigen::Vector2f& carrot);

  // 03/13/2025: Updated along with state machine
  void HandleRun(const double& time, Eigen::Vector2f& cmd_vel,
    float& cmd_angle_vel);
  
  void HandleTurnInPlace(const double& time, Eigen::Vector2f& cmd_vel,
    float& cmd_angle_vel);

  void HandleRecovery(const double& time, Eigen::Vector2f& cmd_vel,
    float& cmd_angle_vel);

  void SetStateMachineConditions();

  bool UpdateLocalTarget();

  // Current map frame robot location (OdometryCallback).
  Eigen::Vector2f robot_loc_;
  // Current map frame robot orientation (OdometryCallback).
  float robot_angle_;
  // Current robot velocity.
  Eigen::Vector2f robot_vel_;
  // Current robot angular speed.
  float robot_omega_;
  // Current odometry frame robot location (OdometryCallback).
  Eigen::Vector2f odom_loc_;
  // Current odometry frame robot angle (OdometryCallback).
  float odom_angle_;
  // Newest odometry message received.
  Odom initial_odom_msg_;
  Odom latest_odom_msg_;
  std::deque<Odom> odom_history_;
  // Odom latest_odom_msg_;
  // Newest image received.
  cv::Mat latest_image_;
  double t_image_;
  // GPS Related Variables
  OSMPlannerParameters osm_params_;
  OSMPlanner osm_planner_;
  GPSPoint robot_gps_loc_;
  GPSPoint initial_gps_loc_;
  bool gps_initialized_;
  int gps_goal_index_;
  std::vector<GPSPoint> gps_nav_goals_loc_;
  GPSTranslator gps_translator_;
  MissionStatus mission_status_;

  // Local carrot planner
  std::unique_ptr<CarrotBase> carrot_planner_;
  CarrotPlan latest_carrot_plan_;

  // state machine
  NavigationState nav_state_; // deprecated to be removed later
  StateMachine state_machine_;

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
  // Point cloud from last laser scan observed, forward predicted for latency
  // compensation.
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
  // Previously computed global navigation plan.
  std::vector<GraphDomain::State> global_plan_path_;

  // Local navigation target for obstacle avoidance planner, in the robot's
  // reference frame.
  Eigen::Vector2f local_target_;

  // Global frame, set by an override message
  Eigen::Vector2f override_target_;

  // // Message for status publishing

  // History of commands sent, to perform latency compensation.
  std::deque<Twist> command_history_;

  // Whether to enable autonomous navigation or not.
  bool enabled_;
  // Navigation parameters.
  NavigationParameters params_;

  // Whether or not things have been initialized.
  bool initialized_;

  // Path sampler.
  std::unique_ptr<motion_primitives::PathRolloutSamplerBase> sampler_;

  // Path evaluator.
  std::unique_ptr<motion_primitives::PathEvaluatorBase> evaluator_;

  // Last Set of path options sampled
  std::vector<std::shared_ptr<motion_primitives::PathRolloutBase>>
      last_options_;
  // Last PathOption taken
  std::shared_ptr<motion_primitives::PathRolloutBase> best_option_;

  // Location of robot at last cost map generation
  Eigen::Vector2f prev_robot_loc_;
  //
  bool intermediate_path_found_;

  Eigen::Vector2f intermediate_goal_;

};

}  // namespace navigation

#endif  // NAVIGATION_H
