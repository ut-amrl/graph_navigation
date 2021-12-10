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

#include <deque>
#include <memory>
#include <vector>

#include "eigen3/Eigen/Dense"

#include "config_reader/config_reader.h"
#include "eight_connected_domain.h"
#include "graph_domain.h"
#include "navigation_parameters.h"
#include "motion_primitives.h"

#ifndef NAVIGATION_H
#define NAVIGATION_H

namespace navigation {

inline std::string GetMapPath(const std::string& dir, const std::string& name) {
  return dir + "/" + name + "/" + name + ".navigation.json";
}

inline std::string GetDeprecatedMapPath(const std::string& dir, const std::string& name) {
  return dir + "/" + name + "/" + name + ".navigation.txt";
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

struct Twist {
  double time;
  Eigen::Vector3f linear;
  Eigen::Vector3f angular;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
};

struct Odom {
  double time;
  Eigen::Vector3f position;
  Eigen::Quaternionf orientation;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
};

enum class NavigationState {
  kStopped = 0,
  kPaused = 1,
  kGoto = 2,
  kTurnInPlace = 3,
  kOverride = 4
};

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
  void GetStraightFreePathLength(float* free_path_length,
                                 float* clearance);
  void GetFreePathLength(float curvature,
                         float* free_path_length,
                         float* clearance,
                         Eigen::Vector2f* obstruction);
  void SetNavGoal(const Eigen::Vector2f& loc, float angle);
  void SetOverride(const Eigen::Vector2f& loc, float angle);
  void Resume();
  bool PlanStillValid();
  void Plan(Eigen::Vector2f goal_loc);
  std::vector<GraphDomain::State> Plan(const Eigen::Vector2f& initial,
                                       const Eigen::Vector2f& end);
  std::vector<int> GlobalPlan(const Eigen::Vector2f& initial,
                              const Eigen::Vector2f& end);
  std::vector<GraphDomain::State> GetPlanPath();
  bool GetCarrot(Eigen::Vector2f& carrot);
  // Enable or disable autonomy.
  void Enable(bool enable);
  // Indicates whether autonomy is enabled or not.
  bool Enabled() const;
  // Stop all navigation functions.
  void Pause();
  // Set parameters for navigation.
  void Initialize(const NavigationParameters& params,
                  const std::string& map_file);

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
  std::string GetNavStatus();
  std::vector<Eigen::Vector2f> GetPredictedCloud();
  float GetCarrotDist();
  float GetObstacleMargin();
  float GetRobotWidth();
  float GetRobotLength();
  const cv::Mat& GetVisualizationImage();
  std::vector<std::shared_ptr<motion_primitives::PathRolloutBase>> GetLastPathOptions();
  std::shared_ptr<motion_primitives::PathRolloutBase> GetOption();

 private:

  // Test 1D TOC motion in a straight line.
  void TrapezoidTest(Eigen::Vector2f& cmd_vel, float& cmd_angle_vel);
  // Test driving straight up to the next obstacle.
  void ObstacleTest(Eigen::Vector2f& cmd_vel, float& cmd_angle_vel);
  // Test obstacle avoidance.
  void ObstAvTest(Eigen::Vector2f& cmd_vel, float& cmd_angle_vel);
  // Test planner.
  void PlannerTest();
  // Run obstacle avoidance local planner.
  void RunObstacleAvoidance(Eigen::Vector2f& cmd_vel, float& cmd_angle_vel);
  // Latency testing routine.
  void LatencyTest(Eigen::Vector2f& cmd_vel, float& cmd_angle_vel);
  // Remove commands older than latest real robot updates (odometry and LIDAR),
  // accounting for latency.
  void PruneLatencyQueue(); // Perform latency compensation by forward-predicting the commands within the latency interval.
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
  Odom latest_odom_msg_;
  // Newest image received.
  cv::Mat latest_image_;
  double t_image_;


  NavigationState nav_state_;
  
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
};

}  // namespace navigation

#endif  // NAVIGATION_H
