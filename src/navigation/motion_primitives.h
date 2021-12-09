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
\file    motion_primitives.h
\brief   Generic interfaces for motion primitives of different platforms.
\author  Joydeep Biswas, (C) 2021
*/
//========================================================================

#include <memory>
#include <vector>

#include "eigen3/Eigen/Dense"
#include "opencv2/core/mat.hpp"

#include "math/line2d.h"
#include "math/poses_2d.h"
#include "navigation_parameters.h"

#ifndef MOTION_PRIMITIVES_H
#define MOTION_PRIMITIVES_H

namespace motion_primitives {

// A path rollout, with the start being the robot's current pose in the robot
// frame - 0,0,0.
struct PathRolloutBase {
  // Length of the path rollout -- this is the cumulative distance traverdsed
  // along the path, $\int ||v(t)||dt$ where $v(t)$ is the instantaneous
  // velocity. 
  virtual float Length() const = 0;

  // Free path length of the rollout -- this is the cumulative free space distance along the direction of the rollout
  // along the path, $\int ||v(t)||dt$ where $v(t)$ is the instantaneous
  // velocity. 
  virtual float FPL() const = 0;

  // Angular Length of the path rollout -- this is cumulative angular distance
  // (not displacement) traversed: $\int ||\dot{\theta}(t)||dt$ where
  // $\dot{\theta}(t)$ is the instantaneous angular velocity.
  virtual float AngularLength() const = 0;

  // The pose of the robot at the end of the path rollout.
  virtual pose_2d::Pose2Df EndPoint() const = 0;

  // Return the pose the robot would be at for fraction f into the path rollout. f \in [0, 1]
  virtual pose_2d::Pose2Df GetIntermediateState(float f) const = 0;

  // The obstacle clearance along the path.
  virtual float Clearance() const = 0;

  // Get actuation commands for the robot to execute this rollout in terms of
  // the robot's linear and angular velocity commands for the specified control
  // period.
  virtual void GetControls(const navigation::MotionLimits& linear_limits,
                           const navigation::MotionLimits& angular_limits,
                           const float dt,
                           const Eigen::Vector2f& linear_vel,
                           const float angular_vel,
                           Eigen::Vector2f& vel_cmd,
                           float& ang_vel_cmd) const = 0;
};

// Path rollout sampler. 
struct PathRolloutSamplerBase {
  // Given the robot's current dynamic state and an obstacle point cloud, return
  // a set of n valid path rollout options that are collision-free.
  virtual std::vector<std::shared_ptr<PathRolloutBase>> GetSamples(int n) = 0;

  // Update the local navigation state, including current velocity, local
  // navigation target, obstacle point cloud, and any other factors relevant for
  // local navigation planning.
  virtual void Update(const Eigen::Vector2f& new_vel, 
                      const float new_ang_vel, 
                      const Eigen::Vector2f& new_local_target,
                      const std::vector<Eigen::Vector2f>& new_point_cloud,
                      const cv::Mat& new_image) {
    vel = new_vel;
    ang_vel = new_ang_vel;
    local_target = new_local_target;
    point_cloud = new_point_cloud;
    image = new_image;
  }

  void SetNavParams(const navigation::NavigationParameters& new_params) {
    nav_params = new_params;
  }

  // Current linear velocity.
  Eigen::Vector2f vel;
  // Current angular velocity.
  float ang_vel;
  // Local navigation target.
  Eigen::Vector2f local_target;
  // Obstacle point cloud.
  std::vector<Eigen::Vector2f> point_cloud;
  // Navigation parameters.
  navigation::NavigationParameters nav_params;
  // Latest image observation.
  cv::Mat image;
};

// Evaluator of path rollout options.
struct PathEvaluatorBase {
  // Default constructor: initialize all values to 0.
  PathEvaluatorBase() : vel(0, 0), ang_vel(0), local_target(0, 0) {}

  // Update the local navigation state, including current velocity, local
  // navigation target, obstacle point cloud, and any other factors relevant for
  // local navigation planning.
  virtual void Update(const Eigen::Vector2f& new_loc, 
                      const float new_ang,
                      const Eigen::Vector2f& new_vel, 
                      const float new_ang_vel, 
                      const Eigen::Vector2f& new_local_target,
                      const std::vector<Eigen::Vector2f>& new_point_cloud,
                      const cv::Mat& new_image) {
    curr_loc = new_loc;
    curr_ang = new_ang;
    vel = new_vel;
    ang_vel = new_ang_vel;
    local_target = new_local_target;
    point_cloud = new_point_cloud;
    image = new_image;
  }

  // Return the best path rollout from the provided set of paths.
  virtual std::shared_ptr<PathRolloutBase> FindBest(
      const std::vector<std::shared_ptr<PathRolloutBase>>& paths) = 0;

  // Current location
  Eigen::Vector2f curr_loc;
  // Current angle
  float curr_ang;
  // Current linear velocity.
  Eigen::Vector2f vel;
  // Current angular velocity.
  float ang_vel;
  // Local navigation target.
  Eigen::Vector2f local_target;
  // Obstacle point cloud.
  std::vector<Eigen::Vector2f> point_cloud;
  // Latest image observation.
  cv::Mat image;
};

float Run1DTimeOptimalControl(const navigation::MotionLimits& limits,
                              const float x_init, 
                              const float v_init, 
                              const float x_final, 
                              const float v_final,
                              const float dt);

// Compute the clearance along the line l with respect to the points.
float StraightLineClearance(const geometry::Line2f& l, 
                            const std::vector<Eigen::Vector2f>& points);

}  // namespace motion_primitives


#endif  // MOTION_PRIMITIVES_H