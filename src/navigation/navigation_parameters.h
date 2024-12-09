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
\file    navigation_parameters.h
\brief   Interface for Navigation parameters as loaded from config.
\author  Kavan Sikand, (C) 2020
*/
//========================================================================

#include <vector>
#include "opencv2/opencv.hpp"

#ifndef NAVIGATION_PARAMETERS_H
#define NAVIGATION_PARAMETERS_H

namespace navigation {

struct MotionLimits {
  // Maximum permissible acceleration magnitude.
  // NOTE: Must be positive!
  float max_acceleration;
  // Maximum permissible deceleration magnitude.
  // NOTE: Must be positive!
  float max_deceleration;
  // Maximum permissible speed.
  // NOTE: Must be positive!
  float max_speed;

  MotionLimits() :
      max_acceleration(0),
      max_deceleration(0),
      max_speed(0) {}

  MotionLimits(float max_acceleration,
               float max_deceleration,
               float max_speed) :
      max_acceleration(max_acceleration),
      max_deceleration(max_deceleration),
      max_speed(max_speed) {}
};

struct NavigationParameters {
  // whether to use intermediate planning or not
  bool do_intermed;
  // Control period in seconds.
  double dt;
  // Motion limits for linear motion.
  MotionLimits linear_limits;
  // Motion limits for angular motion.
  MotionLimits angular_limits;
  // Distance along global plan to find intermediate goal
  float intermediate_goal_dist;
  // System latency in seconds, including sensing latency, processing latency,
  // and actuation latency.
  float system_latency;
  // Safety obstacle margin around the robot.
  float obstacle_margin;
  // Number of options to consider for the local planner.
  unsigned int num_options;
  // Width of the robot.
  float robot_width;
  // Length of the robot.
  float robot_length;
  // Location of the base link w.r.t. the center of the robot.
  // Negative values indicate that the base link is closer to the rear of the
  // robot, for example on a car with ackermann steering, with its base link
  // coincident with its rear axle.
  float base_link_offset;
  float max_free_path_length;
  float max_clearance;
  float local_fov;

  bool can_traverse_stairs;
  bool use_map_speed;

  // Distance tolerance to reaching target.
  float target_dist_tolerance;
  // Velocity tolerance to reaching target.
  float target_vel_tolerance;
  // angle tolerance to reaching target
  float target_angle_tolerance;

  bool use_kinect;

  std::string evaluator_type;

  std::string model_path;

  // Distance of carrot along intermediate path to compute local planner goal
  float carrot_dist;
  // Robot buffer/inflation size
  float max_inflation_radius;
  // Distance between grid locations in costmap
  float local_costmap_resolution;
  // Costmap size in each direction
  float local_costmap_size;
  // Minimum inflation distance to change intermediate plan
  float min_inflation_radius;

  // Same as local costmap parameters but for global costmap
  float global_costmap_resolution;
  float global_costmap_size_x;
  float global_costmap_size_y;
  float global_costmap_origin_x;
  float global_costmap_origin_y;

  // Lidar scan min range
  float lidar_range_min;
  // Lidar scan max range
  float lidar_range_max;

  // Distance between intermediate goal and global carrot before replanning
  float replan_dist;

  // How long an object should stay in the costmap if not continuously observed
  float object_lifespan;
  
  // Coefficient for exponential inflation cost
  float inflation_coeff;
  // Weight for distance cost vs. inflation cost
  float distance_weight;
  // Distance of carrot when using turn in place recovery
  float recovery_carrot_dist;


  cv::Mat K;
  cv::Mat D;
  cv::Mat H;

  // Default constructor, just set defaults.
  NavigationParameters() :
      dt(0.025),
      linear_limits(0.5, 0.5, 0.5),
      angular_limits(0.5, 0.5, 1.0),
      intermediate_goal_dist(5),
      system_latency(0.24),
      obstacle_margin(0.15),
      num_options(41),
      robot_width(0.44),
      robot_length(0.5),
      base_link_offset(0),
      max_free_path_length(10.0),
      max_clearance(1.0),
      can_traverse_stairs(false),
      use_map_speed(true),
      target_dist_tolerance(0.1),
      target_vel_tolerance(0.1),
      target_angle_tolerance(0.05),
      use_kinect(true),
      evaluator_type("cost_map"),
      carrot_dist(2),
      max_inflation_radius(1),
      local_costmap_resolution(0.1),
      local_costmap_size(20),
      min_inflation_radius(0.3),
      global_costmap_resolution(0.1),
      global_costmap_size_x(100),
      global_costmap_size_y(100),
      global_costmap_origin_x(-50),
      global_costmap_origin_y(-50),
      lidar_range_min(0.1),
      lidar_range_max(10),
      replan_dist(2),
      object_lifespan(5),
      inflation_coeff(5),
      distance_weight(2),
      recovery_carrot_dist(0.5){
      }
};
}  // namespace navigation

#endif  // NAVIGATION_PARAMETERS_H
