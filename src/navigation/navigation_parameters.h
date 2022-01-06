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
  // Control period in seconds.
  double dt;
  // Motion limits for linear motion.
  MotionLimits linear_limits;
  // Motion limits for angular motion.
  MotionLimits angular_limits;
  // Distance of carrot from robot to compute local planner goal from
  // global plan.
  float carrot_dist;
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

  cv::Mat K;
  cv::Mat D;
  cv::Mat H;

  // Default constructor, just set defaults.
  NavigationParameters() :
      dt(0.025),
      linear_limits(0.5, 0.5, 0.5),
      angular_limits(0.5, 0.5, 1.0),
      carrot_dist(2.5),
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
      evaluator_type("cost_map") {
      }
};
}  // namespace navigation

#endif  // NAVIGATION_PARAMETERS_H
