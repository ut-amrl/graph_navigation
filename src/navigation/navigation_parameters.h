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

#ifndef NAVIGATION_PARAMETERS_H
#define NAVIGATION_PARAMETERS_H

#include "config_reader/config_reader.h"

namespace navigation {

struct MotionLimit {
  // Maximum permissible acceleration magnitude.
  // NOTE: Must be positive!
  float accel;
  // Maximum permissible deceleration magnitude.
  // NOTE: Must be positive!
  float decel;
  // Maximum permissible speed.
  float speed;

  // Default constructor: set all to zero.
  MotionLimit() : accel(0), decel(0), speed(0) {}

  // Convenience constructor: init values.
  MotionLimit(float accel, float decel, float speed) :
      accel(accel), decel(decel), speed(speed) {}
};

struct NavigationParameters {
  // Control period in seconds.
  double dt;
  // Motion limits for linear motion.
  MotionLimit linear_limits;
  // Motion limits for angular motion.
  MotionLimit angular_limits;
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

  bool can_traverse_stairs;
  bool competence_aware;
  // Whether to use the frequentist method for estimating the probability
  // of navigation failure given previous experiences. This is only effective 
  // in competence-aware mode.
  bool frequentist_mode;
  // Whether to use the Bayesian belief update for estimating the probability 
  // of navigation failure given both previous failure experiences and 
  // prediction of navigation failure by introspective perception. This is only 
  // effective in competence-aware mode.
  bool bayes_mode;
  // Make graph_nav run in airsim mode so that it can exploit control
  // services available by the simulator
  bool airsim_compatible;
  // Whether to load failure logs from file upon initialization. This
  // is only done if in competence_aware mode.
  bool load_failure_logs;
  // Whether to keep adding instances of actual or predicted navigation
  // failures or just keep the latest one. This is only relevant in
  // competence-aware mode (only set to True for implementation
  // of a simple baseline method)
  bool memoryless;
  // If enabled, traversal stats will not be updated during deployment (either 
  // for bayes mode or frequentist mode) and the failure likelihood values
  // that are loaded upon startup will be used for path planning.
  bool lock_traversal_stats;
  // If enabled, traversal stats wll be collected during deployment but will
  // not be used for competence-aware path planning. In this mode, no failure
  // likelihood value will be published to the MDP solver. This mode is to be
  // used for collecting data from repeated traversals of the same navigation
  // edges for obtaining ground truth estimate of the failure likelihood.
  bool only_collect_traversal_stats;

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
      max_free_path_length(6.0),
      max_clearance(1.0),
      can_traverse_stairs(false),
      competence_aware(false),
      frequentist_mode(false),
      bayes_mode(false),
      airsim_compatible(false),
      load_failure_logs(false),
      memoryless(false),
      lock_traversal_stats(false),
      only_collect_traversal_stats(false) {}
};

}  // namespace navigation

#endif  // NAVIGATION_PARAMETERS_H
