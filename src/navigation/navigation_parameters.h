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

    MotionLimits() : max_acceleration(0), max_deceleration(0), max_speed(0) {}

    MotionLimits(float max_acceleration, float max_deceleration, float max_speed)
        : max_acceleration(max_acceleration), max_deceleration(max_deceleration), max_speed(max_speed) {}
};

struct GeometricCenterOffset {
    // Offset of geometric center c_g from base_link origin, in base_link frame.
    // x: positive = c_g forward of base_link.
    // y: positive = c_g left of base_link.
    float x;
    float y;

    GeometricCenterOffset() : x(0), y(0) {}

    GeometricCenterOffset(float x, float y) : x(x), y(y) {}
};

struct NavigationParameters {
    // Control period in seconds.
    double dt;
    // Motion limits for linear motion.
    MotionLimits linear_limits;
    // Motion limits for angular motion.
    MotionLimits angular_limits;
    // System latency in seconds, including sensing latency, processing latency,
    // and actuation latency.
    float actuation_latency;
    // Safety obstacle margin around the robot.
    float obstacle_margin;
    // Number of options to consider for the local planner.
    unsigned int num_options;
    // Width of the robot.
    float robot_width;
    // Length of the robot.
    float robot_length;
    // Offset of geometric center c_g from base_link origin (in base_link frame).
    GeometricCenterOffset geometric_center_offset;
    float max_rollout_length;  // Max rollout/commanded segment length
    float max_lookahead_fpl;   // Max lookahead for free path length computation
    float clearance_band;
    // Half-angle of the lidar field of view cone (radians).
    // Full FOV cone is ±lidar_fov_half_angle. Used to determine when obstacle avoidance can run safely.
    float lidar_fov_half_angle;

    bool can_traverse_stairs;

    // Distance tolerance to reaching target.
    float target_dist_tolerance;
    // Distance tolerance for near-goal nudge behavior (allows OA even when target not centered).
    float nudge_dist_tolerance;
    // Velocity tolerance to reaching target.
    float target_vel_tolerance;
    // angle tolerance to reaching target
    float target_angle_tolerance;
    // Angular velocity tolerance (rad/s) for state transitions.
    // Robot must have |omega| < target_omega_tolerance to transition states.
    float target_omega_tolerance;

    std::string evaluator_type;

    // Distance of carrot along path to compute local planner goal
    float carrot_dist;

    // Motion primitives mode: "ackermann" or "omni"
    std::string motion_primitives_mode;
    bool do_ang_toc;

    // Maximum permissible deviation from the plan
    float max_plan_deviation;
    // Height of laser sensor above robot base frame (for visualization)
    float laser_height;

    // Stuck meta-controller parameters
    float stuck_meta_override_obstacle_margin;
    float stuck_meta_stuck_timeout_sec;
    float stuck_meta_improve_eps;

    // Command mapping parameters
    bool apply_custom_cmd_map;
    // Linear model for x-axis velocity mapping: v_mapped = slope * v + intercept
    float cmd_map_x_slope_pos;
    float cmd_map_x_intercept_pos;
    float cmd_map_x_slope_neg;
    float cmd_map_x_intercept_neg;
    // Linear model for y-axis velocity mapping
    float cmd_map_y_slope_pos;
    float cmd_map_y_intercept_pos;
    float cmd_map_y_slope_neg;
    float cmd_map_y_intercept_neg;
    // Linear model for angular velocity mapping
    float cmd_map_r_slope_pos;
    float cmd_map_r_intercept_pos;
    float cmd_map_r_slope_neg;
    float cmd_map_r_intercept_neg;

    // Default constructor, just set defaults.
    NavigationParameters()
        : dt(0.025),
          linear_limits(0.5, 0.5, 0.5),
          angular_limits(0.5, 0.5, 1.0),
          actuation_latency(0.24),
          obstacle_margin(0.15),
          num_options(41),
          robot_width(0.44),
          robot_length(0.5),
          geometric_center_offset(0, 0),
          max_rollout_length(10.0),
          max_lookahead_fpl(10.0),
          clearance_band(1.0),
          lidar_fov_half_angle(1.57),
          can_traverse_stairs(false),
          target_dist_tolerance(0.1),
          nudge_dist_tolerance(0.3),
          target_vel_tolerance(0.1),
          target_angle_tolerance(0.05),
          target_omega_tolerance(0.15),
          evaluator_type("linear"),
          carrot_dist(2),
          motion_primitives_mode("ackermann"),
          do_ang_toc(false),
          apply_custom_cmd_map(false),
          cmd_map_x_slope_pos(1.0f),
          cmd_map_x_intercept_pos(0.0f),
          cmd_map_x_slope_neg(1.0f),
          cmd_map_x_intercept_neg(0.0f),
          cmd_map_y_slope_pos(1.0f),
          cmd_map_y_intercept_pos(0.0f),
          cmd_map_y_slope_neg(1.0f),
          cmd_map_y_intercept_neg(0.0f),
          cmd_map_r_slope_pos(1.0f),
          cmd_map_r_intercept_pos(0.0f),
          cmd_map_r_slope_neg(1.0f),
          cmd_map_r_intercept_neg(0.0f) {}
};
}  // namespace navigation

#endif  // NAVIGATION_PARAMETERS_H
