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
\file    ackermann_motion_primitives.h
\brief   Motion primitives for Ackermann steering platforms.
\author  Joydeep Biswas, (C) 2021
*/
//========================================================================

#include <math.h>

#include <algorithm>
#include <memory>
#include <vector>

#include "shared/math/poses_2d.h"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "config_reader/config_reader.h"
#include "motion_primitives.h"
#include "constant_curvature_arcs.h"
#include "ackermann_motion_primitives.h"

using Eigen::Vector2f;
using pose_2d::Pose2Df;
using std::max;
using std::min;
using std::shared_ptr;
using std::vector;
using namespace math_util;

CONFIG_FLOAT(max_curvature, "AckermannSampler.max_curvature");
CONFIG_FLOAT(clearance_clip, "AckermannSampler.clearance_path_clip_fraction");

namespace {
// Epsilon value for handling limited numerical precision.
const float kEpsilon = 1e-5;
}  // namespace

namespace motion_primitives {

AckermannSampler::AckermannSampler() {}

void AckermannSampler::SetMaxPathLength(ConstantCurvatureArcPath* path_ptr) {
    ConstantCurvatureArcPath& path = *path_ptr;
    if (fabs(path.curvature) < kEpsilon) {
        path.length = min(nav_params.max_free_path_length, local_target.x());
        path.fpl = path.length;
        return;
    }
    const float turn_radius = 1.0f / path.curvature;
    const float quarter_circle_dist = fabs(turn_radius) * M_PI_2;
    const Vector2f turn_center(0, turn_radius);
    const Vector2f target_radial = local_target - turn_center;
    const Vector2f middle_radial = fabs(turn_radius) * target_radial.normalized();
    const float middle_angle = atan2(fabs(middle_radial.x()), fabs(middle_radial.y()));
    const float dist_closest_to_goal = middle_angle * fabs(turn_radius);
    path.fpl = min<float>({nav_params.max_free_path_length, quarter_circle_dist});
    path.length = min<float>({path.fpl, dist_closest_to_goal});
    const float stopping_dist = Sq(vel.x()) / (2.0 * nav_params.linear_limits.max_deceleration);
    path.length = max(path.length, stopping_dist);
}

vector<shared_ptr<PathRolloutBase>> AckermannSampler::GetSamples(int n) {
    // Generate n path samples by varying curvature values across a range.
    // Each sample represents a constant curvature arc that the robot could follow.
    vector<shared_ptr<PathRolloutBase>> samples;

    // Debug/test mode: return fixed curvature samples
    if (false) {
        samples = {
            shared_ptr<PathRolloutBase>(new ConstantCurvatureArcPath(-0.1)),
            shared_ptr<PathRolloutBase>(new ConstantCurvatureArcPath(0)),
            shared_ptr<PathRolloutBase>(new ConstantCurvatureArcPath(0.1)),
        };
        return samples;
    }

    // Calculate dynamic constraints based on current robot state:
    // - robot_vel_ (vel.x()) determines current speed for curvature limits
    // - robot_omega_ (ang_vel) provides current angular velocity for smooth transitions
    const float max_domega = nav_params.dt * nav_params.angular_limits.max_acceleration;
    const float max_dv = nav_params.dt * nav_params.linear_limits.max_acceleration;
    const float robot_speed = fabs(vel.x());

    // Constrain curvature range based on current velocity and angular velocity
    // to ensure dynamically feasible transitions from current state
    float c_min = -CONFIG_max_curvature;
    float c_max = CONFIG_max_curvature;
    if (robot_speed > max_dv + kEpsilon) {
        c_min = max<float>(c_min, (ang_vel - max_domega) / (robot_speed - max_dv));
        c_max = min<float>(c_max, (ang_vel + max_domega) / (robot_speed - max_dv));
    }
    const float dc = (c_max - c_min) / static_cast<float>(n - 1);

    // Generate samples: currently uses simple uniform sampling over full curvature range
    // (ignoring the dynamically constrained range above - this appears to be a bug)
    if (false) {
        // This would use the dynamically constrained range
        for (float c = c_min; c <= c_max; c += dc) {
            auto sample = new ConstantCurvatureArcPath(c);
            SetMaxPathLength(sample);  // Uses local_target to limit path length
            CheckObstacles(sample);    // Uses fp_point_cloud_ for collision checking
            sample->angular_length = fabs(sample->length * c);
            samples.push_back(shared_ptr<PathRolloutBase>(sample));
        }
    } else {
        // Current implementation: uniform sampling over full curvature range
        const float dc = (2.0f * CONFIG_max_curvature) / static_cast<float>(n - 1);
        for (float c = -CONFIG_max_curvature; c <= CONFIG_max_curvature; c += dc) {
            auto sample = new ConstantCurvatureArcPath(c);
            SetMaxPathLength(sample);  // Uses local_target to limit path length toward goal
            CheckObstacles(sample);    // Uses fp_point_cloud_ for collision detection
            sample->angular_length = fabs(sample->length * c);
            samples.push_back(shared_ptr<PathRolloutBase>(sample));
        }
    }

    return samples;
}

void AckermannSampler::CheckObstacles(ConstantCurvatureArcPath* path_ptr) {
    ConstantCurvatureArcPath& path = *path_ptr;

    // Half-dimensions and offsets.
    const float hl = 0.5f * nav_params.robot_length;
    const float hw = 0.5f * nav_params.robot_width;

    // Margin-augmented footprint bounds in base_link frame.
    const float x_max = nav_params.base_link_offset_x + hl + nav_params.obstacle_margin;
    const float x_min = nav_params.base_link_offset_x - hl - nav_params.obstacle_margin;
    const float y_max = nav_params.base_link_offset_y + hw + nav_params.obstacle_margin;
    const float y_min = nav_params.base_link_offset_y - hw - nav_params.obstacle_margin;

    // Body (no margin) bounds.
    const float x_max_body = nav_params.base_link_offset_x + hl;
    const float x_min_body = nav_params.base_link_offset_x - hl;
    const float y_max_body = nav_params.base_link_offset_y + hw;
    const float y_min_body = nav_params.base_link_offset_y - hw;

    // Half-width (+margin) and without margin (for formulas below).
    const float w = hw + nav_params.obstacle_margin;
    const float w_body = hw;

    // Distance from base_link origin to front face (+margin), for straight-line hit tests.
    const float l = hl + nav_params.obstacle_margin + nav_params.base_link_offset_x;
    const float l_body = hl + nav_params.base_link_offset_x;

    // Straight line case (|curvature| ~ 0)
    if (fabs(path.curvature) < kEpsilon) {
        for (const Vector2f& p : *point_cloud) {
            // Skip points inside robot body (NO margin).
            if (p.x() > x_min_body && p.x() < x_max_body && p.y() > y_min_body && p.y() < y_max_body) {
                continue;
            }
            // Outside swept rect laterally or behind.
            if (p.y() < y_min || p.y() > y_max || p.x() < 0.0f) continue;

            // Obstacle limits free path length.
            path.fpl = std::min(path.fpl, p.x() - x_max);
        }

        // Clearance over the executed segment [0, fpl].
        path.clearance = nav_params.max_clearance;
        for (const Vector2f& p : *point_cloud) {
            // Skip body points (NO margin).
            if (p.x() > x_min_body && p.x() < x_max_body && p.y() > y_min_body && p.y() < y_max_body) {
                continue;
            }
            if (p.x() - x_max > path.fpl || p.x() < 0.0f) continue;

            const float lateral_dist = (p.y() < nav_params.base_link_offset_y) ? (y_min - p.y()) : (p.y() - y_max);
            path.clearance = std::min<float>(path.clearance, std::fabs(lateral_dist));
        }
        path.clearance = std::max(0.0f, path.clearance);
        path.fpl = std::max(0.0f, path.fpl);
        path.length = std::min(path.fpl, path.length);

        // Directional stopping distance (forward-only speed).
        const float v_fwd = std::max(0.0f, vel.x());
        const float stopping_dist = (v_fwd * v_fwd) / (2.0f * nav_params.linear_limits.max_deceleration);
        if (path.fpl < stopping_dist) {
            path.length = 0.0f;
        }
        return;
    }

    // Curved path case.
    const float path_radius = 1.0f / path.curvature;
    const Vector2f c(0, path_radius);
    const float s = (path_radius > 0.0f) ? 1.0f : -1.0f;

    // Front corners (margin-inflated) in base_link frame.
    const Vector2f inner_front_corner(x_max, (s > 0.0f) ? y_max : y_min);
    const Vector2f outer_front_corner(x_max, (s > 0.0f) ? y_min : y_max);

    // Radial bounds wrt the turn center.
    const float r1 = std::max<float>(0.0f, std::fabs(path_radius) - w);  // inner side radius
    const float r1_sq = Sq(r1);
    const float r2_sq = (inner_front_corner - c).squaredNorm();
    const float r3_sq = (outer_front_corner - c).squaredNorm();

    float angle_min = M_PI;
    path.obstruction = Vector2f(-nav_params.max_free_path_length, 0);

    using std::isfinite;
    for (const Vector2f& p : *point_cloud) {
        if (!isfinite(p.x()) || !isfinite(p.y()) || p.x() < 0.0f) continue;

        // Skip points inside robot body (NO margin).
        if (p.x() > x_min_body && p.x() < x_max_body && p.y() > y_min_body && p.y() < y_max_body) {
            continue;
        }

        // If already inside margin-inflated footprint → immediate collision.
        if (p.x() > x_min && p.x() < x_max && p.y() > y_min && p.y() < y_max) {
            path.length = 0.0f;
            path.obstruction = p;
            angle_min = 0.0f;
            break;
        }

        // Radial location wrt center.
        const float r_sq = (p - c).squaredNorm();
        if (r_sq < r1_sq || r_sq > r3_sq) continue;

        const float r = std::sqrt(r_sq);
        const float theta = (path.curvature > 0.0f) ? std::atan2<float>(p.x(), path_radius - p.y())
                                                    : std::atan2<float>(p.x(), p.y() - path_radius);

        float alpha;
        if (r_sq < r2_sq) {
            // Hits side first.
            const float x = std::fabs(path_radius) - w;
            alpha = (x > 0.0f) ? std::acos(x / r) : (static_cast<float>(M_PI_2) + std::acos(-x / r));
        } else {
            // Hits front first.
            alpha = std::asin(std::min(1.0f, std::max(0.0f, l / r)));
        }

        const float path_length = std::max<float>(0.0f, std::fabs(path_radius) * (theta - alpha));
        if (path.length > path_length) {
            path.length = path_length;
            path.obstruction = p;
            angle_min = theta;
        }
    }

    // Directional stopping distance (forward-only).
    {
        const float v_fwd = std::max(0.0f, vel.x());
        const float stopping_dist = (v_fwd * v_fwd) / (2.0f * nav_params.linear_limits.max_deceleration);
        if (path.length < stopping_dist) path.length = 0.0f;
    }

    path.length = std::max(0.0f, path.length);
    angle_min = std::min<float>(angle_min, path.length * std::fabs(path.curvature));
    path.clearance = nav_params.max_clearance;

    for (const Vector2f& p : *point_cloud) {
        const float theta = (path.curvature > 0.0f) ? std::atan2<float>(p.x(), path_radius - p.y())
                                                    : std::atan2<float>(p.x(), p.y() - path_radius);
        if (theta < CONFIG_clearance_clip * angle_min && theta > 0.0f) {
            const float r = (p - c).norm();
            const float current_clearance = std::fabs(r - std::fabs(path_radius));
            if (path.clearance > current_clearance) {
                path.clearance = current_clearance;
            }
        }
    }
    path.clearance = std::max(0.0f, path.clearance);
}

}  // namespace motion_primitives