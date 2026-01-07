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
\file    omnidirectional_motion_primitives.cc
\brief   Omnidirectional motion primitives implementation.
\author  Sadanand Modak (C) 2025
*/
//========================================================================

#include <math.h>
#include <algorithm>
#include <memory>
#include <vector>

#include "shared/math/poses_2d.h"
#include "eigen3/Eigen/Dense"
#include "omnidirectional_motion_primitives.h"
#include "motion_primitives.h"
#include "navigation.h"

using Eigen::Vector2f;
using pose_2d::Pose2Df;
using std::shared_ptr;
using std::vector;

namespace motion_primitives {

// OmnidirectionalMovePath implementation
float OmnidirectionalMovePath::Length() const { return length; }

float OmnidirectionalMovePath::FPL() const { return fpl; }

float OmnidirectionalMovePath::AngularLength() const {
    return 0.0f;  // No angular movement for straight line motion
}

float OmnidirectionalMovePath::Clearance() const { return clearance; }

float OmnidirectionalMovePath::LOSClearance() const { return los_clearance; }

void OmnidirectionalMovePath::GetControls(const navigation::MotionLimits& linear_limits,
                                          const navigation::MotionLimits& angular_limits, const float dt,
                                          const Vector2f& vel, const float ang_vel, Vector2f& vel_cmd,
                                          float& ang_vel_cmd) const {
    // Calculate velocity component along the path direction
    const float v_along = vel.dot(direction);

    if (v_along < 0.0f) {
        // Wrong-way: robot moving opposite to desired path direction.
        // Must brake first before accelerating toward goal.
        const float speed = vel.norm();
        const float dv = linear_limits.max_deceleration * dt;
        if (speed > 1e-3f) {
            const float new_speed = std::max(0.0f, speed - dv);
            vel_cmd = vel * (new_speed / speed);  // same direction as current motion, reduced magnitude
        } else {
            // Braked to near-zero - can now start toward goal
            vel_cmd = Vector2f::Zero();
        }
    } else {
        // Correct direction or stopped - use 1D TOC
        const float speed = Run1DTimeOptimalControl(linear_limits, 0, v_along, length, 0, dt);
        vel_cmd = speed * direction;
    }

    // No rotation during straight-line motion; navigation owns yaw alignment
    ang_vel_cmd = 0;
}

Pose2Df OmnidirectionalMovePath::GetIntermediateState(float f) const {
    // Straight line movement only
    return Pose2Df(0, f * length * direction);
}

Pose2Df OmnidirectionalMovePath::EndPoint() const { return GetIntermediateState(1.0); }

// OmniSampler implementation
OmniSampler::OmniSampler() {}

namespace {
inline void PrecomputeUnitDirs(int n, std::vector<Eigen::Vector2f>& cache) {
    cache.resize(n);
    const float step = 2.0f * static_cast<float>(M_PI) / static_cast<float>(n);
    for (int i = 0; i < n; ++i) {
        const float a = step * i;
        cache[i] = Eigen::Vector2f(cos(a), sin(a));
    }
}
}  // namespace

void OmniSampler::SetMaxPathLength(OmnidirectionalMovePath* move) {
    // Projection of goal onto this direction (negative = pointing away from goal)
    const float distance_to_goal_along_direction = local_target.dot(move->direction);

    // Allow a small "escape" move even if not making forward progress (lateral/backwards),
    // so these samples don't get discarded by the evaluator.
    constexpr float kEscapeLength = 0.2f;
    const float desired_dist = (distance_to_goal_along_direction > 0.0f)
                                   ? std::min(distance_to_goal_along_direction, nav_params.max_rollout_length)
                                   : std::min(kEscapeLength, nav_params.max_rollout_length);
    move->length = desired_dist;
}

vector<shared_ptr<PathRolloutBase>> OmniSampler::GetSamples(int n) {
    // TODO: add resampling from LOS distn? ie, check line of sight from init samples, drop non-LOS ones, resample from
    // LOS angle distn
    if (n <= 0) return {};
    vector<shared_ptr<PathRolloutBase>> samples(n);

    // Cache unit directions per n to avoid trig every cycle.
    static int cached_n = -1;
    static std::vector<Eigen::Vector2f> unit_dirs;
    if (cached_n != n) {
        PrecomputeUnitDirs(n, unit_dirs);
        cached_n = n;
    }

#pragma omp parallel for schedule(runtime)
    for (int i = 0; i < n; ++i) {
        auto move = std::make_shared<OmnidirectionalMovePath>(unit_dirs[i], 0.0f);
        SetMaxPathLength(move.get());
        CheckObstacles(move.get());
        samples[i] = std::static_pointer_cast<PathRolloutBase>(move);
    }
    return samples;
}

// Checks obstacles for the given omnidirectional move path and computes:
// - fpl: Free Path Length (positive = distance to first collision, negative = escaping path length if penetrating)
// - length: Executed path distance (clamped by FPL and stopping constraints, 0.0 means unusable path)
// - clearance: Lateral clearance during executed segment (0..clearance_band)
// - los_clearance: Line-of-sight clearance from endpoint to local_target
void OmniSampler::CheckObstacles(OmnidirectionalMovePath* move) {
    // Path-aligned basis (in base_link frame)
    const Eigen::Vector2f dir_f = move->direction;       // forward along dir
    const Eigen::Vector2f dir_l(-dir_f.y(), dir_f.x());  // lateral to dir

    // Robot body (no margin) -- ONLY used for skipping points inside the robot body
    const OffsetRect robot_body = {
        Eigen::Vector2f(nav_params.geometric_center_offset.x, nav_params.geometric_center_offset.y),
        0.5f * nav_params.robot_length, 0.5f * nav_params.robot_width};

    // Inflated robot body
    const OffsetRect robot_infl = {robot_body.center, robot_body.half_x + nav_params.obstacle_margin,
                                   robot_body.half_y + nav_params.obstacle_margin};

    // Projections spans of inflated body
    float lat_min, lat_max;
    robot_infl.range(dir_l, lat_min, lat_max);
    float back, front;
    robot_infl.range(dir_f, back, front);

    // --------------------
    // 1) Signed FPL + forward free distance
    // --------------------
    // Compute SIGNED FPL:
    // - If not penetrating now: FPL = +min among all points of the entry time to first hit (standard)
    // - If penetrating now:     FPL = -max among all penetrating of the exit time (escape distance)
    // Also compute fpl_forward: the "true forward free distance to NEW collisions" (ignoring already-penetrating
    // points), used to clamp Length and stopping checks.
    float fpl_forward = nav_params.max_lookahead_fpl;  // distance to first NEW collision
    float escape_dist = 0.0f;                          // distance required to clear all currently-penetrating points
    bool penetrating = false;

    for (const Eigen::Vector2f& p : *point_cloud) {
        if (robot_body.contains(p)) continue;  // skip points inside the robot body

        // Lateral projection doesn't change under translation along dir_f
        const float p_lat = p.dot(dir_l);
        if (p_lat < lat_min || p_lat > lat_max)
            continue;  // skip points outside the lateral projection spans of the inflated body

        bool overlaps_now = false;
        float t_enter = 0.0f, t_exit = 0.0f;
        if (!robot_infl.enter_exit_times(dir_f, p, overlaps_now, t_enter, t_exit)) continue;

        if (overlaps_now) {
            penetrating = true;
            escape_dist = std::max(escape_dist, std::min(t_exit, nav_params.max_lookahead_fpl));
        } else {
            fpl_forward = std::min(fpl_forward, t_enter);
        }
    }

    fpl_forward = std::clamp(fpl_forward, 0.0f, nav_params.max_lookahead_fpl);
    escape_dist = std::clamp(escape_dist, 0.0f, nav_params.max_lookahead_fpl);
    move->fpl = penetrating ? -escape_dist : fpl_forward;

    // Executed distance must not exceed distance to NEW collisions
    move->length = std::min(move->length, fpl_forward);

    // Safety check: if can't stop before NEW collision, mark path unusable
    const float v_f = std::max(0.0f, vel.dot(dir_f));
    const float stopping_dist = (v_f * v_f) / (2.0f * nav_params.linear_limits.max_deceleration);
    if (fpl_forward < stopping_dist) {
        move->length = 0.0f;
    }

    // Early return if path is unusable
    if (move->length <= 0.0f) {
        move->clearance = 0.0f;
        move->los_clearance = 0.0f;
        return;
    }

    // --------------------
    // 2) Clearance over executed segment
    // --------------------
    float clearance = nav_params.clearance_band;
    const float lat_search_min = lat_min - nav_params.clearance_band;
    const float lat_search_max = lat_max + nav_params.clearance_band;

    for (const Eigen::Vector2f& p : *point_cloud) {
        if (robot_body.contains(p)) continue;

        const float p_f = p.dot(dir_f);
        const float p_lat = p.dot(dir_l);

        // Finding the t in [0, executed_length] where this point p is in the pependicular-to-u inflated body slab as
        // that is when this point affects the clearance value, ie, back <= (p_f - t) <= front  =>  t ∈ [p_f - front,
        // p_f - back]
        const float t0 = p_f - front;
        const float t1 = p_f - back;
        // Skip if it never enters for t>=0 the perp-slab or enters beyond executed length
        if (t1 < 0.0f || t0 > move->length) continue;
        // Skip if it is outside the lateral clearance search band
        if (p_lat < lat_search_min || p_lat > lat_search_max) continue;

        float lateral_dist = 0.0f;
        if (p_lat < lat_min)
            lateral_dist = lat_min - p_lat;
        else if (p_lat > lat_max)
            lateral_dist = p_lat - lat_max;
        else
            lateral_dist = 0.0f;  // inside inflated body slab => clearance 0. Note: this is NOT =contains(). This is a
                                  // conservative approximation

        clearance = std::min(clearance, lateral_dist);
        if (clearance <= 0.0f) break;
    }
    move->clearance = std::max(0.0f, clearance);

    // --------------------
    // 3) LOS Clearance: clearance from endpoint to local_target
    // --------------------
    const Eigen::Vector2f endpoint = move->length * move->direction;
    move->los_clearance = motion_primitives::LOSClearanceToLine(geometry::Line2f(endpoint, local_target), *point_cloud);
}

}  // namespace motion_primitives