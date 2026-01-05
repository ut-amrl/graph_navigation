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
\file    omnidirectional_motion_primitives.h
\brief   Omnidirectional motion primitives header.
\author  Sadanand Modak (C) 2025
*/
//========================================================================

#ifndef OMNIDIRECTIONAL_MOTION_PRIMITIVES_H
#define OMNIDIRECTIONAL_MOTION_PRIMITIVES_H

#include <algorithm>
#include <cmath>
#include <limits>

#include "motion_primitives.h"

namespace motion_primitives {

// Rectangle with center offset from base_link, sides aligned with base_link axes.
// Denote {B} as base_link aligned frame at base_link origin, {G} as base_link aligned frame at geometric center.
struct OffsetRect {
    Eigen::Vector2f center;  // center position in {B}
    float half_x, half_y;    // half-side lengths in {G}

    // Check if point (in {B}) is inside this rectangle
    bool contains(const Eigen::Vector2f& p) const {
        const Eigen::Vector2f rel = p - center;
        return std::fabs(rel.x()) <= half_x && std::fabs(rel.y()) <= half_y;
    }

    // Extent from geometric center in direction u: half_x*|u.x| + half_y*|u.y|
    float extent(const Eigen::Vector2f& u) const { return half_x * std::fabs(u.x()) + half_y * std::fabs(u.y()); }

    // Support function from base_link origin in direction u: h(u) = (center · u) + extent(u)
    float support(const Eigen::Vector2f& u) const { return center.dot(u) + extent(u); }

    // Projection interval [min,max] of this rectangle onto direction u. Both min and max are signed values, along u.
    void range(const Eigen::Vector2f& u, float& min_proj, float& max_proj) const {
        const float c = center.dot(u);
        const float e = extent(u);
        min_proj = c - e;  // -support(-u)
        max_proj = c + e;  // support(u)
    }

    // Slab intersection for translation:
    // Compute the time interval [t_enter, t_exit] where point p, when translated along -dir
    // (i.e., p - t*dir), lies inside this rectangle. This is equivalent to translating the rectangle
    // along +dir until it contains point p.
    // Returns:
    // - true if there's any intersection for t >= 0, false otherwise.
    // - sets overlaps_now to true if point p is currently inside/on boundary at t=0.
    // - sets t_enter to t >= 0 when p enters the rectangle (or 0 if already inside at t=0).
    // - sets t_exit to t >= 0 when p exits the rectangle.
    // Uses ray-AABB "slab" intersection: for each axis, compute t-intervals where the point
    // is within slab bounds, then intersect all per-axis intervals.
    bool enter_exit_times(const Eigen::Vector2f& dir, const Eigen::Vector2f& p, bool& overlaps_now, float& t_enter,
                          float& t_exit) const {
        constexpr float kEps = 1e-6f;
        const Eigen::Vector2f q = p - center;  // p in {G} (geometric center frame)

        float tmin = -std::numeric_limits<float>::infinity();
        float tmax = std::numeric_limits<float>::infinity();

        auto update_axis = [&](float q_axis, float u_axis, float half_extent) -> bool {
            if (std::fabs(u_axis) < kEps) {
                // No motion along this axis: must already be within slab to ever intersect.
                return (q_axis >= -half_extent && q_axis <= half_extent);
            }
            const float inv_u = 1.0f / u_axis;
            // For slab: -half_extent <= q_axis - t*u_axis <= +half_extent
            const float t0 = (q_axis - half_extent) * inv_u;  // t when hitting +half_extent boundary
            const float t1 = (q_axis + half_extent) * inv_u;  // t when hitting -half_extent boundary
            const float a = std::min(t0, t1);                 // entry time for this axis
            const float b = std::max(t0, t1);                 // exit time for this axis
            tmin = std::max(tmin, a);                         // tighten entry time across all axes
            tmax = std::min(tmax, b);                         // tighten exit time across all axes
            return (tmin <= tmax);
        };

        // If tmin > tmax, there is no intersection for t >= 0.
        if (!update_axis(q.x(), dir.x(), half_x)) return false;
        if (!update_axis(q.y(), dir.y(), half_y)) return false;

        // Entire intersection is in the "past" wrt dir → no hit for t >= 0.
        if (tmax < 0.0f) return false;

        overlaps_now = (tmin <= 0.0f && tmax >= 0.0f);
        t_enter = std::max(0.0f, tmin);
        t_exit = tmax;
        return true;
    }
};

// Omnidirectional motion primitive - straight line movement in x-y plane
struct OmnidirectionalMovePath : PathRolloutBase {
    ~OmnidirectionalMovePath() = default;

    float Length() const override;
    float FPL() const override;
    float AngularLength() const override;
    float Clearance() const override;
    float LOSClearance() const override;
    pose_2d::Pose2Df EndPoint() const override;
    pose_2d::Pose2Df GetIntermediateState(float f) const override;
    void GetControls(const navigation::MotionLimits& linear_limits, const navigation::MotionLimits& angular_limits,
                     const float dt, const Eigen::Vector2f& linear_vel, const float angular_vel,
                     Eigen::Vector2f& vel_cmd, float& ang_vel_cmd) const override;

    // Default constructor
    OmnidirectionalMovePath()
        : direction(0, 0), length(0), fpl(0), clearance(0), los_clearance(std::numeric_limits<float>::infinity()) {}

    // Constructor for straight line movement
    OmnidirectionalMovePath(const Eigen::Vector2f& dir, float len)
        : direction(dir.normalized()),
          length(len),
          fpl(0),
          clearance(0),
          los_clearance(std::numeric_limits<float>::infinity()) {}

    Eigen::Vector2f direction;  // Unit vector for movement direction
    float length;               // Executed distance along direction (clamped by FPL + stopping constraint)
    float fpl;            // Free path length: distance until inflated footprint would first collide along direction
    float clearance;      // Extra lateral buffer to inflated footprint during executed segment (0..clearance_band)
    float los_clearance;  // Lateral clearance from path endpoint to local_target (FLT_MAX if no obstacles project)
};

// Omnidirectional path rollout sampler
struct OmniSampler : PathRolloutSamplerBase {
    std::vector<std::shared_ptr<PathRolloutBase>> GetSamples(int n) override;
    OmniSampler();

    void CheckObstacles(OmnidirectionalMovePath* move);
    void SetMaxPathLength(OmnidirectionalMovePath* move);
};

}  // namespace motion_primitives

#endif  // OMNIDIRECTIONAL_MOTION_PRIMITIVES_H