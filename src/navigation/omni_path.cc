#include "omni_path.h"

#include <algorithm>
#include "math/math_util.h"

using Eigen::Vector2f;
using pose_2d::Pose2Df;
using namespace math_util;
using navigation::MotionLimits;

namespace motion_primitives {

OmniPath::OmniPath(const Vector2f& motion, float clearance)
    : motion_(motion),
      clearance_(clearance),
      length_(motion.norm()) {}

float OmniPath::Length() const {
  return length_;
}

float OmniPath::FPL() const {
  return length_;
}

float OmniPath::AngularLength() const {
  return 0.0f;
}

float OmniPath::Clearance() const {
  return clearance_;
}

Pose2Df OmniPath::EndPoint() const {
  return Pose2Df(0.0f, motion_);
}

Pose2Df OmniPath::GetIntermediateState(float f) const {
  f = std::clamp(f, 0.0f, 1.0f);
  return Pose2Df(0.0f, f * motion_);
}

void OmniPath::GetControls(const MotionLimits& linear_limits,
                           const MotionLimits& angular_limits,
                           const float dt,
                           const Vector2f& linear_vel,
                           const float angular_vel,
                           Vector2f& vel_cmd,
                           float& ang_vel_cmd) const {
  vel_cmd.x() = Run1DTimeOptimalControl(
      linear_limits, 0, linear_vel.x(), motion_.x(), 0, dt);
  vel_cmd.y() = Run1DTimeOptimalControl(
      linear_limits, 0, linear_vel.y(), motion_.y(), 0, dt);
  ang_vel_cmd = 0.0f;
}

}  // namespace motion_primitives
