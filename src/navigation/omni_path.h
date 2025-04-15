#ifndef OMNI_MOTION_PRIMITIVE_H
#define OMNI_MOTION_PRIMITIVE_H

#include <memory>
#include "motion_primitives.h"
#include "math/poses_2d.h"
#include "eigen3/Eigen/Dense"

namespace motion_primitives {

class OmniPath : public PathRolloutBase {
 public:
 Eigen::Vector2f motion;
 float length;


  OmniPath(const Eigen::Vector2f& motion, float clearance);
  ~OmniPath() override = default;

  float Length() const override;
  float FPL() const override;
  float AngularLength() const override;
  float Clearance() const override;

  pose_2d::Pose2Df EndPoint() const override;
  pose_2d::Pose2Df GetIntermediateState(float f) const override;

  void GetControls(const navigation::MotionLimits& linear_limits,
                   const navigation::MotionLimits& angular_limits,
                   const float dt,
                   const Eigen::Vector2f& linear_vel,
                   const float angular_vel,
                   Eigen::Vector2f& vel_cmd,
                   float& ang_vel_cmd) const override;

 private:
  float clearance_;
};

}  // namespace motion_primitives

#endif  // OMNI_MOTION_PRIMITIVE_H
