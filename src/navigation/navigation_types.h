#pragma once

#include <vector>

#include "eigen3/Eigen/Dense"
#include "shared/math/gps_util.h"

namespace navigation {

struct PathOption {
  float curvature;
  float clearance;
  float free_path_length;
  float clearance_to_goal;
  float dist_to_goal;
  explicit PathOption(float c) : curvature(c) {}
  PathOption() {}
  Eigen::Vector2f obstruction;
  Eigen::Vector2f closest_point;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
};

struct Twist {
  double time;
  Eigen::Vector3f linear;
  Eigen::Vector3f angular;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
};

struct Odom {
  double time;
  Eigen::Vector3f position;
  Eigen::Quaternionf orientation;
  Eigen::Affine2f toAffine2f() const {
    return Eigen::Translation2f(position.x(), position.y()) *
           Eigen::Rotation2Df(2.0f * atan2f(orientation.z(), orientation.w()));
  }
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
};

struct MissionStatus {
  double time;
  uint8_t status;
  int64_t mission_id;
  int64_t next_goal_id;
  std::vector<gps_util::GPSPoint> goals;
  std::vector<gps_util::GPSPoint> goals_reached;

  MissionStatus() : time(0), status(0), mission_id(-1), next_goal_id(-1) {}
};

struct CarrotPlan {
  int path_idx;
  std::vector<Eigen::Vector2f> path;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
};

}  // namespace navigation