#include "omni_sampler.h"
#include "omni_path.h"
#include "math/math_util.h"

#include <cmath>
#include <memory>
#include <algorithm>

using Eigen::Vector2f;
using namespace math_util;

namespace motion_primitives {
  OmniSampler::OmniSampler() {
    // You can leave this empty or initialize things later if needed.
  }

  void OmniSampler::SetMaxPathLength(OmniPath* path_ptr) {
    path_ptr->length = std::min(nav_params.max_free_path_length, path_ptr->motion.norm());
    path_ptr->length = std::min(path_ptr->length, local_target.norm());
    const float stopping_dist = 
        (Sq(vel.x()) + Sq(vel.y())) / (2.0 * nav_params.linear_limits.max_deceleration);
    path_ptr->length = std::max(path_ptr->length, stopping_dist);

    path_ptr->motion = path_ptr->motion.normalized() * path_ptr->length;
  }

  std::vector<std::shared_ptr<PathRolloutBase>> OmniSampler::GetSamples(int n) {
    std::vector<std::shared_ptr<PathRolloutBase>> samples;
  
    // --- Hardcoded obstacle avoidance parameters ---
    const float angularResolution = M_PI / 180.0f;  // 1 degree
    // const float robotRadius = 0.24f;
    const float minClearPathLength = 0.75f; // 0.75f;
    const float maxDeviationAngle = M_PI / 3.0f;    // 60 degrees
    const float maxObstacleDistance = 3.0f; // 4.0f
    const float robotRadius = 0.35f;
  
    // --- Sample directions ---
    const int numAngles = static_cast<int>(2 * M_PI / angularResolution);
    std::vector<float> ranges(numAngles, maxObstacleDistance);
    std::vector<float> angles(numAngles);
  
    for (int i = 0; i < numAngles; ++i) {
      // angles[i] = i * angularResolution - 0.5f * M_PI;
      angles[i] = i * angularResolution - M_PI;
    }
  
    // --- Process point cloud ---
    for (const Eigen::Vector2f& p : point_cloud) {
      if (!std::isfinite(p.x()) || !std::isfinite(p.y())) continue;
  
      float r = p.norm();
      if (r > maxObstacleDistance) continue;
  
      float a = atan2(p.y(), p.x());
  
      float dA = 2.0f * atan2(robotRadius, r);
      float aMin = a - dA;
      float aMax = a + dA;
  
      int iMin = static_cast<int>(std::floor((aMin + M_PI) / angularResolution));
      int iMax = static_cast<int>(std::floor((aMax + M_PI) / angularResolution));
  
      for (int i = iMin; i < iMax; ++i) {
        ranges[i] = std::min(ranges[i], r);
      }
    }
  
    // --- Desired direction ---
    const Eigen::Vector2f desired = local_target.normalized();
    const float desired_angle = atan2(desired.y(), desired.x());
  
    for (int i = 0; i < numAngles; ++i) {
      float a = angles[i];
      float deviation = fabs(AngleDiff(a, desired_angle));
      if (deviation > maxDeviationAngle || ranges[i] < minClearPathLength) {
        continue;
      }
  
      float range = ranges[i];
      Eigen::Vector2f motion(cos(a), sin(a));
      motion *= range;
  
      auto path = std::make_shared<OmniPath>(motion, 0.0f);
      samples.push_back(path);
    }
  
    return samples;
  }

}  // namespace motion_primitives
