#include "navigation.h"
#include "shared/math/math_util.h"

using Eigen::Rotation2Df;
using Eigen::Vector2f;
using std::atan2;
using std::max;
using std::min;
using std::shared_ptr;
using std::string;
using std::swap;
using std::vector;

using namespace math_util;

DECLARE_bool(no_local);

namespace navigation {
void Navigation::SetSafePose(const bool &gotSafe, const Eigen::Vector2f &loc, const float &angle) {
    got_safe_pose_ = gotSafe;
    safe_local_target_loc_ = loc;
    safe_ground_target_angle_ = angle;
}

Eigen::Vector2f Navigation::GetSafeTarget() {
    return safe_local_target_loc_;
}

float Navigation::GetSafeAngle() {
    return safe_ground_target_angle_;
}

bool Navigation::EnabledContingency() const {
    return contingency_enabled_;
}

void Navigation::EnableContingency(bool enable) {
    contingency_enabled_ = enable;
}

void Navigation::GetSafeLocalLocFromGround(Eigen::Vector2f &loc, const Eigen::Vector2f &gr) {
    loc = Rotation2Df(-robot_angle_) * (gr - robot_loc_);
}

void Navigation::GetSafeGroundAngleFromLocal(float &angle, const float &loc_angle) {
    angle = loc_angle + robot_angle_;
}

// TODO Implement another function to take robofleet input -> set nav_state_ to kContingency
// TODO Make RunObstaAvoid (and maybe TurnInPlace too) return a bool when they cannot do it successfully and catch that bool to return output of Run and ContingencyPlanner properly

bool Navigation::ContingencyPlanner(const Eigen::Vector2f &initial, Eigen::Vector2f &cmd_vel, float &angular_vel_cmd, const bool &kDebug) {
    if (!got_safe_pose_) {
        return false;
    }

    if (kDebug) {
        printf("\nNav Contingency\n");
    }

    if (safe_local_target_loc_.squaredNorm() < Sq(params_.target_dist_tolerance) && robot_vel_.squaredNorm() < Sq(params_.target_vel_tolerance) && AngleDist(robot_angle_, safe_ground_target_angle_) < params_.target_angle_tolerance) {
        Halt(cmd_vel, angular_vel_cmd);
    } else {
        Eigen::Vector2f local_target(0, 0);
        local_target = safe_local_target_loc_;
        const float theta = atan2(local_target.y(), local_target.x());
        if (!FLAGS_no_local) {
            if (fabs(theta) > params_.local_fov || (safe_local_target_loc_.squaredNorm() < Sq(params_.target_dist_tolerance) && robot_vel_.squaredNorm() < Sq(params_.target_vel_tolerance))) {
                if (kDebug)
                    printf("TurnInPlace\n");
                TurnInPlace(cmd_vel, angular_vel_cmd);
            } else {
                if (kDebug)
                    printf("ObstAv\n");
                RunObstacleAvoidance(cmd_vel, angular_vel_cmd);
            }
        }
    }
    return true;
}

}  // namespace navigation