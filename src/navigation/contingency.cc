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

void Navigation::SetSafePose(const Eigen::Vector2f &loc, float angle) {
    nav_state_ = NavigationState::kContingency;
    safe_local_target_loc_ = loc;
    safe_local_target_angle_ = angle;
}

Eigen::Vector2f Navigation::GetSafeTarget() {
    return safe_local_target_loc_;
}

float Navigation::GetSafeAngle() {
    return safe_local_target_angle_;
}

bool Navigation::ExtractSafePose(Eigen::Vector2f &loc, float &angle) {
    // TODO Implement: extract from image rostopic, convert to robot ref frame then

    // Testing hardcoded (the below values are what you see when you set nav goal and websocket outputs)
    Eigen::Vector2f dummy_loc(30.21, 21.12);
    float dummy_angle(-0.64);
    loc = Rotation2Df(-robot_angle_) * (dummy_loc - robot_loc_);
    angle = dummy_angle;
    return true;
}

// TODO Implement another function to take joystick input -> set nav_state_ to kContingency
// TODO Implement another function to take robofleet input -> set nav_state_ to kContingency

bool Navigation::ContingencyPlanner(const Eigen::Vector2f &initial, Eigen::Vector2f &cmd_vel, float &angular_vel_cmd, const bool &kDebug) {
    Eigen::Vector2f safe_loc(0, 0);
    float safe_angle(0);
    bool gotSafePose = ExtractSafePose(safe_loc, safe_angle);
    if (!gotSafePose) {
        return false;
    }
    SetSafePose(safe_loc, safe_angle);

    if (safe_local_target_loc_.squaredNorm() < Sq(params_.target_dist_tolerance) && robot_vel_.squaredNorm() < Sq(params_.target_vel_tolerance) && AngleDist(robot_angle_, safe_local_target_angle_) < params_.target_angle_tolerance) {
        nav_state_ = NavigationState::kStopped;
    }

    if (kDebug) {
        printf("\nNav Contingency\n");
    }

    if (nav_state_ == NavigationState::kStopped) {
        Halt(cmd_vel, angular_vel_cmd);
        return true;
    } else if (nav_state_ == NavigationState::kContingency) {
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