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
\file    social_navigation.cc
\brief   Implements Social Navigation behaviors on top of graph_navigation
\author  Jarrett Holtz, (C) 2021
*/
//========================================================================
#include <string>
#include "social_nav.h"
#include "graph_domain.h"
#include "math/geometry.h"
#include "math/line2d.h"

using Eigen::Vector2f;
using Eigen::Rotation2Df;
using std::string;
using std::vector;
using navigation::Odom;
using navigation::Twist;
using navigation::SocialAction;
using navigation::SocialNav;
using navigation::GraphDomain;
using geometry::Line2f;
using math_util::DegToRad;

namespace navigation {

SocialNav::SocialNav() :
  navigation_(),
  last_action_(SocialAction::GoAlone),
  pose_(0,0),
  theta_(0),
  target_locked_(false),
  humans_({}),
  target_id_(0) {
}

void SocialNav::UpdateCommandHistory(const Twist& twist) {
  navigation_.UpdateCommandHistory(twist);
}

int SocialNav::GetTargetId() {
  return target_id_;
}

Navigation* SocialNav::GetGraphNav() {
  return &navigation_;
}

Eigen::Vector2f SocialNav::GetLocalTarget() {
  return navigation_.GetTarget();
}

Human SocialNav::GetClosest(const vector<Human> humans,
                 const Vector2f pose,
                 const vector<int> indices,
                 int& index,
                 int& id) {
  float best_dist = 9999;
  Human best_human;
  best_human.id = -1;
  best_human.pose = {0, 0};
  int best_index = 0;
  int count = 0;
  if (humans.size() == 0) {
    return best_human;
  }
  for (Human human : humans) {
    const Vector2f h_pose = human.pose;
    if (h_pose.norm() < best_dist) {
      best_dist = h_pose.norm();
      best_human = human;
      best_index = count;
    }
    count++;
  }
  id = indices[best_index];
  index = best_index;
  return best_human;
}

Human SocialNav::GetBestAngle(const Human& a,
                   const int& id_a,
                   const Human& b,
                   const int& id_b,
                   const Human& c,
                   const int& id_c,
                   int& output_id) {
  const bool kDebug = false;
  const float target_angle = geometry::Angle(navigation_.GetTarget());
  const Vector2f robot_vel = navigation_.GetVelocity();
  const Vector2f adjusted_a = a.vel + robot_vel;
  const float angle_a = geometry::Angle(adjusted_a);
  const Vector2f adjusted_b = b.vel + robot_vel;
  const float angle_b = geometry::Angle(adjusted_b);
  const Vector2f adjusted_c = c.vel + robot_vel;
  const float angle_c = geometry::Angle(adjusted_c);
  const float diff_a = fabs(math_util::AngleDiff(target_angle, angle_a));
  const float diff_b = fabs(math_util::AngleDiff(target_angle, angle_b));
  const float diff_c = fabs(math_util::AngleDiff(target_angle, angle_c));

  const float b_threshold = DegToRad(30.0);
  const float c_threshold = DegToRad(30.0);

  if (kDebug) {
    cout << "Target Angle: " << target_angle << endl;
    cout << "Angle A: " << angle_a << endl;
    cout << "Angle B: " << angle_b << endl;
    cout << "Angle C: " << angle_c << endl;
    cout << "Diff A: " << diff_a << endl;
    cout << "Diff B: " << diff_b << endl;
    cout << "Diff C: " << diff_c << endl;
  }

  if (diff_a - diff_b < b_threshold || id_b < 0) {
    if (diff_a - diff_c < c_threshold || id_c < 0) {
      output_id = id_a;
      return a;
    }
  }

  if (diff_b - diff_c < b_threshold || id_c < 0) {
    output_id = id_b;
    return b;
  }

  output_id = id_c;
  return c;
}

Human SocialNav::FindFollowTarget(bool* found) {
  // TODO(jaholtz) make this and the one used by pips the same?
  vector<Human> front;
  vector<int> indices;
  const float kLowerAngle = DegToRad(90.0);
  const float kUpperAngle = DegToRad(30.0);

  for (size_t i = 0; i < humans_.size(); i++) {
    Human human = humans_[i];
    const Vector2f h_pose = human.pose;
    if (h_pose.norm() < geometry::kEpsilon) {
      continue;
    }
    const float angle = math_util::AngleMod(geometry::Angle(h_pose));
    if (h_pose.x() > 0) {
      if (angle < kLowerAngle || angle > kUpperAngle) {
        front.push_back(human);
        indices.push_back(i);
      }
    }
  }
  *found = true;
  if (front.size() < 1) {
    *found = false;
    Human best_human;
    best_human.id = -1;
    best_human.pose = {0, 0};
    return best_human;
  }
  int id_a = -1;
  int index = -1;
  const Human h_a = GetClosest(front, pose_, indices, index, id_a);
  if (index > -1) {
    front.erase(front.begin() + index);
    indices.erase(indices.begin() + index);
  }
  int id_b = -1;
  index = -1;
  const Human h_b = GetClosest(front, pose_, indices, index, id_b);
  if (index > -1) {
    front.erase(front.begin() + index);
    indices.erase(indices.begin() + index);
  }
  int id_c = -1;
  index = -1;
  const Human h_c = GetClosest(front, pose_, indices, index, id_c);
  return GetBestAngle(h_a, id_a, h_b, id_b, h_c, id_c, target_id_);
}

void SocialNav::SetNavGoal(const Eigen::Vector2f &loc, float angle) {
  navigation_.SetNavGoal(loc, angle);
}

bool SocialNav::TargetGood() {
  const float kFollowThresh = 5.0;
  const bool lost_target =
      humans_[target_id_].pose.norm() < geometry::kEpsilon;
  const bool too_far =
    humans_[target_id_].pose.norm() > kFollowThresh;
  // TODO(jaholtz) Check if target moving in the right direction
  return !too_far && !lost_target;
}

// Helper function for left and right, identifies the robot's travel
// direction along an edge, and returns the angle of edge travel.
float SocialNav::GetTravelAngle(const Line2f& edge,
                                const Vector2f& current_carrot) {
  // Determine which direction the robot is moving along the edge.
  const Vector2f dir_1(edge.p1 - edge.p0);
  const Vector2f dir_2(edge.p0 - edge.p1);
  const Vector2f robot_dir(current_carrot - pose_);
  float angle_1 = geometry::Angle(dir_1);
  // if (angle_1 < 0) {
    // angle_1 = angle_1 + M_PI;
  // }
  const float angle_2 = angle_1 + (M_PI / 2);
  float robot_angle = geometry::Angle(robot_dir);
  // if (robot_angle < 0) {
    // robot_angle = robot_angle + M_PI;
  // }
  const float diff_1 = math_util::AngleDiff(angle_1, robot_angle);
  const float diff_2 = math_util::AngleDiff(angle_2, robot_angle);
  if (diff_1 < diff_2) {
    return angle_1;
  }
  return angle_2;
}

void SocialNav::GoAlone() {
  target_locked_ = false;
  navigation_.SetMaxVel(kMaxVel);
  navigation_.SetObstacleMargin(0.1);
  navigation_.SetClearanceWeight(0.0);
  navigation_.Resume();
}

// Drives in the left 'lane' with respect to the nav edge.
void SocialNav::Left() {
  // cout << "Left" << endl;
  // Find the navigation edge (line).
  GraphDomain::NavigationEdge closest_edge;
  float closest_dist = FLT_MAX;
  // TODO(jaholtz) document GetTarget, GetCarrot, GetOverrideTarget, etc.
  Vector2f local_nav_target = navigation_.GetTarget();
  const Vector2f current_carrot = (Rotation2Df(theta_) *
      local_nav_target) + pose_;
  navigation_.GetNavEdge(current_carrot,
                         &closest_edge,
                         &closest_dist);

  // Project the point onto the edge
  const Line2f edge = closest_edge.edge;
  const Vector2f proj = edge.Projection(current_carrot);

  // Get the angle of travel along the edge.
  const float angle = GetTravelAngle(edge, current_carrot);

  // Project the carrot to the 'left' side of the edge.
  // Use this as the override carrot.
  const Vector2f left(0.0, 0.4*closest_edge.width);
  Rotation2Df rotate(angle);
  const Vector2f override = (rotate * left) + proj;
  const Vector2f local_target = Rotation2Df(-theta_) * (override - pose_);
  navigation_.SetOverride(local_target, 0.0);
  target_locked_ = false;
  navigation_.SetMaxVel(kMaxVel);
  navigation_.SetObstacleMargin(0.1);
  navigation_.SetClearanceWeight(-0.5);
}

// Drives in the right 'lane' with respect to the nav edge.
void SocialNav::Right() {
  // Find the navigation edge (line).
  GraphDomain::NavigationEdge closest_edge;
  float closest_dist = FLT_MAX;
  // TODO(jaholtz) document GetTarget, GetCarrot, GetOverrideTarget, etc.
  const Vector2f current_carrot = (Rotation2Df(theta_) *
      navigation_.GetTarget()) + pose_;
  navigation_.GetNavEdge(current_carrot,
                         &closest_edge,
                         &closest_dist);

  // Project the point onto the edge
  const Line2f edge = closest_edge.edge;
  const Vector2f proj = edge.Projection(current_carrot);

  // Get the angle of travel along the edge.
  const float angle = GetTravelAngle(edge, current_carrot);

  // Project the carrot to the 'right' side of the edge.
  // Use this as the override carrot.
  const Vector2f right(0.0, 0.4 * -closest_edge.width);
  Rotation2Df rotate(angle);
  const Vector2f override = (rotate * right) + proj;
  const Vector2f local_target = Rotation2Df(-theta_) * (override - pose_);
  navigation_.SetOverride(local_target, 0.0);
  target_locked_ = false;
  navigation_.SetMaxVel(kMaxVel);
  navigation_.SetObstacleMargin(0.1);
  navigation_.SetClearanceWeight(-0.5);
}

// Moves out of the way of traffic, off to the appropriate side of the edge.
void SocialNav::StepAside() {
  // Find the navigation edge (line).
  GraphDomain::NavigationEdge closest_edge;
  float closest_dist;
  // TODO(jaholtz) document GetTarget, GetCarrot, GetOverrideTarget, etc.
  navigation_.GetNavEdge(pose_,
                         &closest_edge,
                         &closest_dist);

  // Project the point onto the edge
  const Line2f edge = closest_edge.edge;
  const Vector2f proj = edge.Projection(pose_);

  // Find the closest standoff point to the current robot.
  const static float kStandAsideDist = 0.75;
  const Vector2f right(0.0, -kStandAsideDist * closest_edge.width);
  const Vector2f left(0.0, kStandAsideDist * closest_edge.width);
  Rotation2Df rotate(geometry::Angle(edge.Dir()));
  const Vector2f left_point = proj + rotate * left;
  const Vector2f right_point = proj + rotate * right;

  const float dist_l = (pose_ - left_point).norm();
  const float dist_r = (pose_ - right_point).norm();

  Vector2f target = right_point;
  // Set the override to the closest stand off point.
  if (dist_l < dist_r) {
    target = left_point;
  }
  const Vector2f local_target = Rotation2Df(-theta_) * (target - pose_);
  navigation_.SetOverride(local_target, 0.0);
  target_locked_ = false;
  navigation_.SetMaxVel(kMaxVel);
  navigation_.SetObstacleMargin(0.1);
  navigation_.SetClearanceWeight(-0.5);
}

// Stops in Place
void SocialNav::Halt() {
  target_locked_ = false;
  navigation_.Pause();
}

// Follows the closest human moving in the desired travel direction.
void SocialNav::Follow() {
  const float kFollowDist = 1.0;
  bool found = true;
  follow_target_ = FindFollowTarget(&found);
  if (!found) {
    Halt();
    return;
  }
  const Vector2f h_pose = follow_target_.pose;
  const Vector2f target_pose =
      h_pose - (kFollowDist * h_pose.normalized());
  navigation_.SetMaxVel(kMaxVel);
  navigation_.SetObstacleMargin(0.1);
  navigation_.SetClearanceWeight(0.00);
  navigation_.SetOverride(target_pose, 0.0);
}

// Passes the closest human in the desired travel direction.
void SocialNav::Pass() {
  // const float kLeadDist = 2.0;
  bool found = true;
  follow_target_ = FindFollowTarget(&found);
  if (!found) {
    Halt();
    return;
  }
  // const Vector2f h_pose = follow_target_.pose;
  // const Vector2f local_target = GetLocalTarget();
  // const Vector2f target_dir = local_target - h_pose;

  // const Vector2f target_pose =
      // h_pose + (kLeadDist * target_dir.normalized());
  navigation_.SetMaxVel(kMaxVel * 1.5);
  navigation_.SetObstacleMargin(0.5);
  navigation_.SetClearanceWeight(10.0);
  // navigation_.SetOverride(target_pose, 0.0);
}

std::vector<int> SocialNav::GlobalPlan(const Eigen::Vector2f &initial,
                           const Eigen::Vector2f &end) {
  return navigation_.GlobalPlan(initial, end);
}

void SocialNav::Run(const double& time,
                    const SocialAction& action,
                    const Eigen::Vector2f& robot_loc,
                    const float& robot_theta,
                    const Odom& odom,
                    const std::vector<Eigen::Vector2f>& cloud,
                    const vector<Human> humans,
                    Vector2f& cmd_vel, float& cmd_angle_vel) {
  humans_ = humans;
  // Update the necessary components in the underlying navigation
  navigation_.UpdateOdometry(odom);
  pose_ = robot_loc;
  theta_ = robot_theta;
  navigation_.UpdateLocation(robot_loc, robot_theta);
  navigation_.ObservePointCloud(cloud, time);

  if (navigation_.Initialized()) {
    // Call the appropriate action
    if (action == SocialAction::GoAlone) {
      GoAlone();
    } else if (action == SocialAction::Follow) {
      Follow();
    } else if (action == SocialAction::Pass) {
      Pass();
    } else if (action == SocialAction::Left) {
      Left();
    } else if (action == SocialAction::Right) {
      Right();
    } else if (action == SocialAction::StepAside) {
      StepAside();
    } else {
      Halt();
    }

    // Run the Underlying Navigation
    navigation_.Run(time, cmd_vel, cmd_angle_vel);
  } else {
    GoAlone();
  }
}

}  // namespace navigation
