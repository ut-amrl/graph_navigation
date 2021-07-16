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
\file    social_navigation.h
\brief   Implements Social Navigation behaviors on top of graph_navigation
\author  Jarrett Holtz, (C) 2021
*/
//========================================================================
#include "navigation.h"

#ifndef SOCIAL_NAVIGATION_H
#define SOCIAL_NAVIGATION_H

namespace navigation {

enum SocialAction { GoAlone, Halt, Follow, Pass };

struct Human {
  int id;
  Eigen::Vector2f pose;
  Eigen::Vector2f vel;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
};

class SocialNav {
 public:
  explicit SocialNav();
  void Run(const double& time,
           const SocialAction& action,
           const Eigen::Vector2f& robot_loc,
           const float& robot_theta,
           const Odom& odom,
           const std::vector<Eigen::Vector2f>& cloud,
           const std::vector<Human> humans,
           Eigen::Vector2f& cmd_vel, float& cmd_angle_vel);
  void UpdateCommandHistory(const Twist& twist);
  void SetNavGoal(const Eigen::Vector2f& loc, float angle);
  std::vector<GraphDomain::State> Plan(const Eigen::Vector2f& initial,
                                       const Eigen::Vector2f& end);
  std::vector<int> GlobalPlan(const Eigen::Vector2f& initial,
                              const Eigen::Vector2f& end);

  int GetTargetId();
  Eigen::Vector2f GetLocalTarget();
  Navigation* GetGraphNav();
 private:
  Navigation navigation_;
  SocialAction last_action_;
  Eigen::Vector2f pose_;
  float theta_;
  bool target_locked_;
  std::vector<Human> humans_;
  Human follow_target_;
  int target_id_;
  const bool local_humans_ = true;
  void GoAlone();
  void Halt();
  void Follow();
  void Pass();
  Human GetClosest(const std::vector<Human> humans,
      const Eigen::Vector2f pose,
      const std::vector<int> indices,
      int& index,
      int& id);
  Human FindFollowTarget(bool* found);
  Human GetBestAngle(const Human& a,
      const int& id_a,
      const Human& b,
      const int& id_b,
      const Human& c,
      const int& id_c,
      int& output_id);
  bool TargetGood();
  const float kMaxVel = 1.5;
};

}  // namespace navigation

#endif  // SOCIAL_NAVIGATION_H
