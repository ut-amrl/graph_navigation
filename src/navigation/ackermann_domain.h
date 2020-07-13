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
\file    ackermann_domain.h
\brief   A* planner 3-connected Ackermann steering domain
\author  Joydeep Biswas, (C) 2019
*/
//========================================================================

// C headers.
#include <inttypes.h>

// C++ headers.
#include <unordered_map>
#include <unordered_set>
#include <vector>

// Library headers.
#include "eigen3/Eigen/Dense"
#include "glog/logging.h"
#include "ros/ros.h"
#include "visualization_msgs/Marker.h"

// Project headers.
#include "shared/math/math_util.h"
#include "shared/ros/ros_helpers.h"
#include "simple_queue.h"

#ifndef ACKERMANN_DOMAIN_H
#define ACKERMANN_DOMAIN_H

namespace navigation {

struct AckermannDomain {
  struct State {
    // Location in R^2.
    Eigen::Vector2f loc;
    // Discretized orientation:
    //   0: East  (+ve X axis)
    //   1: North (+ve Y axis)
    //   2: West  (-ve X axis)
    //   3: South (-ve Y axis)
    int orientation;
    State (float x,
           float y,
           int orientation) : loc(x, y), orientation(orientation) {
      DCHECK_GE(orientation, 0);
      DCHECK_LE(orientation, 3);
    }
    State (const Eigen::Vector2f& loc,
           int orientation) : loc(loc), orientation(orientation) {
      DCHECK_GE(orientation, 0);
      DCHECK_LE(orientation, 3);
    }
  };

  struct Visualizer {
    Visualizer(float radius,
               ros::Publisher* pub,
               visualization_msgs::Marker* msg_ptr) :
        kRadius(radius), publisher(*pub), msg(*msg_ptr) {
      ros_helpers::ClearMarker(msg_ptr);
    }
    void DrawEdge(const State& s1, const State& s2) {
      static const bool kDebug = false;
      if (kDebug) {
        printf("%7.2f,%7.2f,%1d -> %7.2f,%7.2f,%1d\n",
              s1.loc.x(),
              s1.loc.y(),
              s1.orientation,
              s2.loc.x(),
              s2.loc.y(),
              s2.orientation);
      }
      const int kSegments = 10;
      if (s1.orientation == s2.orientation) {
        // Straight line motion.
        ros_helpers::DrawEigen2DLine(s1.loc, s2.loc, &msg);
      } else {
        int turn = s2.orientation - s1.orientation;
        CHECK(turn == 1 || turn == -1 || turn == 3 || turn == -3);
        if (turn == 3) turn = -1;
        if (turn == -3) turn = 1;
        Eigen::Vector2f center = s1.loc;
        float a0 = -0.5 * M_PI;
        switch (s1.orientation) {
          case 0 : {
            center += Eigen::Vector2f(0, turn * kRadius);
            a0 = -0.5 * M_PI;
          }  break;
          case 1 : {
            center += Eigen::Vector2f(-turn * kRadius, 0);
            a0 = 0;
          }  break;
          case 2 : {
            center += Eigen::Vector2f(0, -turn * kRadius);
            a0 = 0.5 * M_PI;
          }  break;
          case 3 : {
            center += Eigen::Vector2f(turn * kRadius, 0);
            a0 = M_PI;
          }  break;
          default: {
            CHECK(false);
          }
        }
        if (turn < 0) a0 = math_util::AngleMod(a0 + M_PI);
        const float a1 = a0 + turn * 0.5 * M_PI;
        const float da = (a1 - a0) / static_cast<float>(kSegments);
        Eigen::Vector2f p0 =
            center + kRadius * Eigen::Vector2f(cos(a0), sin(a0));
        for (int i = 0; i < kSegments; ++i) {
          float a = a0 + da * static_cast<float>(i);
          Eigen::Vector2f p1 =
              center + kRadius * Eigen::Vector2f(cos(a), sin(a));
          ros_helpers::DrawEigen2DLine(p0, p1, &msg);
          p0 = p1;
        }
        Eigen::Vector2f p1 =
            center + kRadius * Eigen::Vector2f(cos(a1), sin(a1));
        ros_helpers::DrawEigen2DLine(p0, p1, &msg);
      }
      publisher.publish(msg);
      if (kDebug) Sleep(0.01);
    }
    const float kRadius;
    ros::Publisher& publisher;
    visualization_msgs::Marker& msg;
  };

  AckermannDomain(float size_x,
                  float size_y,
                  const Eigen::Vector2f& origin,
                  float turn_radius) :
      kTurnRadius(turn_radius),
      kMapWidth(size_x / turn_radius),
      kMapHeight(size_y / turn_radius),
      kStateStepSize(4),
      kStateStride(4 * kMapWidth),
      kMapOrigin(origin),
      kArcLength(kTurnRadius * 0.5 * M_PI) {}

  State KeyToState(uint64_t key) const {
    const float y = key / kStateStride;
    const float x = (key % kStateStride) / kStateStepSize;
    const int r = (key % kStateStride) % kStateStepSize;
    return State(x + kMapOrigin.x(), y + kMapOrigin.y(), r);
  }

  uint64_t StateToKey(const State& s) const {
    Eigen::Vector2i loc_int =
        ((s.loc - kMapOrigin) / kTurnRadius).cast<int>();
    math_util::Bound<int>(0, kMapWidth, &(loc_int.x()));
    math_util::Bound<int>(0, kMapHeight, &(loc_int.y()));
    CHECK_GE(s.orientation, 0);
    CHECK_LE(s.orientation, 3);
    return (loc_int.y() * kStateStride +
        loc_int.x() * kStateStepSize +
        s.orientation);
  }

  // Return the edge cost, assuming the two states are indeed connectable.
  float EdgeCost(const State& s1, const State& s2) const {
    if (s1.orientation == s2.orientation) {
      // Straight line motion.
      return 1.0;
    }
    // Else, the motion is on a pi/2 arc.
    return kArcLength;
  }

  float EdgeCost(const uint64_t k_s1, const uint64_t k_s2) const {
    const int o1 = GetOrientation(k_s1);
    const int o2 = GetOrientation(k_s2);
    if (o1 == o2) {
      // Straight line motion.
      return 1.0;
    }
    // Else, the motion is on a pi/2 arc.
    return kArcLength;
  }

  float Heuristic(const State& s1, const State& s2) const {
    return (s1.loc - s2.loc).norm();
  }

  float Heuristic(const uint64_t k_s1, const uint64_t k_s2) const {
    return (KeyToState(k_s1).loc - KeyToState(k_s2).loc).norm();
  }

  int GetOrientation(uint64_t s_key) const {
    return (s_key % 4);
  }

  int GetX(uint64_t s_key) const {
    return ((s_key % kStateStride) / 4);
  }

  int GetY(uint64_t s_key) const {
    return (s_key / kStateStride);
  }

  bool CheckLineCollision(const Eigen::Vector2f& v1,
                          const Eigen::Vector2f& v2) {
    return false;
  }

  void GetNeighbors(const State& s,
                    std::vector<State>* neighbors) const {
    int o = s.orientation;
    neighbors->clear();
    switch (o) {
      // Facing +x:
      case 0: {
        // Move forward: line check.
        // Turn left: Quadrant 4 check.
        // Turn right: Quadrant 1 check.
      }  break;

      // Facing +y:
      case 1: {
        // Move forward: line check.
        // Turn left: Quadrant 1 check.
        // Turn right: Quadrant 2 check.
      }  break;

      // Facing -x:
      case 2: {
        // Move forward: line check.
        // Turn left: Quadrant 2 check.
        // Turn right: Quadrant 3 check.
      }  break;

      // Facing -y:
      case 3: {
        // Move forward: line check.
        // Turn left: Quadrant 3 check.
        // Turn right: Quadrant 4 check.
      }  break;

      default: {
        // Invalid orientation.
        CHECK(false);
      }
    }
  }
  // Get neighbors to a state.
  void GetNeighborsKeys(uint64_t s_key,
                        std::vector<uint64_t>* neighbors) const {
    int o = GetOrientation(s_key);
    int y = GetY(s_key);
    int x = GetX(s_key);
    neighbors->clear();
    switch (o) {
      // Facing +x:
      case 0: {
        if (x < kMapWidth - 1) {
          // +1 x
          neighbors->push_back(s_key + kStateStepSize);
          if (y < kMapHeight - 1) {
            // +1 x, +1 y, +1 o
            neighbors->push_back(s_key + kStateStride + kStateStepSize + 1);
            // Quadrant 4 arc.
          }
          if (y > 0) {
            // +1 x, -1 y, -1 (+3) o
            neighbors->push_back(s_key + kStateStride - kStateStepSize + 3);
            // Quadrant 1 arc.
          }
        }
      }  break;

      // Facing +y:
      case 1: {
        if (y < kMapHeight - 1) {
          // +1 y
          neighbors->push_back(s_key + kStateStride);
          if (x < kMapWidth - 1) {
            // +1 x, +1 y, -1 o
            neighbors->push_back(s_key + kStateStride + kStateStepSize - 1);
          }
          if (x > 0) {
            // -1 x, +1 y, +1 o
            neighbors->push_back(s_key + kStateStride - kStateStepSize + 1);
          }
        }
      }  break;

      // Facing -x:
      case 2: {
        if (x > 0) {
          // -1 x
          neighbors->push_back(s_key - kStateStepSize);
          if (y < kMapHeight - 1) {
            // -1 x, +1 y, -1 o
            neighbors->push_back(s_key + kStateStride - kStateStepSize - 1);
          }
          if (y > 0) {
            // -1 x, -1 y, +1 o
            neighbors->push_back(s_key - kStateStride - kStateStepSize + 1);
          }
        }
      }  break;

      // Facing -y:
      case 3: {
        if (y > 0) {
          // -1 y
          neighbors->push_back(s_key - kStateStride);
          if (x < kMapWidth - 1) {
            // +1 x, -1 y, +1 (-3) o
            neighbors->push_back(s_key - kStateStride + kStateStepSize - 3);
          }
          if (y > 0) {
            // -1 x, -1 y, -1 o
            neighbors->push_back(s_key - kStateStride - kStateStepSize - 1);
          }
        }
      }  break;

      default: {
        // Invalid orientation.
        CHECK(false);
      }
    }
  };

  const float kTurnRadius;
  const float kMapWidth;
  const float kMapHeight;
  // The number of states per x-step, which is equal to the number of
  // orientations.
  const uint64_t kStateStepSize;
  // The number of states per y-step, which is equal to (number of
  // orientations) * (width of the map).
  const uint64_t kStateStride;
  const Eigen::Vector2f kMapOrigin;
  const float kArcLength;
};

}  // namespace navigation
#endif  // ACKERMANN_DOMAIN_H
