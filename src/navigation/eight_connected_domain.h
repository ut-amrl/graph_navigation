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
\file    astar.h
\brief   Implementation of A* planner
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
#include "std_msgs/ColorRGBA.h"
#include "visualization_msgs/Marker.h"

// Project headers.
#include "shared/math/math_util.h"
#include "shared/ros/ros_helpers.h"
#include "simple_queue.h"
#include "vector_map/vector_map.h"

#ifndef EIGHT_CONNECTED_DOMAIN_H
#define EIGHT_CONNECTED_DOMAIN_H

namespace navigation {

struct EightConnectedDomain {
  typedef Eigen::Vector2f State;

  EightConnectedDomain(float size_x,
                       float size_y,
                       const Eigen::Vector2f& origin,
                       float resolution,
                       const std::string& map_file) :
      kResolution(resolution),
      kMapWidthInt(size_x / resolution),
      kMapHeightInt(size_y / resolution),
      kDiagonalLength(std::sqrt(2.0f)),
      kMapOrigin(origin),
      vector_map_(map_file),
      kMargin(0.1),
      offset_(0,0) {}

  State KeyToState(uint64_t key) const {
    const float y = kResolution * static_cast<float>(key / kMapWidthInt);
    const float x = kResolution * static_cast<float>(key % kMapWidthInt);
    return (State(x + kMapOrigin.x(), y + kMapOrigin.y()) + offset_);
  }

  uint64_t StateToKey(const State& s) const {
    Eigen::Vector2i loc_int =
        ((s - kMapOrigin - offset_) / kResolution).cast<int>();
    math_util::Bound<int>(0, kMapWidthInt, &(loc_int.x()));
    math_util::Bound<int>(0, kMapHeightInt, &(loc_int.y()));
    return (loc_int.y() * kMapWidthInt + loc_int.x());
  }

  // Return the edge cost, assuming the two states are indeed connectable.
  float EdgeCost(const State& s1, const State& s2) const {
    if (s1.x() == s2.x() || s1.y() == s2.y()) {
      return 1;
    } else {
      return kDiagonalLength;
    }
  }

  float EdgeCost(const uint64_t k_s1, const uint64_t k_s2) const {
    const uint64_t x1_int = k_s1 % kMapWidthInt;
    const uint64_t y1_int = k_s1 / kMapWidthInt;
    const uint64_t x2_int = k_s2 % kMapWidthInt;
    const uint64_t y2_int = k_s2 / kMapWidthInt;

    if (x1_int == x2_int || y1_int == y2_int) {
      return 1;
    } else {
      return kDiagonalLength;
    }
  }

  float Heuristic(const State& s1, const State& s2) const {
    return ((s1 - s2).norm() / kResolution);
  }

  float Heuristic(const uint64_t k_s1, const uint64_t k_s2) const {
    const int64_t dx = k_s1 % kMapWidthInt - k_s2 % kMapWidthInt;
    const int64_t dy = k_s1 / kMapWidthInt - k_s2 / kMapWidthInt;
    return std::sqrt(static_cast<float>(dx * dx + dy * dy));
  }

  bool CheckLineCollision(const Eigen::Vector2f& v1,
                          const Eigen::Vector2f& v2) {
    return false;
  }

  void GetNeighbors(const State& s,
                    std::vector<State>* neighbors) const {
    printf("%s unimplemented\n", __PRETTY_FUNCTION__);
  }

  template<int dx, int dy>
  void CheckAndAdd(const State& s,
                   const uint64_t& x_int,
                   const uint64_t& y_int,
                   uint64_t s_key,
                   std::vector<uint64_t>* neighbors) const {
    if (dx < 0 && int64_t(x_int) < dx) return;
    else if (x_int + dx >= kMapWidthInt) return;
    if (dy < 0 && int64_t(y_int) < dy) return;
    else if (dy == 1 && y_int + dy >= kMapHeightInt) return;
    const Eigen::Vector2f s1 = s + kResolution * Eigen::Vector2f(dx, dy);
    // if (vector_map_.Intersects(s, s1)) return;
    for (const auto& l : vector_map_.lines) {
      if (l.CloserThan(s, s1, kMargin)) return;
    }
    const uint64_t s1_key = s_key + dx + dy * kMapWidthInt;
    neighbors->push_back(s1_key);
  }

  // Get neighbors to a state.
  void GetNeighborsKeys(uint64_t s_key,
                        std::vector<uint64_t>* neighbors) const {
    neighbors->clear();
    const uint64_t x_int = s_key % kMapWidthInt;
    const uint64_t y_int = s_key / kMapWidthInt;
    if (true) {
      const State s = KeyToState(s_key);
      CheckAndAdd<1, 0>(s, x_int, y_int, s_key, neighbors);
      CheckAndAdd<1, 1>(s, x_int, y_int, s_key, neighbors);
      CheckAndAdd<0, 1>(s, x_int, y_int, s_key, neighbors);
      CheckAndAdd<-1, 1>(s, x_int, y_int, s_key, neighbors);
      CheckAndAdd<-1, 0>(s, x_int, y_int, s_key, neighbors);
      CheckAndAdd<-1, -1>(s, x_int, y_int, s_key, neighbors);
      CheckAndAdd<0, -1>(s, x_int, y_int, s_key, neighbors);
      CheckAndAdd<1, -1>(s, x_int, y_int, s_key, neighbors);
      return;
    }
    const bool x_plus = (x_int + 1 < kMapWidthInt);
    const bool x_minus = (x_int > 0);
    const bool y_plus = (y_int + 1 < kMapHeightInt);
    const bool y_minus = (y_int > 0);
    if (x_plus) {
      // +1 x.
      neighbors->push_back(s_key + 1);
      // +1 x, +1 y.
      if (y_plus) {
        neighbors->push_back(s_key + kMapWidthInt + 1);
      }
      // +1 x, -1 y.
      if (y_minus) {
        neighbors->push_back(s_key - kMapWidthInt + 1);
      }
    }
    if (x_minus) {
      // -1 x.
      neighbors->push_back(s_key - 1);
      // -1 x, +1 y.
      if (y_plus) {
        neighbors->push_back(s_key + kMapWidthInt - 1);
      }
      // -1 x, -1 y.
      if (y_minus) {
        neighbors->push_back(s_key - kMapWidthInt - 1);
      }
    }
    if (y_plus) neighbors->push_back(s_key + kMapWidthInt);
    if (y_minus) neighbors->push_back(s_key - kMapWidthInt);
  };

  void SetOffset(const Eigen::Vector2f& loc) {
    offset_ = Eigen::Vector2f(0, 0);
    offset_ = loc -(KeyToState(StateToKey(loc)));
  }

  const float kResolution;
  const uint64_t kMapWidthInt;
  const uint64_t kMapHeightInt;
  const float kDiagonalLength;
  const Eigen::Vector2f kMapOrigin;
  // Vector map to use for obstacle checking.
  vector_map::VectorMap vector_map_;
  const float kMargin;
  Eigen::Vector2f offset_;
};


}  // namespace navigation
#endif  // EIGHT_CONNECTED_DOMAIN_H
