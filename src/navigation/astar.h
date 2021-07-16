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

// Project headers.
#include "shared/math/math_util.h"
#include "simple_queue.h"

#ifndef A_STAR_H
#define A_STAR_H

namespace navigation {

// Struct to keep track of node priorities in A*, with tie-breaking to
// prefer expansion of nodes farther along the path.
struct AStarPriority {
  AStarPriority() {}

  AStarPriority(float g, float h) : g(g), h(h) {}

  // Returns true iff this has lower priority than the other.
  // Note that higher cost => lower priority.
  bool operator<(const AStarPriority& other) const {
    const float f = g + h;
    const float f_other = other.g + other.h;
    // This has lower priority if its total cost is higher.
    if (f > f_other + kEpsilon) return true;
    if (f < f_other - kEpsilon) return false;
    // Tie-breaking when costs are the same:
    // This has lower priority if its g-value is lower.
    return (g < other.g);
  }

  // Returns true iff this has higher priority than the other.
  // Note that lower cost => higher priority.
  bool operator>(const AStarPriority& other) const {
    const float f = g + h;
    const float f_other = other.g + other.h;
    // This has higher priority if its total cost is lower.
    if (f < f_other - kEpsilon) return true;
    if (f > f_other + kEpsilon) return false;
    // Tie-breaking when costs are the same:
    // This has higher priority if its g-value is higher.
    return (g > other.g);
  }

  // Epsilon for float comparisons.
  static constexpr float kEpsilon = 1e-3;
  // Cost to go: cost from start to this node.
  float g;
  // Heuristic: estimated cost from this node to the goal.
  float h;
};

template <class Domain>
struct NodeHash {
  explicit NodeHash(const Domain& domain) : domain(domain) {}
  uint64_t operator()(const typename Domain::State& s) const {
    return domain.StateToKey(s);
  }
  const Domain& domain;
};

template <class Domain, class Visualizer>
bool AStar(const typename Domain::State& start,
           const typename Domain::State& goal,
           const Domain& domain,
           Visualizer* const viz,
           std::vector<typename Domain::State>* path) {
  static CumulativeFunctionTimer function_timer_(__FUNCTION__);
  CumulativeFunctionTimer::Invocation invoke(&function_timer_);
  static const uint64_t kMaxEdgeExpansions = 1000;
  static const bool kDebug = false;
  std::unordered_map<uint64_t, uint64_t> parent_map_;
  // G-values of nodes in the open and closed list.
  std::unordered_map<uint64_t, float> g_values_;
  // Closed set (nodes with optimal costs).
  std::unordered_set<uint64_t> closed_set_;
  // Initialize an empty priority queue.
  SimpleQueue<uint64_t, AStarPriority> queue;
  const uint64_t k_start = domain.StateToKey(start);
  const uint64_t k_goal = domain.StateToKey(goal);
  if (kDebug) printf("A* plan from %lu to %lu\n", k_start, k_goal);
  // Add start to priority queue.
  queue.Push(k_start, AStarPriority(0, domain.Heuristic(start, goal)));
  g_values_[k_start] = 0;
  // Clear the closed set.
  closed_set_.clear();

  const double t_start = GetMonotonicTime();

  // Re-use neighbors vector to limit memory re-allocations.
  std::vector<uint64_t> neighbors;
  uint64_t edge_expansions = 0;
  // While priority queue is non-empty:
  while (edge_expansions < kMaxEdgeExpansions && !queue.Empty()) {
    ++edge_expansions;
    // Get the node with the highest priority.
    const uint64_t k_current = queue.Pop();
    if (kDebug) {
      printf("Add to closed: %5lu  g:%8.3f h:%8.3f\n",
             k_current,
             g_values_[k_current],
             domain.Heuristic(k_current, k_goal));
    }
    closed_set_.insert(k_current);
    // Get all neighbors.
    domain.GetNeighborsKeys(k_current, &neighbors);
    // For all neighbors:
    for (const uint64_t& k_next : neighbors) {
      // If not on closed list:
      if (kDebug) printf("     neighbor: %5lu", k_next);
      if (closed_set_.find(k_next) == closed_set_.end()) {
        // Compute tentative g value.
        const float g =
            g_values_[k_current] + domain.EdgeCost(k_next, k_current);
        // Compute heuristic for the next node.
        const float h = domain.Heuristic(k_next, k_goal);
        if (kDebug) printf("  g:%8.3f h:%8.3f", g, h);
        if (!queue.Exists(k_next) || g_values_[k_next] > g) {
          // This node does not exist on the queue, or it currently has a
          // higher g-value: add to queue, or update it.
          parent_map_[k_next] = k_current;
          g_values_[k_next] = g;
          queue.Push(k_next, AStarPriority(g, h));
          viz->DrawEdge(domain.KeyToState(k_current),
                        domain.KeyToState(k_next));
          if (kDebug) printf(" Push to open list");
        }
        if (kDebug) printf("\n");
      } else {
        if (kDebug) printf("  already closed\n");
      }
    }
    // If goal has a parent:
    if (parent_map_.find(k_goal) != parent_map_.end()) {
      const double t_end = GetMonotonicTime();
      if (kDebug) printf("Path found in %f seconds.\n", t_end - t_start);
      // We're done. Extract path, and return.
      path->clear();
      uint64_t current = k_goal;
      do {
        path->push_back(domain.KeyToState(current));
        CHECK(parent_map_.find(current) != parent_map_.end());
        current = parent_map_[current];
      } while (current != k_start);
      path->push_back(domain.KeyToState(k_start));
      return true;
    }
  }
  const double t_end = GetMonotonicTime();
  printf("No path found, took %f seconds.\n", t_end - t_start);
  // Priority queue is exhausted, but path not found. No path exists.
  return false;
}


}  // namespace navigation
#endif  // A_STAR_H
