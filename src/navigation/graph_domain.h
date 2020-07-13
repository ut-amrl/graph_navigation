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
\file    graph_domain.h
\brief   Domain definition for A* planner to use graphs loaded from files.
\author  Joydeep Biswas, (C) 2019
*/
//========================================================================

// C headers.
#include <inttypes.h>

// C++ headers.
#include <algorithm>
#include <string>
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
#include "shared/util/helpers.h"
#include "vector_map/vector_map.h"

#ifndef GRAPH_DOMAIN_H
#define GRAPH_DOMAIN_H

namespace navigation {

struct GraphDomain {
  struct State {
    uint64_t id;
    Eigen::Vector2f loc;
    State(uint64_t id, float x, float y) : id(id), loc(x, y) {}
    State(uint64_t id, const Eigen::Vector2f loc) : id(id), loc(loc) {}
    State() {}
  };

  GraphDomain() {}

  explicit GraphDomain(const std::string& map_file) {
    // Load graph from file.
    Load(map_file);
  }

  State KeyToState(uint64_t key) const {
    DCHECK_LT(key, states.size());
    return states[key];
  }

  uint64_t StateToKey(const State& s) const {
    return s.id;
  }

  // Return the edge cost, assuming the two states are indeed connectable.
  float EdgeCost(const State& s1, const State& s2) const {
    return (s1.loc - s2.loc).norm();
  }

  float EdgeCost(const uint64_t k_s1, const uint64_t k_s2) const {
    DCHECK_LT(k_s1, states.size());
    DCHECK_LT(k_s2, states.size());
    return EdgeCost(states[k_s1], states[k_s2]);
  }

  float Heuristic(const State& s1, const State& s2) const {
    return (s1.loc - s2.loc).norm();
  }

  float Heuristic(const uint64_t k_s1, const uint64_t k_s2) const {
    DCHECK_LT(k_s1, states.size());
    DCHECK_LT(k_s2, states.size());
    return Heuristic(states[k_s1], states[k_s2]);
  }

  void GetNeighbors(const State& s,
                    std::vector<State>* neighbors) const {
    printf("%s unimplemented\n", __PRETTY_FUNCTION__);
  }

  // Get neighbors to a state.
  void GetNeighborsKeys(uint64_t s_key,
                        std::vector<uint64_t>* state_neighbors) const {
    CHECK_LT(s_key, states.size());
    CHECK_LT(s_key, neighbors.size());
    state_neighbors->clear();
    state_neighbors->insert(state_neighbors->end(),
                            neighbors[s_key].begin(),
                            neighbors[s_key].end());
  }

  void GrowIfNeeded(uint64_t id) {
    if (states.size() <= id) {
      states.resize(id + 1, State(0, 0, 0));
      neighbors.resize(id + 1);
    }
  }

  void ResetDynamicStates() {
    states = static_states;
    neighbors = static_neighbors;
  }

  uint64_t GetClosestState(const Eigen::Vector2f& v) {
    uint64_t closest_state = 0;
    float closest_sq_dist = FLT_MAX;
    for (size_t i = 0; i < states.size(); ++i) {
      const State& s = states[i];
      const float sq_dist = (s.loc - v).squaredNorm();
      if (sq_dist < closest_sq_dist) {
        closest_state = i;
        closest_sq_dist = sq_dist;
      }
    }
    return closest_state;
  }

  float GetClosestEdge(const Eigen::Vector2f& v,
                       uint64_t* p0_id_ptr,
                       uint64_t* p1_id_ptr) {
    uint64_t& p0 = *p0_id_ptr;
    uint64_t& p1 = *p1_id_ptr;
    float closest_dist = FLT_MAX;
    for (const State& s : states) {
      for (const uint64_t n : neighbors[s.id]) {
        const Eigen::Vector2f& a = s.loc;
        const Eigen::Vector2f& b = states[n].loc;
        const float dist = geometry::DistanceFromLineSegment(v, a, b);
        if (dist < closest_dist) {
          p0 = s.id;
          p1 = n;
          closest_dist = dist;
        }
      }
    }
    return closest_dist;
  }

  uint64_t AddState(const Eigen::Vector2f& v) {
    State s(states.size(), v);
    GrowIfNeeded(s.id);
    states[s.id] = s;
    return s.id;
  }

  void DeleteState(const uint64_t s_id) {
    // Delete all edges that touch this state.
    for (std::vector<uint64_t>& n : neighbors) {
      for (size_t i = 0; i < n.size(); ++i) {
        if (n[i] == s_id) {
          n.erase(n.begin() + i);
          --i;
        } else if (n[i] > s_id) {
          // Renumber states after this state.
          --n[i];
        }
      }
    }
    // Remove the adjacency row for this state.
    neighbors.erase(neighbors.begin() + s_id);
    // Remove this state.
    for (size_t i = 0; i < states.size(); ++i) {
      if (states[i].id == s_id) {
        states.erase(states.begin() + i);
        --i;
      } else if (states[i].id > s_id) {
        // Renumber states after this state.
        --states[i].id;
      }
    }
  }

  void DeleteUndirectedEdge(const uint64_t s1, const uint64_t s2) {
    for (uint64_t s : {s1, s2}) {
      std::vector<uint64_t>& n = neighbors[s];
      for (size_t j = 0; j < n.size(); ++j) {
        if ((s == s1 && n[j] == s2) || (s == s2 && n[j] == s1)) {
          n.erase(n.begin() + j);
        }
      }
    }
  }

  bool NeighborExists(const std::vector<uint64_t>& neighbors,
                      const uint64_t s) {
    return (std::find(neighbors.begin(), neighbors.end(), s)
        != neighbors.end());
  }
  void AddDirectedEdge(const uint64_t s1, const uint64_t s2) {
    GrowIfNeeded(s1);
    GrowIfNeeded(s2);
    if (!NeighborExists(neighbors[s1], s2)) {
      neighbors[s1].push_back(s2);
    }
  }

  void AddUndirectedEdge(const uint64_t s1, const uint64_t s2) {
    GrowIfNeeded(s1);
    GrowIfNeeded(s2);
    if (!NeighborExists(neighbors[s1], s2)) {
      neighbors[s1].push_back(s2);
    }
    if (!NeighborExists(neighbors[s2], s1)) {
      neighbors[s2].push_back(s1);
    }
  }

  uint64_t AddDynamicState(const Eigen::Vector2f& v) {
    static const bool kDebug = true;
    CHECK(!states.empty());
    // Find the closest Edge.
    uint64_t p0_id = 0, p1_id = 1;
    GetClosestEdge(v, &p0_id, &p1_id);
    const Eigen::Vector2f& p0 = states[p0_id].loc;
    const Eigen::Vector2f& p1 = states[p1_id].loc;
    if (kDebug) {
      printf("Adding %f,%f\n", v.x(), v.y());
      printf("p0: %lu (%f,%f)\n", p0_id, p0.x(), p0.y());
      printf("p1: %lu (%f,%f)\n", p1_id, p1.x(), p1.y());
    }
    // Find the projection of v on to the line segment p0 : p1;
    const Eigen::Vector2f pmid =
        geometry::ProjectPointOntoLineSegment(v, p0, p1);

    // Add p0 : pmid
    const uint64_t pmid_id = AddState(pmid);
    AddUndirectedEdge(p0_id, pmid_id);

    // Add p1 : pmid
    AddUndirectedEdge(p1_id, pmid_id);

    // Delete p0 : p1, since there is a p0 : pmid : p1 pathway now.
    DeleteUndirectedEdge(p0_id, p1_id);

    // Add pmid : v
    const uint64_t v_id = AddState(v);
    AddUndirectedEdge(pmid_id, v_id);

    printf("Adding dynamic state %f,%f (%lu) %lu %lu %lu\n",
        v.x(), v.y(), v_id, p0_id, p1_id, pmid_id);
    return v_id;
  }

  bool Save(const std::string& file) {
    fprintf(stderr, "Saving nav map not implemented.\n");
    return false;
  }

  uint64_t GetClosestVertex(const Eigen::Vector2f& p) {
    float best_sq_dist = FLT_MAX;
    uint64_t best = 0;
    for (const State& s : states) {
      if ((s.loc - p).squaredNorm() <  best_sq_dist) {
        best_sq_dist = (s.loc - p).squaredNorm();
        best = s.id;
      }
    }
    return best;
  }

  void Load(const std::string& file) {
    static const bool kDebug = true;
    ScopedFile fid(file, "r", true);
    CHECK_NOTNULL(fid());
    bool valid = true;
    uint64_t id = 0;
    float x = 0, y = 0;
    int num_neighbors = 0;
    int num_edges = 0;
    states.clear();
    neighbors.clear();
    auto drawmap = [&]() {
      printf("Map:\n======\n");
      for (const State s : states) {
        printf("%4lu: %8.3f,%8.3f", s.id, s.loc.x(), s.loc.y());
        CHECK_GT(neighbors.size(), s.id);
        for (const uint64_t n : neighbors[s.id]) {
          printf(" %4lu", n);
        }
        printf("\n");
      }
      printf("Map:\n======\n");
    };
    while (valid &&
        fscanf(fid(), "%lu, %f, %f, %d", &id, &x, &y, &num_neighbors) == 4) {
      GrowIfNeeded(id);
      states[id] = State(id, x, y);
      if (kDebug) {
        printf("Node %lu (%f,%f), with %d neighbors:\n",
               id,
               x,
               y,
               num_neighbors);
      }
      for (int i = 0; i < num_neighbors; ++i) {
        uint64_t n = 0;
        if (fscanf(fid(), ", %lu", &n) == 1) {
          if (kDebug) printf("%lu -> %lu\n", id, n);
          AddUndirectedEdge(id, n);
          ++num_edges;
        } else {
          if (kDebug) printf("\n");
          valid = false;
          break;
        }
      }
      // drawmap();
    }
    printf("Loaded %s with %d states, %d edges\n",
            file.c_str(),
            static_cast<int>(states.size()),
            num_edges);
    static_states = states;
    static_neighbors = neighbors;
    if (kDebug) drawmap();
  }

  std::vector<State> states;
  std::vector<std::vector<uint64_t> > neighbors;
  std::vector<State> static_states;
  std::vector<std::vector<uint64_t> > static_neighbors;
};


}  // namespace navigation
#endif  // GRAPH_DOMAIN_H
