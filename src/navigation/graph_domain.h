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
#include "math/geometry.h"
#include "math/line2d.h"
#include "math/math_util.h"
#include "ros/ros_helpers.h"
#include "util/helpers.h"
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

  // V2 Map Edge structure.
  struct NavigationEdge {
    geometry::Line2f edge;
    uint64_t s0_id;
    uint64_t s1_id;
    float max_speed;
    float max_clearance;
  };


  GraphDomain() {}

  explicit GraphDomain(const std::string& map_file) {
    // Load graph from file.
    LoadV2(map_file);
  }

  State KeyToState(uint64_t key) const {
    DCHECK_LT(key, states.size());
    return states[key];
  }

  uint64_t StateToKey(const State& s) const {
    return s.id;
  }

  // Return the edge cost, assuming the two states are indeed connectable.
  float EdgeCost(const State& s0, const State& s1) const {
    return (s0.loc - s1.loc).norm();
  }

  float EdgeCost(const uint64_t k_s1, const uint64_t k_s2) const {
    DCHECK_LT(k_s1, states.size());
    DCHECK_LT(k_s2, states.size());
    return EdgeCost(states[k_s1], states[k_s2]);
  }

  float Heuristic(const State& s0, const State& s1) const {
    return (s0.loc - s1.loc).norm();
  }

  float Heuristic(const uint64_t k_s1, const uint64_t k_s2) const {
    DCHECK_LT(k_s1, states.size());
    DCHECK_LT(k_s2, states.size());
    return Heuristic(states[k_s1], states[k_s2]);
  }

  // Get neighbors to a state.
  void GetNeighborsKeys(uint64_t s_key,
                        std::vector<uint64_t>* state_neighbors) const {
    CHECK_LT(s_key, states.size());
    state_neighbors->clear();
    for (const NavigationEdge& e : edges) {
      if (e.s0_id == s_key) state_neighbors->push_back(e.s1_id);
      if (e.s1_id == s_key) state_neighbors->push_back(e.s0_id);
    }
  }

  void GrowIfNeeded(uint64_t id) {
    if (states.size() <= id) {
      states.resize(id + 1, State(0, 0, 0));
    }
  }

  void ResetDynamicStates() {
    states = static_states;
    edges = static_edges;
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

  NavigationEdge GetClosestEdge(const Eigen::Vector2f& v) const {
    NavigationEdge closest_edge;
    closest_edge.edge.p0 = closest_edge.edge.p1 = 
        Eigen::Vector2f(FLT_MAX, FLT_MAX);
    closest_edge.s0_id = closest_edge.s1_id = 0;
    closest_edge.max_clearance = closest_edge.max_speed = 0;
    if (edges.empty()) return closest_edge;
    float closest_dist = FLT_MAX;
    for (const NavigationEdge& e : edges) {
      const float dist = e.edge.Distance(v);
      if (dist < closest_dist) {
        closest_dist = dist;
        closest_edge = e;
      }
    }
    return closest_edge;
  }

  float GetClosestEdge(const Eigen::Vector2f& v,
                       uint64_t* p0,
                       uint64_t* p1) const {
    float closest_dist = FLT_MAX;
    if (edges.empty()) return closest_dist;
    NavigationEdge closest_edge = edges[0];
    for (const NavigationEdge& e : edges) {
      const float dist = e.edge.Distance(v);
      if (dist < closest_dist) {
        closest_dist = dist;
        closest_edge = e;
      }
    }
    *p0 = closest_edge.s0_id;
    *p1 = closest_edge.s1_id;
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
    for (int i = edges.size() - 1; i >= 0; --i) {
      if (edges[i].s0_id == s_id || edges[i].s1_id == s_id) {
        edges.erase(edges.begin() + i);
      }
    }
    // Renumber state IDs in the edges list.
    for (NavigationEdge& e : edges) {
      if (e.s0_id > s_id) --e.s0_id;
      if (e.s1_id > s_id) --e.s1_id;
    }
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

  void DeleteUndirectedEdge(const uint64_t s0, const uint64_t s1) {
    // This edge must exist, hence the list can't be empty.
    CHECK_GT(edges.size(), 0);
    for (int i = edges.size() - 1; i >= 0; --i) {
      if (edges[i].s0_id == s0 && 
          edges[i].s1_id == s1) {
        edges.erase(edges.begin() + i);
      }
    }
  }

  void AddUndirectedEdge(const uint64_t s0, 
                         const uint64_t s1,
                         const float max_speed,
                         const float max_clearance) {
    CHECK_LT(s0, states.size());
    CHECK_LT(s1, states.size());
    NavigationEdge e;
    e.s0_id = s0;
    e.s1_id = s1;
    e.max_speed = max_speed;
    e.max_clearance = max_clearance;
    e.edge.p0 = states[s0].loc;
    e.edge.p1 = states[s1].loc;
    edges.push_back(e);
  }

  uint64_t AddDynamicState(const Eigen::Vector2f& v) {
    static const bool kDebug = false;
    CHECK(!states.empty());
    // Find the closest Edge.
    const NavigationEdge e = GetClosestEdge(v);
    const uint64_t p0_id = e.s0_id, p1_id = e.s1_id;
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
    AddUndirectedEdge(p0_id, pmid_id, e.max_speed, e.max_clearance);

    // Add p1 : pmid
    AddUndirectedEdge(p1_id, pmid_id, e.max_speed, e.max_clearance);

    // Delete p0 : p1, since there is a p0 : pmid : p1 pathway now.
    DeleteUndirectedEdge(p0_id, p1_id);

    // Add pmid : v
    const uint64_t v_id = AddState(v);
    AddUndirectedEdge(pmid_id, v_id, e.max_speed, e.max_clearance);

    if (kDebug) {
      printf("Adding dynamic state %f,%f (%lu) %lu %lu %lu\n",
          v.x(), v.y(), v_id, p0_id, p1_id, pmid_id);
    }
    return v_id;
  }

  // Save a V2 Map from V2 map structures.
  bool SaveV2(const std::string& file) const {
    ScopedFile fid(file, "w", true);
    fprintf(fid(), "%lu\n", states.size());
    fprintf(fid(), "%lu\n", edges.size());
    for (const State& s : states) {
      fprintf(fid(), "%lu, %f, %f\n", s.id, s.loc.x(), s.loc.y());
    }
    for(const NavigationEdge& e : edges) {
      fprintf(fid(), "%lu, %lu, %f, %f\n", 
          e.s0_id, e.s1_id, e.max_speed, e.max_clearance);
    }
    return true;
  }

  // Load from a V2 map file.
  bool LoadV2(const std::string& file) {
    ScopedFile fid(file, "r", true);
    CHECK_NOTNULL(fid());
    printf("Loading %s\n", file.c_str());
    uint64_t num_states = 0, num_edges = 0;
    CHECK_EQ(fscanf(fid(), "%lu", &num_states), 1);
    CHECK_EQ(fscanf(fid(), "%lu", &num_edges), 1);
    states.resize(num_states);
    for (State& s : states) {
      CHECK_EQ(fscanf(fid(), "%lu, %f, %f", 
          &(s.id), &(s.loc.x()), &(s.loc.y())), 3);
    }
    for (uint64_t i = 0; i < num_edges; ++i) {
      uint64_t s0 = 0, s1 = 0;
      float max_speed = 0, max_clearance = 0;
      CHECK_EQ(fscanf(fid(), "%lu, %lu, %f, %f", 
          &(s0), &(s1), &(max_speed), &(max_clearance)), 4);
      AddUndirectedEdge(s0, s1, max_speed, max_clearance);
    }
    printf("Loaded %s with %lu vertices, %lu edges\n", 
        file.c_str(), num_states, num_edges);

    static_states = states;
    static_edges = edges;
    return true;
  }

  uint64_t GetClosestVertex(const Eigen::Vector2f& p) const {
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

  void GetClearanceAndSpeedFromLoc(const Eigen::Vector2f& p, 
                                   float* clearance, 
                                   float* speed) const {
    if (edges.empty()) return;
    const NavigationEdge closest_edge = GetClosestEdge(p);
    if (clearance) *clearance = closest_edge.max_clearance;
    if (speed) *speed = closest_edge.max_speed;
  }

  std::vector<State> states;
  std::vector<State> static_states;
  // V2 Map edges.
  std::vector<NavigationEdge> edges;
  std::vector<NavigationEdge> static_edges;
};


}  // namespace navigation
#endif  // GRAPH_DOMAIN_H
