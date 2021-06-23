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

// NOTE: This file is an exact copy of `src/navigation_map/navigation_map.h` in `https://github.com/ut-amrl/vector_display`. This should be kept in sync.

// C headers.
#include <inttypes.h>

// C++ headers.
#include <algorithm>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <array>
#include <fstream>

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
#include "navigation_parameters.h"

#include "nlohmann/json.hpp"
using json = nlohmann::json;

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

    json toJSON() const {
      json data;
      data["id"] = id;
      data["loc"]["x"] = loc.x();
      data["loc"]["y"] = loc.y();
      return data;
    }

    static State fromJSON(const json& j) {
      State s;
      s.id = j["id"].get<uint64_t>();
      s.loc = Eigen::Vector2f(j["loc"]["x"].get<float>(), j["loc"]["y"].get<float>());
      return s;
    }
  };

  // V2 Map Edge structure.
  struct NavigationEdge {
    NavigationEdge() {
      for (size_t i = 0; i < 2; i++) {
        for (size_t j = 0; j < failure_log_odds_prior[i].size(); j++) {
          failure_log_odds_prior[i][j] = 
              std::log(failure_belief[i][j] / (1 - failure_belief[i][j]));
          failure_log_odds[i][j] = failure_log_odds_prior[i][j];
        }
      }
    }

    geometry::Line2f edge;
    uint64_t s0_id;
    uint64_t s1_id;
    float max_speed = 50.0f;
    float max_clearance = 10.0f;
    bool has_door;
    bool has_stairs;

    // TODO(srabiee): Add support for variable number of failure types
    const static int kFailureTypeCount = 2;
    // Traversal counts for forward direction (0) and reverse direction (1)
    std::array<uint64_t, 2> traversal_count = {0, 0};
    std::array<uint64_t, kFailureTypeCount> failure_count_fwd{0, 0};
    std::array<uint64_t, kFailureTypeCount> failure_count_rev{0, 0};

    // Failure belief for catastrophic failure (0), non-catastrophic_failure (1)
    // and no failures (2). The first dimension represents the direction of
    // traversal: 0: fwd and 1: rev
    std::array<std::array<float, kFailureTypeCount + 1>, 2> failure_belief = {{{
        0.0001, 0.0001, 0.9998}, {0.0001, 0.0001, 0.9998}}};
    std::array<std::array<float, kFailureTypeCount + 1>, 2> failure_belief_normalized = {{{
        0.0001, 0.0001, 0.9998}, { 0.0001, 0.0001, 0.9998}}};


    std::array<std::array<float, kFailureTypeCount + 1>, 2>
                                  failure_log_odds;
    std::array<std::array<float, kFailureTypeCount + 1>, 2>
                                  failure_log_odds_prior;


    json toJSON() const {
      json data;
      data["s0_id"] = s0_id;
      data["s1_id"] = s1_id;
      data["max_speed"] = max_speed;
      data["max_clearance"] = max_clearance;
      data["has_door"] = has_door;
      data["has_stairs"] = has_stairs;
      // no need to save the edge, it's redundant data that can be recovered at load time
      return data;
    }

    json TraversalInfoToJSON() const {
      json data;
      json traversal_count_json(traversal_count);
      json failure_count_fwd_json(failure_count_fwd);
      json failure_count_rev_json(failure_count_rev);
      json failure_belief_fwd_json(failure_belief[0]);
      json failure_belief_rev_json(failure_belief[1]);
      json failure_log_odds_prior_fwd_json(failure_log_odds_prior[0]);
      json failure_log_odds_prior_rev_json(failure_log_odds_prior[1]);
      data["traversal_count"] = traversal_count_json;
      data["failure_count_fwd"] = failure_count_fwd_json;
      data["failure_count_rev"] = failure_count_rev_json;
      data["failure_belief_fwd"] = failure_belief_fwd_json;
      data["failure_belief_rev"] = failure_belief_rev_json;
      data["failure_log_odds_prior_fwd"] = failure_log_odds_prior_fwd_json;
      data["failure_log_odds_prior_rev"] = failure_log_odds_prior_rev_json;
      data["s0_id"] = s0_id;
      data["s1_id"] = s1_id;


      return data;
    }

    void UpdateTraversalInfoFromJSON(const json& json_obj) {
      CHECK(json_obj["traversal_count"].is_array());
      CHECK(json_obj["failure_count_fwd"].is_array());
      CHECK(json_obj["failure_count_rev"].is_array());
      CHECK(json_obj["failure_belief_fwd"].is_array());
      CHECK(json_obj["failure_belief_rev"].is_array());
      CHECK(json_obj["failure_log_odds_prior_fwd"].is_array());
      CHECK(json_obj["failure_log_odds_prior_rev"].is_array());
      CHECK_EQ(json_obj["traversal_count"].size(), traversal_count.size());
      CHECK_EQ(json_obj["failure_count_fwd"].size(), failure_count_fwd.size());
      CHECK_EQ(json_obj["failure_count_rev"].size(), failure_count_rev.size());
      CHECK_EQ(json_obj["failure_belief_fwd"].size(), failure_belief[0].size());
      CHECK_EQ(json_obj["failure_belief_rev"].size(), failure_belief[1].size());
      CHECK_EQ(json_obj["failure_log_odds_prior_fwd"].size(),
               failure_log_odds_prior[0].size());
      CHECK_EQ(json_obj["failure_log_odds_prior_rev"].size(), 
               failure_log_odds_prior[1].size());

      for (size_t i = 0; i < traversal_count.size(); i++) {
        traversal_count[i] = json_obj["traversal_count"][i].get<uint64_t>();
      }

      for (size_t i = 0; i < failure_count_fwd.size(); i++) {
        failure_count_fwd[i] = json_obj["failure_count_fwd"][i].get<uint64_t>();
      }

      for (size_t i = 0; i < failure_count_rev.size(); i++) {
        failure_count_rev[i] = json_obj["failure_count_rev"][i].get<uint64_t>();
      }

      for (size_t i = 0; i < failure_belief[0].size(); i++) {
        failure_belief[0][i] = json_obj["failure_belief_fwd"][i].get<float>();
      }

      for (size_t i = 0; i < failure_belief[1].size(); i++) {
        failure_belief[1][i] = json_obj["failure_belief_rev"][i].get<float>();
      }

      for (size_t i = 0; i < failure_log_odds_prior[0].size(); i++) {
        failure_log_odds_prior[0][i] = json_obj["failure_log_odds_prior_fwd"][i].get<float>();
      }

      for (size_t i = 0; i < failure_log_odds_prior[1].size(); i++) {
        failure_log_odds_prior[1][i] = json_obj["failure_log_odds_prior_rev"][i].get<float>();
      }

      // Compute the failure_log_odds
      for (size_t i = 0; i < 2; i++) {
        for (size_t j = 0; j < failure_log_odds_prior[i].size(); j++) {
          failure_log_odds[i][j] = 
              std::log(failure_belief[i][j] / (1 - failure_belief[i][j]));
        }
      }

      // Compute the normalized failure belief
      for (size_t i = 0; i < 2; i++) {
        float sum = 0.0;
        for (size_t j = 0; j < failure_belief[i].size(); j++) {
          sum += failure_belief[i][j];
        }
        for (size_t j = 0; j < failure_belief_normalized[i].size(); j++) {
          failure_belief_normalized[i][j] = failure_belief[i][j] / sum;
        }
      }
    }

    // Constructing a navigation edge from JSON assumes you already know the states that exist in the world
    static NavigationEdge fromJSON(const json& j, const std::vector<State>& states) {
      NavigationEdge e;
      e.s0_id = j["s0_id"].get<uint64_t>();
      e.s1_id = j["s1_id"].get<uint64_t>();
      e.max_speed = j["max_speed"].get<float>();
      e.max_clearance = j["max_clearance"].get<float>();

      e.has_door = j["has_door"].get<bool>();
      e.has_stairs = j["has_stairs"].get<bool>();

      e.edge.p0 = states[e.s0_id].loc;
      e.edge.p1 = states[e.s1_id].loc;
      return e;
    }

    // Update the belief over different failure states given the observation
    // being the raw images. The "observation" argument here is actually
    // the inverse measurement likelihood for each type of failure given the
    // captured raw images. The value is provided by introspective perception.
    void UpdateFailureBelief(
        int direction,
        const std::array<float, kFailureTypeCount + 1>& observation) {
      const bool kDebug = false;
      CHECK_LT(direction, 2);
      for (size_t j = 0; j < observation.size(); j++) {
        failure_log_odds[direction][j] =
            std::log(observation[j] / (1 - observation[j])) +
            failure_log_odds[direction][j] -
            failure_log_odds_prior[direction][j];
      }

      if (kDebug) {
        std::cout << "Before belief update: " << std::endl;
        std::cout << s0_id << "-> " << s1_id << ": " << traversal_count[0]
                  << std::endl;
        std::cout << "Fwd fail " << failure_belief[0][0] << ", "
                  << failure_belief[0][1] << std::endl;
        std::cout << "Rev fail " << failure_belief[1][0] << ", "
                  << failure_belief[1][1] << std::endl;
        std::cout << std::endl;
      }

      float sum = 0.0;
      for (size_t j = 0; j < failure_belief[direction].size(); j++) {
        failure_belief[direction][j] =
            1 - 1 / (1 + exp(failure_log_odds[direction][j]));
        sum += failure_belief[direction][j];
      }
      for (size_t j = 0; j < failure_belief_normalized[direction].size(); j++) {
        failure_belief_normalized[direction][j] = failure_belief[direction][j] 
                                                  / sum;
      }

      if (kDebug) {
        std::cout << "After belief update: " << std::endl;
        std::cout << s0_id << "-> " << s1_id << ": " << traversal_count[0]
                  << std::endl;
        std::cout << "Fwd fail " << failure_belief[0][0] << ", "
                  << failure_belief[0][1] << std::endl;
        std::cout << "Rev fail " << failure_belief[1][0] << ", "
                  << failure_belief[1][1] << std::endl;
        std::cout << std::endl << std::endl;
      }
    }

    // Update the belief over different failure states given the observation
    // being the experienced failure/ no-failure after traversing the edge.
    // The observation is represented as a one-hot vector.
    void UpdateFailureBelief(
        int direction,
        const std::array<bool, kFailureTypeCount + 1>& observation) {
      const bool kDebug = false;
      // P(failure | failure_experienced)
      float kDelta = 0.96;
      if (observation.back()) {
        // P(No_failure | successful_traversal)
        kDelta = 0.9998;
      }
      const float kInvObsLikelihoodHit = kDelta;
      const float kInvObsLikelihoodMiss = (1 - kDelta) / (kFailureTypeCount);
      CHECK_LT(direction, 2);
      for (size_t j = 0; j < observation.size(); j++) {
        if (observation[j] == true) {
          failure_log_odds[direction][j] =
              std::log(kInvObsLikelihoodHit / (1 - kInvObsLikelihoodHit)) +
              failure_log_odds[direction][j] -
              failure_log_odds_prior[direction][j];
        } else {
          failure_log_odds[direction][j] =
              std::log(kInvObsLikelihoodMiss / (1 - kInvObsLikelihoodMiss)) +
              failure_log_odds[direction][j] -
              failure_log_odds_prior[direction][j];
        }
      }

      if (kDebug) {
        std::cout << "Before belief update: " << std::endl;
        std::cout << s0_id << "-> " << s1_id << ": " << traversal_count[0]
                  << std::endl;
        std::cout << "Fwd fail " << failure_belief[0][0] << ", "
                  << failure_belief[0][1] << std::endl;
        std::cout << "Rev fail " << failure_belief[1][0] << ", "
                  << failure_belief[1][1] << std::endl;
        std::cout << std::endl;
      }

      float sum = 0.0;
      for (size_t j = 0; j < failure_belief[direction].size(); j++) {
        failure_belief[direction][j] =
            1 - 1 / (1 + exp(failure_log_odds[direction][j]));
        sum += failure_belief[direction][j];
      }
      for (size_t j = 0; j < failure_belief_normalized[direction].size(); j++) {
        failure_belief_normalized[direction][j] = failure_belief[direction][j] 
                                                  / sum;
      }

      if (kDebug) {
        std::cout << "After belief update: " << std::endl;
        std::cout << s0_id << "-> " << s1_id << ": " << traversal_count[0]
                  << std::endl;
        std::cout << "Fwd fail " << failure_belief[0][0] << ", "
                  << failure_belief[0][1] << std::endl;
        std::cout << "Rev fail " << failure_belief[1][0] << ", "
                  << failure_belief[1][1] << std::endl;
        std::cout << std::endl << std::endl;
      }
    }
  };

  GraphDomain() {
    this->params_ = new NavigationParameters();
  }

  explicit GraphDomain(const std::string& map_file, const navigation::NavigationParameters* params) {
    // Load graph from file.
    Load(map_file);
    this->params_ = params;
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
      if (e.s0_id == s_key && (!e.has_stairs || params_->can_traverse_stairs)) state_neighbors->push_back(e.s1_id);
      if (e.s1_id == s_key && (!e.has_stairs || params_->can_traverse_stairs)) state_neighbors->push_back(e.s0_id);
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

  bool GetClosestState(const Eigen::Vector2f& v, uint64_t* closest_state) {
    float closest_sq_dist = FLT_MAX;
    bool found = false;
    for (size_t i = 0; i < states.size(); ++i) {
      const State& s = states[i];
      const float sq_dist = (s.loc - v).squaredNorm();
      if (sq_dist < closest_sq_dist) {
        found = true;
        *closest_state = i;
        closest_sq_dist = sq_dist;
      }
    }
    return found;
  }

  bool GetClosestStaticState(const Eigen::Vector2f& v, 
                             uint64_t* closest_state,
                             float* distance_to_closest_state) {
    float closest_sq_dist = FLT_MAX;
    bool found = false;
    for (size_t i = 0; i < static_states.size(); ++i) {
      const State& s = static_states[i];
      const float sq_dist = (s.loc - v).squaredNorm();
      if (sq_dist < closest_sq_dist) {
        found = true;
        *closest_state = i;
        closest_sq_dist = sq_dist;
      }
    }

    *distance_to_closest_state = sqrt(closest_sq_dist);
    return found;
  }

  bool GetClosestEdge(const Eigen::Vector2f& v, NavigationEdge* closest_edge, float* closest_dist) const {
    bool found = false;
    closest_edge->s0_id = -1;
    closest_edge->s1_id = -1;
    if (edges.empty()) return closest_dist;
    for (const NavigationEdge& e : edges) {
      const float dist = e.edge.Distance(v);
      if (dist < *closest_dist && (!e.has_stairs || params_->can_traverse_stairs)) {
        *closest_dist = dist;
        *closest_edge = e;
        found = true;
      }
    }
    return found;
  }

  bool GetClosestEdge(const Eigen::Vector2f& v,
                      const std::vector<uint64_t> &ignore_node_ids,
                      NavigationEdge* closest_edge,
                      float* closest_dist,
                      bool* is_dynamic_edge) const {
    const bool kDebug = false;
    bool found = false;
    closest_edge->s0_id = -1;
    closest_edge->s1_id = -1;
    if (edges.empty()) return closest_dist;
    for (const NavigationEdge& e : edges) {
      const float dist = e.edge.Distance(v);
      if (dist < *closest_dist &&
          (!e.has_stairs || params_->can_traverse_stairs)) {
        bool skip_edge = false;
        for (const uint64_t& ignored_node_id : ignore_node_ids) {
          if (e.s0_id == ignored_node_id || e.s1_id == ignored_node_id) {
            skip_edge = true;
            break;
          }
        }
        if (skip_edge) {
          if (kDebug) {
            std::cout << "Skipping edge " << e.s0_id << " to " << e.s1_id 
                      << std::endl;
            std::cout << "start goal ids: " << ignore_node_ids[0] << ", " 
                      << ignore_node_ids[1] << std::endl;
            std::cout << "*****************************************"
                      << std::endl 
                      << "*****************************************"
                      << std::endl;
          }
          continue;
        }

        *closest_dist = dist;
        *closest_edge = e;
        found = true;

        if (e.s0_id >= static_states.size() ||
            e.s1_id >= static_states.size()) {
          *is_dynamic_edge = true;
        } else {
          *is_dynamic_edge = false;
        }
      }
    }
    return found;
  }

  bool GetClosestStaticEdge(const Eigen::Vector2f& v,
                            NavigationEdge* closest_edge,
                            float* closest_dist) const {
    bool found;
    closest_edge->s0_id = -1;
    closest_edge->s1_id = -1;
    if (static_edges.empty()) return closest_dist;
    for (const NavigationEdge& e : static_edges) {
      const float dist = e.edge.Distance(v);
      if (dist < *closest_dist &&
          (!e.has_stairs || params_->can_traverse_stairs)) {
        *closest_dist = dist;
        *closest_edge = e;
        found = true;
      }
    }
    return found;
  }

  uint64_t AddState(const Eigen::Vector2f& v) {
    State s(states.size(), v);
    GrowIfNeeded(s.id);
    states[s.id] = s;
    return s.id;
  }

  void GetDynamicEdges(std::vector<NavigationEdge>* dynamic_edges) const {
    dynamic_edges->clear();
    for (auto& edge : edges) {
      if (edge.s0_id >= static_states.size() ||
          edge.s1_id >= static_states.size()) {
        dynamic_edges->push_back(edge);
      }
    }
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
      if ((edges[i].s0_id == s0 && 
          edges[i].s1_id == s1) || (edges[i].s0_id == s1 && 
          edges[i].s1_id == s0)) {
        edges.erase(edges.begin() + i);
      }
    }
  }

  void AddUndirectedEdge(const uint64_t s0, 
                         const uint64_t s1,
                         const float max_speed,
                         const float max_clearance,
                         const bool has_door,
                         const bool has_stairs) {
    CHECK_LT(s0, states.size());
    CHECK_LT(s1, states.size());
    NavigationEdge e;
    e.s0_id = s0;
    e.s1_id = s1;
    e.max_speed = max_speed;
    e.max_clearance = max_clearance;
    e.edge.p0 = states[s0].loc;
    e.edge.p1 = states[s1].loc;
    e.has_door = has_door;
    e.has_stairs = has_stairs;
    edges.push_back(e);
  }

  void AddUndirectedEdge(const json& edge) {
    uint64_t s0 = edge["s0_id"].get<uint64_t>();
    uint64_t s1 = edge["s1_id"].get<uint64_t>();
    CHECK_LT(s0, states.size());
    CHECK_LT(s1, states.size());
    edges.push_back(NavigationEdge::fromJSON(edge, states));
  }

  // This should only be called while upgrading v1 maps
  void AddUndirectedEdge(const uint64_t s0, const uint64_t s1) {
    GrowIfNeeded(s0);
    GrowIfNeeded(s1);
    NavigationEdge e;
    e.s0_id = s0;
    e.s1_id = s1;
    e.edge.p0 = states[s0].loc;
    e.edge.p1 = states[s1].loc;
    e.has_door = false;
    e.has_stairs = false;
    edges.push_back(e);
  }

  void UpdateEdgeTraversalStats(uint64_t s0_id,
                                uint64_t s1_id,
                                uint64_t traversal_count_diff,
                                const std::array<uint64_t, 2>& failure_count) {
    const bool kDebug = false;
    size_t edge_id = 0;
    int direction = 0;
    if (GetStaticEdgeID(s0_id, s1_id, &edge_id, &direction)) {
      static_edges[edge_id].traversal_count[direction] += traversal_count_diff;
      for (size_t i = 0; i < failure_count.size(); i++) {
        if (direction == 0) {
          static_edges[edge_id].failure_count_fwd[i] += failure_count[i];
        } else {
          static_edges[edge_id].failure_count_rev[i] += failure_count[i];
        }
      }

      for (size_t i = 0; i < failure_count.size(); i++) {
        for (size_t j = 0; j < failure_count[i]; j++) {
          std::array<bool, 3> observation = {false, false, false};
          observation[i] = true;
          static_edges[edge_id].UpdateFailureBelief(direction, observation);
        }
      }
    }

    if (kDebug) {
      // PrintEdgeTraversalStats();
      PrintFailureBelief();
    }
  }

  void UpdateEdgeTraversalStats(uint64_t s0_id,
                                uint64_t s1_id,
                                uint64_t traversal_count_diff) {
    const bool kDebug = false;
    size_t edge_id = 0;
    int direction = 0;
    if (GetStaticEdgeID(s0_id, s1_id, &edge_id, &direction)) {
      static_edges[edge_id].traversal_count[direction] += traversal_count_diff;

      // Update the belief over transitioning to failure states on this edge
      // given the reported successful traversals of the edge
      for (size_t i = 0; i < traversal_count_diff; i++) {
        std::array<bool, 3> observation = {false, false, true};
        static_edges[edge_id].UpdateFailureBelief(direction, observation);
      }
    }

    if (kDebug) {
      // PrintEdgeTraversalStats();
      PrintFailureBelief();
    }
  }

  void PrintEdgeTraversalStats() {
    for (size_t i = 0; i < static_edges.size(); i++) {
      std::cout << static_edges[i].s0_id << "-> " << static_edges[i].s1_id
          << ": " << static_edges[i].traversal_count[0] << ","
          << static_edges[i].traversal_count[1] << std::endl;
      std::cout << "Fwd fail " << static_edges[i].failure_count_fwd[0] << ", "
                << static_edges[i].failure_count_fwd[1] << std::endl;
      std::cout << "Rev fail " << static_edges[i].failure_count_rev[0] << ", "
                << static_edges[i].failure_count_rev[1] << std::endl;
      std::cout << std::endl;
    }
  }

  void PrintFailureBelief() {
    for (size_t i = 0; i < static_edges.size(); i++) {
      std::cout << static_edges[i].s0_id << "-> " << static_edges[i].s1_id
          << ": " << static_edges[i].traversal_count[0] << std::endl;
      std::cout << "Fwd fail " << static_edges[i].failure_belief[0][0] << ", "
                << static_edges[i].failure_belief[0][1] << std::endl;
      std::cout << "Rev fail " << static_edges[i].failure_belief[1][0] << ", "
                << static_edges[i].failure_belief[1][1] << std::endl;
      std::cout << std::endl;
    }
  }

  // Finds the id of the static edge with the given start and end node ids
  bool GetStaticEdgeID(uint64_t s0_id,
                       uint64_t s1_id,
                       size_t* edge_id,
                       int* direction) {
    for (size_t i = 0; i < static_edges.size(); i++) {
      if (static_edges[i].s0_id == s0_id && static_edges[i].s1_id == s1_id) {
        *edge_id = i;
        *direction = 0;
        return true;
      } else if (static_edges[i].s0_id == s1_id &&
                 static_edges[i].s1_id == s0_id) {
        *edge_id = i;
        *direction = 1;
        return true;
      }
    }
    // No such static edge was found
    return false;
  }

  uint64_t AddDynamicState(const Eigen::Vector2f& v,
                           bool enable_reduced_graph) {
    static const bool kDebug = false;
    CHECK(!states.empty());
    // Find the closest Edge.
    NavigationEdge closest_edge;
    float closest_dist = FLT_MAX;
    if (!GetClosestEdge(v, &closest_edge, &closest_dist)) {
      std::cerr << "Unable to find closest edge to point " << v << std::endl;
      return 0;
    }

    // If reduced graph is enabled and the new node location is close to a 
    // static node, connect it directly to the closest static node instead 
    // of projecting it to the closest edge. 
    if (enable_reduced_graph) {
      uint64_t closest_static_state = 0;
      float dist_to_closest_state = 0.0;
      // Max acceptable distance of the input location to the
      // closest static state in order to use the static node instead of
      // adding a new dynamic state
      const float kMaxDistanceToStaticState = 1.0;
      if (GetClosestStaticState(
              v, &closest_static_state, &dist_to_closest_state)) {
        if (dist_to_closest_state < kMaxDistanceToStaticState) {
          std::vector<uint64_t> state_neighbors;
          GetNeighborsKeys(closest_static_state, &state_neighbors);
          if (!state_neighbors.empty()) {
            const uint64_t v_id = AddState(v);
            AddUndirectedEdge(v_id,
                              closest_static_state,
                              closest_edge.max_speed,
                              closest_edge.max_clearance,
                              false,
                              false);

            return v_id;
          }
        }
      }
    }

    // Find the projection of v on to the line segment p0 : p1;
    const Eigen::Vector2f pmid =
        geometry::ProjectPointOntoLineSegment(v, closest_edge.edge.p0, closest_edge.edge.p1);

    // Add p0 : pmid
    const uint64_t pmid_id = AddState(pmid);
    const uint64_t p0_id = closest_edge.s0_id;
    const uint64_t p1_id = closest_edge.s1_id;
    
    // NOTE: Here, we are assuming that any dynamic edges added are free of stairs and doors.
    // Additionally, we adopt the max_speed and max_clearance of the closest edge.
    AddUndirectedEdge(p0_id, pmid_id, closest_edge.max_speed, closest_edge.max_clearance, false, false);

    // Add p1 : pmid
    AddUndirectedEdge(p1_id, pmid_id, closest_edge.max_speed, closest_edge.max_clearance, false, false);

    // Delete p0 : p1, since there is a p0 : pmid : p1 pathway now.
    DeleteUndirectedEdge(p0_id, p1_id);

    // Add pmid : v
    const uint64_t v_id = AddState(v);
    
    // NOTE: Here, we are assuming that any dynamic edges added are free of stairs and doors.
    // Additionally, we adopt the max_speed and max_clearance of the closest edge.
    AddUndirectedEdge(pmid_id, v_id, closest_edge.max_speed, closest_edge.max_clearance, false, false);

    if (kDebug) {
      printf("Adding dynamic state %f,%f (%lu) %lu %lu %lu\n",
          v.x(), v.y(), v_id, p0_id, p1_id, pmid_id);
    }
    return v_id;
  }

  // Save a V2 Map from V2 map structures.
  bool Save(const std::string& file) {
    json j;
    std::vector<json> state_jsons;
    for (const State& s: static_states) {
      state_jsons.push_back(s.toJSON());
    }
    j["nodes"] = state_jsons;


    std::vector<json> edge_jsons;
    for (const NavigationEdge& e: edges) {
      edge_jsons.push_back(e.toJSON());
    }
    j["edges"] = edge_jsons;

    std::ofstream o(file);
    o << std::setw(4) << j << std::endl;
    o.close();
    return true;
  }

  // Get the full graph in JSON (including the dynamic states)
  json GetGraphInJSON() const {
    json j;
    std::vector<json> state_jsons;
    for (const State& s: states) {
      state_jsons.push_back(s.toJSON());
    }
    j["nodes"] = state_jsons;


    std::vector<json> edge_jsons;
    for (const NavigationEdge& e: edges) {
      edge_jsons.push_back(e.toJSON());
    }
    j["edges"] = edge_jsons;

    return j;
  }

  // Get the static graph in JSON
  json GetStaticGraphInJSON() const {
    json j;
    std::vector<json> state_jsons;
    for (const State& s: static_states) {
      state_jsons.push_back(s.toJSON());
    }
    j["nodes"] = state_jsons;


    std::vector<json> edge_jsons;
    for (const NavigationEdge& e: static_edges) {
      edge_jsons.push_back(e.toJSON());
    }
    j["edges"] = edge_jsons;

    return j;
  }

  void DrawMap() {
    printf("Map:\n===Nodes===\n");
    for (const State& s : states) {
      printf("%lu: %8.3f,%8.3f\n", s.id, s.loc.x(), s.loc.y());
    }
    printf("===Edges===\n");
    for(const NavigationEdge& e : edges) {
      printf("%lu <-> %lu\n", e.s0_id, e.s1_id);
    }
  };

  // Load from a V2 map file.
  bool Load(const std::string& file) {
    static const bool kDebug = true;
    std::ifstream i(file);
    json j;
    i >> j;
    i.close();
    
    CHECK(j["nodes"].is_array());
    auto const states_json = j["nodes"];

    states.clear();
    states.resize(states_json.size());
    edges.clear();
    
    for(const json& j : states_json) {
      State s = State::fromJSON(j);
      states[s.id] = s;    
    }

    CHECK(j["edges"].is_array());
    auto const edges_json = j["edges"];

    for(const json& j : edges_json) {
      AddUndirectedEdge(j);
    }
    
    printf("Loaded %s with %lu states, %lu edges\n",
            file.c_str(),
            states.size(),
            edges.size());

    if (kDebug) DrawMap();

    static_edges = edges;
    static_states = states;
    return true;
  }

  // Save a V2 Map from V1 map structures.
  bool SaveV2FromV1(const std::string& in_file, const std::string& out_file) {
    ScopedFile fid(in_file, "r", true);
    CHECK_NOTNULL(fid());
    bool valid = true;
    uint64_t id = 0;
    float x = 0, y = 0;
    int num_edges = 0;
    int num_neighbors = 0;
    states.clear();
    while (valid &&
          !feof(fid()) &&
          fscanf(fid(), "%lu, %f, %f, %d", &id, &x, &y, &num_neighbors) == 4) {
      GrowIfNeeded(id);
      states[id] = State(id, x, y);
      for (int i = 0; i < num_neighbors; ++i) {
        uint64_t n = 0;
        if (fscanf(fid(), ", %lu", &n) == 1) {
          AddUndirectedEdge(id, n);
          ++num_edges;
        } else {
          valid = false;
          break;
        }
      }
    }

    printf("Loaded %s with %d states, %d edges\n",
            in_file.c_str(),
            static_cast<int>(states.size()),
            num_edges);

    DrawMap();
    
    Save(out_file);
    
    return true;
  }

  void GetClearanceAndSpeedFromLoc(const Eigen::Vector2f& p, 
                                   float* clearance, 
                                   float* speed) const {
    if (edges.empty()) return;
    NavigationEdge closest_edge;
    float closest_dist = FLT_MAX;
    if (!GetClosestEdge(p, &closest_edge, &closest_dist)) return;
    if (clearance) *clearance = closest_edge.max_clearance;
    if (speed) *speed = closest_edge.max_speed;
  }

  std::vector<State> states;
  std::vector<State> static_states;
  // V2 Map edges.
  std::vector<NavigationEdge> edges;
  std::vector<NavigationEdge> static_edges;

  const navigation::NavigationParameters* params_;
};


}  // namespace navigation
#endif  // GRAPH_DOMAIN_H
