// Copyright 2021 srabiee@cs.utexas.edu
// Department of Computer Sciences,
// University of Texas at Austin
//
//
// This software is free: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License Version 3,
// as published by the Free Software Foundation.
//
// This software is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// Version 3 in the file COPYING that came with this distribution.
// If not, see <http://www.gnu.org/licenses/>.
// ========================================================================
/*!
\file    introspection.h
\brief   Helper functions and data structures required for integration of
         graph navigation with introspective perception.
*/
//========================================================================

#ifndef INTROSPECTION_H
#define INTROSPECTION_H

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "graph_navigation/IntrospectivePerceptionInfo.h"
#include "nlohmann/json.hpp"

namespace introspection {

using json = nlohmann::json;

struct FailureData {
  FailureData(const float& failure_prob,
              const int& failure_type,
              const float& angle,
              const Eigen::Vector2f& location)
      : failure_prob(failure_prob),
        failure_type(failure_type),
        angle(angle),
        location(location) {}
  FailureData() {}

  json toJSON() const {
    json json_obj;
    json_obj["failure_prob"] = failure_prob;
    json_obj["failure_type"] = failure_type;
    json_obj["angle"] = angle;
    json_obj["location"]["x"] = location.x();
    json_obj["location"]["y"] = location.y();
    return json_obj;
  }

  static FailureData fromJSON(const json& j) {
    FailureData failure_instance;
    failure_instance.failure_prob = j["failure_prob"].get<float>();
    failure_instance.failure_type = j["failure_type"].get<uint64_t>();
    failure_instance.angle = j["angle"].get<float>();
    failure_instance.location = Eigen::Vector2f(
        j["location"]["x"].get<float>(), j["location"]["y"].get<float>());
    return failure_instance;
  }

  float failure_prob;
  int failure_type;
  float angle;
  Eigen::Vector2f location;
};

graph_navigation::IntrospectivePerceptionInfo
GenerateIntrospectivePerceptionInfoMsg(uint64_t s0_id,
                                       uint64_t s1_id,
                                       float failure_likelihood,
                                       int failure_type);

}  // namespace introspection

#endif  // INTROSPECTION_H