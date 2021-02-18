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

namespace introspection {
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
