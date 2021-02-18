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
\file    introspection.cc
\brief   Helper functions and data structures required for integration of
         graph navigation with introspective perception.
*/
//========================================================================

#include "introspection.h"

namespace introspection {

graph_navigation::IntrospectivePerceptionInfo
GenerateIntrospectivePerceptionInfoMsg(uint64_t s0_id,
                                       uint64_t s1_id,
                                       float failure_likelihood,
                                       int failure_type) {
  graph_navigation::IntrospectivePerceptionInfo msg;
  msg.s0_id = s0_id;
  msg.s1_id = s1_id;
  msg.failure_likelihood = failure_likelihood;
  msg.failure_type = static_cast<uint8_t>(failure_type);

  return msg;
}

}  // namespace introspection
