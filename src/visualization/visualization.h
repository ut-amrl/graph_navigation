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
\file    visualization.h
\brief   Helper functions for visualizations
\author  Joydeep Biswas, (C) 2019
*/
//========================================================================
#pragma once

#include <stdint.h>
#include <string>
#include "eigen3/Eigen/Dense"

#ifdef ROS1
  #include "amrl_msgs/VisualizationMsg.h"
#else
  #include "amrl_msgs/msg/visualization_msg.hpp"
#endif

namespace visualization {

#ifdef ROS1
  using VisualizationMsgType = amrl_msgs::VisualizationMsg;
#else
  using VisualizationMsgType = amrl_msgs::msg::VisualizationMsg;
#endif

// Clear all elements in the message.
void ClearVisualizationMsg(VisualizationMsgType & msg);

// Return a new visualization message, with initialized headers and namespace.
VisualizationMsgType NewVisualizationMessage(const std::string & frame,
                                               const std::string & ns);

// Add a single point to the visualization message.
void DrawPoint(const Eigen::Vector2f & p, uint32_t color, VisualizationMsgType & msg);

// Add a single line to the visualization message.
void DrawLine(const Eigen::Vector2f & p0, const Eigen::Vector2f & p1,
              uint32_t color, VisualizationMsgType & msg);

// Add a "cross" to the visualization message.
void DrawCross(const Eigen::Vector2f & location, float size,
               uint32_t color, VisualizationMsgType & msg);

// Add an arc to the visualization message.
void DrawArc(const Eigen::Vector2f & center, float radius,
             float start_angle, float end_angle, uint32_t color,
             VisualizationMsgType & msg);

// Add a particle to the visualization message.
void DrawParticle(const Eigen::Vector2f & loc, float angle,
                  VisualizationMsgType & msg);

void DrawPathOption(const float curvature, const float distance,
                    const float clearance, const uint32_t color,
                    bool show_clearance, VisualizationMsgType & msg);

} // namespace visualization
