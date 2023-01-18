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
\file    visualization.cc
\brief   Helper functions for visualizations
\author  Joydeep Biswas, (C) 2019
*/
//========================================================================

#include "visualization.h"

#include <algorithm>
#include <string>

#include "amrl_msgs/ColoredArc2D.h"
#include "amrl_msgs/ColoredLine2D.h"
#include "amrl_msgs/ColoredPoint2D.h"
#include "amrl_msgs/Pose2Df.h"
#include "amrl_msgs/VisualizationMsg.h"
#include "eigen3/Eigen/Dense"
#include "geometry_msgs/Point.h"
#include "ros/ros.h"

using amrl_msgs::ColoredArc2D;
using amrl_msgs::ColoredLine2D;
using amrl_msgs::ColoredPoint2D;
using amrl_msgs::Pose2Df;
using amrl_msgs::VisualizationMsg;
using Eigen::Vector2f;
using std::max;
using std::string;

namespace {
template <class T1, class T2>
void SetPoint(const T1& p1, T2* p2) {
  p2->x = p1.x();
  p2->y = p1.y();
}

}  // namespace

namespace visualization {

// Clear all elements in the message.
void ClearVisualizationMsg(VisualizationMsg& msg) {
  msg.points.clear();
  msg.lines.clear();
  msg.arcs.clear();
}

// Return new visualization message, with initialized headers and namespace.
amrl_msgs::VisualizationMsg NewVisualizationMessage(const string& frame,
                                                    const string& ns) {
  VisualizationMsg msg;
  msg.header.frame_id = frame;
  msg.header.seq = 0;
  msg.ns = ns;
  return msg;
}

void DrawPoint(const Vector2f& p, uint32_t color, VisualizationMsg& msg) {
  ColoredPoint2D point;
  SetPoint(p, &point.point);
  point.color = color;
  msg.points.push_back(point);
}

void DrawLine(const Vector2f& p0, const Vector2f& p1, uint32_t color,
              VisualizationMsg& msg) {
  ColoredLine2D line;
  SetPoint(p0, &line.p0);
  SetPoint(p1, &line.p1);
  line.color = color;
  msg.lines.push_back(line);
}

// overloaded function: RViz version
void DrawLine(const Vector2f& p0, const Vector2f& p1, uint32_t color,
              visualization_msgs::Marker& msg) {
  msg.color.a = 1.0;
  msg.color.r = static_cast<float>((color & 0xFF0000ull) >> 16) / 255.0;
  msg.color.g = static_cast<float>((color & 0xFF00ull) >> 8) / 255.0;
  msg.color.b = static_cast<float>(color & 0xFFull) / 255.0;

  geometry_msgs::Point pt;
  pt.x = p0.x();
  pt.y = p0.y();
  // pt.z = 0.3;
  msg.points.push_back(pt);
  pt.x = p1.x();
  pt.y = p1.y();
  // pt.z = 0.3;
  msg.points.push_back(pt);
}

void DrawCross(const Eigen::Vector2f& location, float size, uint32_t color,
               VisualizationMsg& msg) {
  DrawLine(location + Vector2f(size, size), location - Vector2f(size, size),
           color, msg);
  DrawLine(location + Vector2f(size, -size), location - Vector2f(size, -size),
           color, msg);
}

// overloaded funtion: RViz version
void DrawCross(const Eigen::Vector2f& location, float size, uint32_t color,
               visualization_msgs::Marker& msg, std::string frame) {

  msg.header.frame_id = frame;
  msg.header.stamp = ros::Time::now();
  msg.id = 1;
  msg.type = visualization_msgs::Marker::LINE_LIST;
  msg.action = visualization_msgs::Marker::ADD;
  msg.scale.x = 0.02;
  msg.pose.orientation.w = 1.0;

  DrawLine(location + Vector2f(size, size), location - Vector2f(size, size),
           color, msg);
  DrawLine(location + Vector2f(size, -size), location - Vector2f(size, -size),
           color, msg);
}

void DrawArc(const Vector2f& center, float radius, float start_angle,
             float end_angle, uint32_t color, VisualizationMsg& msg) {
  ColoredArc2D arc;
  SetPoint(center, &arc.center);
  arc.radius = radius;
  arc.start_angle = start_angle;
  arc.end_angle = end_angle;
  arc.color = color;
  msg.arcs.push_back(arc);
}

// Overloaded function: RViz Version
void DrawArc(const Vector2f& center, float radius, float start_angle,
             float end_angle, uint32_t color, visualization_msgs::Marker& msg, std::string frame) {

  msg.header.frame_id = frame;
  msg.header.stamp = ros::Time::now();
  msg.id = 1;
  msg.type = visualization_msgs::Marker::LINE_STRIP;
  msg.action = visualization_msgs::Marker::ADD;
  msg.pose.orientation.w = 1.0;

  static const float tesselation = 0.1;
  float dTheta = tesselation / radius;
  float z = 0.3;
  msg.scale.x = 0.02;

  // msg.color.a = static_cast<float>((color & 0xFF000000ull) >> 24) / 255.0;
  msg.color.a = 0.5;
  msg.color.r = static_cast<float>((color & 0xFF0000ull) >> 16) / 255.0;
  msg.color.g = static_cast<float>((color & 0xFF00ull) >> 8) / 255.0;
  msg.color.b = static_cast<float>(color & 0xFFull) / 255.0;

  geometry_msgs::Point pt;
  for (float theta = start_angle; theta < end_angle; theta += dTheta) {
    float c1 = cos(theta), s1 = sin(theta);
    pt.x = radius * c1 + center.x();
    pt.y = radius * s1 + center.y();
    pt.z = z;
    msg.points.push_back(pt);
  }
  float c1 = cos(end_angle), s1 = sin(end_angle);
  pt.x = radius * c1 + center.x();
  pt.y = radius * s1 + center.y();
  pt.z = z;
  msg.points.push_back(pt);
}

void DrawParticle(const Vector2f& loc, float angle, VisualizationMsg& msg) {
  fprintf(stderr, "%s Not implemented.\n", __PRETTY_FUNCTION__);
}

void DrawPathOption(const float curvature, const float distance,
                    const float clearance, const uint32_t color,
                    bool show_clearance, VisualizationMsg& msg) {
  // TODO: color by clearance.
  // static const uint32_t kPathColor = 0xC0C0C0;
  if (fabs(curvature) < 0.001) {
    DrawLine(Vector2f(0, 0), Vector2f(distance, 0), color, msg);
    if (show_clearance) {
      DrawLine(Vector2f(0, clearance), Vector2f(distance, clearance), color,
               msg);
      DrawLine(Vector2f(0, clearance), Vector2f(distance, clearance), color,
               msg);
    }
  } else {
    const float r = 1.0f / curvature;
    const Vector2f center(0, r);
    const float a = fabs(distance * curvature);
    const float a0 = ((curvature > 0.0f) ? -M_PI_2 : (M_PI_2 - a));
    const float a1 = ((curvature > 0.0f) ? (-M_PI_2 + a) : M_PI_2);
    DrawArc(center, fabs(r), a0, a1, color, msg);
    if (show_clearance) {
      DrawArc(center, max<float>(0, fabs(r) - clearance), a0, a1, color, msg);
      DrawArc(center, max<float>(0, fabs(r) + clearance), a0, a1, color, msg);
    }
  }
}

// Overloaded function: RViz Version
void DrawPathOption(const float curvature, const float distance,
                    const float clearance, const uint32_t color,
                    bool show_clearance, visualization_msgs::Marker& msg,
                    std::string frame) {
  // TODO: color by clearance.
  msg.header.frame_id = frame;
  msg.header.stamp = ros::Time::now();

  if (fabs(curvature) < 0.001) {
    msg.type = visualization_msgs::Marker::LINE_LIST;
    msg.action = visualization_msgs::Marker::ADD;
    msg.pose.orientation.w = 1.0;

    DrawLine(Vector2f(0, 0), Vector2f(distance, 0), color, msg);
    if (show_clearance) {
      DrawLine(Vector2f(0, clearance), Vector2f(distance, clearance), color,
               msg);
      DrawLine(Vector2f(0, clearance), Vector2f(distance, clearance), color,
               msg);
    }
  } else {
    msg.type = visualization_msgs::Marker::LINE_STRIP;
    msg.action = visualization_msgs::Marker::ADD;
    msg.pose.orientation.w = 1.0;

    const float r = 1.0f / curvature;
    const Vector2f center(0, r);
    const float a = fabs(distance * curvature);
    const float a0 = ((curvature > 0.0f) ? -M_PI_2 : (M_PI_2 - a));
    const float a1 = ((curvature > 0.0f) ? (-M_PI_2 + a) : M_PI_2);
    DrawArc(center, fabs(r), a0, a1, color, msg, frame);

    // TODO: correct this so it works with Marker type LINE_STRIP
    if (show_clearance) {
      DrawArc(center, max<float>(0, fabs(r) - clearance), a0, a1, color, msg, frame);
      DrawArc(center, max<float>(0, fabs(r) + clearance), a0, a1, color, msg, frame);
    }
  }
}

}  // namespace visualization
