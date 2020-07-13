// Copyright (c) 2018 joydeepb@cs.umass.edu
// College of Information and Computer Sciences,
// University of Massachusetts Amherst
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include <stdio.h>
#include <vector>
#include "eigen3/Eigen/Dense"

#include "glog/logging.h"
#include "ros/ros.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"

#ifndef GUI_HELPERS_H
#define GUI_HELPERS_H

namespace gui_helpers {

// RGBA color with each channel scaling from 0.0 to 1.0.
struct Color4f {
  float r;
  float g;
  float b;
  float a;
  Color4f() {}
  Color4f(float r, float g, float b, float a) : r(r), g(g), b(b), a(a) {}

  static const Color4f kRed;
  static const Color4f kGreen;
  static const Color4f kBlue;
  static const Color4f kWhite;
  static const Color4f kBlack;
  static const Color4f kYellow;
  static const Color4f kCyan;
  static const Color4f kMagenta;
};

// Initialize the marker to use the global "map" coordinate frame, identity
// pose, and specified color and type.
void InitializeMarker(int marker_type,
                      const Color4f& color,
                      float scale_x,
                      float scale_y,
                      float scale_z,
                      visualization_msgs::Marker* msg);

template <class Vector3>
geometry_msgs::Point StdPoint(const Vector3& v) {
  geometry_msgs::Point p;
  p.x = v.x();
  p.y = v.y();
  p.z = v.z();
  return p;
}

template <class Color>
std_msgs::ColorRGBA StdColor(const Color& color) {
  std_msgs::ColorRGBA std_color;
  std_color.a = color.a;
  std_color.r = color.r;
  std_color.g = color.g;
  std_color.b = color.b;
  return std_color;
}

template <class Vector3>
void AddLine(const Vector3& v1,
             const Vector3& v2,
             const Color4f& color,
             visualization_msgs::Marker* msg) {
  CHECK_EQ(msg->type, visualization_msgs::Marker::LINE_LIST);
  msg->points.push_back(StdPoint(v1));
  msg->points.push_back(StdPoint(v2));
  msg->colors.push_back(StdColor(color));
  msg->colors.push_back(StdColor(color));
}

template <class Vector3>
void AddPoint(const Vector3& v,
              const Color4f& color,
              visualization_msgs::Marker* msg) {
  CHECK_EQ(msg->type, visualization_msgs::Marker::POINTS);
  msg->points.push_back(StdPoint(v));
  msg->colors.push_back(StdColor(color));
}

template <class Vector3>
void AddTriangle(const Vector3& v1,
                 const Vector3& v2,
                 const Vector3& v3,
                 const Color4f& color,
                 visualization_msgs::Marker* msg) {
  CHECK_EQ(msg->type, visualization_msgs::Marker::TRIANGLE_LIST);
  msg->points.push_back(StdPoint(v1));
  msg->points.push_back(StdPoint(v2));
  msg->points.push_back(StdPoint(v3));
  msg->colors.push_back(StdColor(color));
  msg->colors.push_back(StdColor(color));
  msg->colors.push_back(StdColor(color));
}

}  // namespace gui_helpers

#endif  // GUI_HELPERS_H
